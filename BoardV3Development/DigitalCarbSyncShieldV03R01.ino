/*
 
 Copyright (c) 2014 Thomas Hogue
 
 Permission is hereby granted, free of charge, 
 to any person obtaining a copy of this software 
 and associated documentation files (the "Software"), 
 to deal in the Software without restriction, 
 including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons
 to whom the Software is furnished to do so, 
 subject to the following conditions:
 The above copyright notice and this permission notice
 shall be included in all copies or substantial portions of the Software.
 
 Digital_Carb_Sync_Shield
 
 For Arduino Uno R3
 Accepts 5-0v from 1 bar pressure sensors
 Creates arrays of values for all cylinders
 */


#include <RunningAverage.h>
#define RPMSIZE 4   // may not need if sampling at 2hz
#define PIN 3 
// the interrupt pin be careful with text substitution
#define INTERRUPT 1
/* the int associate with the pin per board
 Pin 3 is interrput 1 on the Uno
 Pin 3 is interrupt 0 on the Leonardo and probably the blendmicro
 */
const byte numReadingsMax = 100;
/* this should be maybe 5-150. Too large overflows RAM.
 readings x cylinders x 2bytes per integer = size in ram
 The higher the number,
 the more the readings will be smoothed, but the slower the output will
 respond to the input.  Using a constant rather than a normal variable let us
 use this value to determine the size of the readings array.
 */
const byte numCylindersMax = 6;
/* max is 6 on shield. Reduce to actual number of sensors 
 or calibration may be off. This sets up array sizes
 
 */
int inputPin[] = {
  2,5,1,4,0,3}; //this determines display locations
/* 
 Shield v2 sensor# labels left to right top to bottom is 2,1,0,5,4,3
 usual inputPin 2,5,1,4,0,3
 maps sensors and pins to array and display
 set the input pin array to match shield
 but it is only used per numCylinders
 These can be rearranged if desired
 to change tube placements
 board builders may tie signal out to voltage in for blank sensor pads
 */
byte rpmPer;
/*  sparks per revolution is bike specific
 it defaults to 1 which is most common
 is saved in EEPROM
 future mods may make this adjustable
 or it will be deprecated
 */
int sensitivity = 337;
/*  how fine input is parsed.
 - Must be less than 999 for LCD.
 - Must be less that 255 for serial bytes 
 - 500 makes a convenient voltage ratio
 - 337 makes it 1/10th inch of mercury per point RECOMMENDED
 - 674 makes it 5/100ths inch of mercury per point too sensitive
 */
byte calibrationFlag = 1;
/* use the first readings pass to calibrate
 1 means do calibration plus 2
 2 means do inches of mercury and cylinder sensor location displays
 then sets to zero and does it no more
 a good shield with good sensors doesn't need to calibrate
 because it introduces small integer math rounding errors
 a shield with quirky soldering may need calibration
 a shield with inconsistent sensors may need calibration
 */
// user adjustments above these lines only - be careful
// ====================================================
// ====================================================

#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8,9,10,11,12,13);
// v2 shield = 8,9,10,11,12,13
// Sainsmart LCD keypad 8,9,4,5,6,7 for testing

#include <EEPROM.h>   // to save num cyl and sparks per etc
volatile unsigned long rpmCnt=0;
volatile unsigned long rpmTot=0;
int readings[numCylindersMax][numReadingsMax];   // the readings from the analog input
byte numReadings = numReadingsMax;              // allow this to change to vary smooth vs speed
byte numCylinders = numCylindersMax;           //
long total[numCylindersMax];                  // the running total
int average[numCylindersMax];                // the running average 
int runPeak [numCylindersMax];              // running peak value
int calibrate [numCylindersMax];           // an adjustment for input voltage variance

RunningAverage cntRA(RPMSIZE);

void setup()
{
  analogReference(EXTERNAL);   // MUST CALL BEFORE ANALOGREAD !!!!!!
  //Serial.begin(9600);  // for debugging. Serial collides with noInterrupts - duh.
  //Serial.println("begin");
  //delay(3000);

  // initialize LCD
  // set up the LCD's number of columns and rows. Might want 4 rows some day
  //lcd.begin(20, 4);
  lcd.begin(16,2);
  // initialize LCD
  //  lcd.init();
  //  lcd.backlight();
  //  Serial.println("lcd");
  //  delay(1000);
  cntRA.clear();

  // new timer code
  cli(); //stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 31248; // = (16*10^6) / (1*1024) - 1 (must be <65536) 31248==2 secs
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //allow interrupts

  // end new timer code

  pinMode(PIN, INPUT);     //set the pin to input
  digitalWrite(PIN, HIGH); //not sure about this
  // PCintPort::attachInterrupt(PIN,rpmInt,RISING);  // for non hardware interrupt pins
  attachInterrupt(INTERRUPT,rpmInt,FALLING);
  // sparks per revolution. bike specific.
  //Serial.println("atc");
  //  delay(1000);
  rpmPer = EEPROM.read(1);
  //  Serial.println(rpmPer);
  //  delay(1000);
  if (rpmPer<1||rpmPer>2) {
    rpmPer=1; // if first time setting pick 1. Allow change to 2 someday.
    //    Serial.println("y");
    EEPROM.write(1,rpmPer);
  }

  // initialize all the readings to 0: 
  for (byte thisCylinder = 0; thisCylinder < numCylinders; thisCylinder++)
  {

    total[thisCylinder] = 0;
    average[thisCylinder] = 0;
    runPeak[thisCylinder] = 0;
    calibrate[thisCylinder] = 0;
    for (byte thisReading = 0; thisReading < numReadings; thisReading++)
    { 
      readings [thisCylinder][thisReading] = 0;  
    }
  }
}

void loop() {
  loadRA();
  switch (calibrationFlag)
  {
  case 0:
    for (byte thisCylinder = 0; thisCylinder < numCylinders; thisCylinder++)
    {
      // calculate the peak 
      for (int thisPeak = 0; thisPeak < numReadings; thisPeak++)
      {
        runPeak[thisCylinder] = max(runPeak[thisCylinder], readings[thisCylinder][thisPeak]);
      }
    }
    display();
    break;

  case 1:
    runCali();
  case 2:         //should drop through
    calcInhg();
    pinLocs();
    calibrationFlag=0;

  }
  /* set a short delay between
   readings to add stability
   set to approx 1ms when running live
   */
  delay (1); 
}

void loadRA(){
  /* for each reading read all cylinders then increment
   to keep coordination close to right in theory
   we are looping with index pre-set in setup, then incremented and reset here
   */
  for (byte index = 0; index < numReadings; index++)
  {
    for (byte thisCylinder = 0; thisCylinder < numCylinders; thisCylinder++)
    {

      // subtract the last reading
      // or switch logic to use runingAverage which uses real number arrays not integer
      total[thisCylinder] = total[thisCylinder]  - readings[thisCylinder][index];         
      /* read from the sensor. As vacuum increases MAP sensor output signal voltage drops
       so use map command to reverse output so that 
       more vacuum == lower voltage == higher data points
       */
      readings[thisCylinder][index] = map(analogRead(inputPin[thisCylinder]), 0, 1023, sensitivity, 0) + calibrate[thisCylinder]; 
      // add the reading to the total

      total[thisCylinder] = total[thisCylinder]  + readings[thisCylinder] [index];       

      // calculate the average: overwrites previous until last reading
      // this could use some improvement and efficiency
      if (calibrationFlag > 0) average[thisCylinder]  = total[thisCylinder]  / (index + 1); 
    }
    delay(1);
  }
}
void display(){
  //Serial.println("display");
  //delay(2000);
  // this all could use some efficiency

    String outString;
  String outString2;  
  String outNum;
  byte dispCylinder = 0;            

  //  unsigned long rpmTime = millis();   // if these are moved below the first time through is error
  //  rpmCnt=0;                // but we could maybe get rid of the delay and let 
  // the readings cycle be our rpm period. Remember to make rpmTime global.

  delay(500); 

  /* For floatiing point math
   mult revs by the conversion to mins before dividing by elapsed time
   then trim to integer
   Both time and count are inflated when multiple sparks per rev
   */
  cntRA.addValue(rpmTot);  // count per 2 secs
  float rpmFloat=cntRA.getAverage();
  //rpmFloat *= 60.0;
  //rpmFloat /= 2.0;
  rpmFloat *= 30.0; // slightly more efficient
  if (rpmPer>1) rpmFloat /= float(rpmPer);  // if multiple sparks per revolution
  lcd.clear();
  outString = "Cy1 Cy3 Cy5 RPM";
  outString2 = "Cy2 Cy4 Cy6 nnn";
  switch (numCylinders) {
  case 6:
    outNum = "";
    outNum += runPeak[5];
    outString2.replace("Cy6",outNum);
  case 5:
    outString2.replace("Cy6","");// if still there
    outNum = "";
    outNum += runPeak[4];
    outString.replace("Cy5",outNum);
  case 4:
    outString2.replace("Cy6","");// if still there
    outString.replace("Cy5","");// if still there
    outNum = "";
    outNum += runPeak[3];
    outString2.replace("Cy4",outNum);
  case 3:
    outNum = "";
    outNum += runPeak[2];
    outString.replace("Cy3",outNum);
  case 2:
    outNum = "";
    outNum += runPeak[1];
    outString2.replace("Cy2",outNum);
  default:
    outNum = "";
    outNum += runPeak[0];
    outString.replace("Cy1",outNum);
    // load rpm display here
    if (rpmTot>0) {
      char buffer[10];
      outNum = "";
      outNum += dtostrf(rpmFloat, 3, 0, buffer);
      outString2.replace("nnn",outNum);
    }
    else {
      outString.replace("RPM","");// blank out if not in use
      outString2.replace("nnn","");
    }
    lcd.setCursor(0,0);
    lcd.print (outString);
    lcd.setCursor(0,1);
    lcd.print (outString2);
  }


  // delay (500);

  // reset all the readings to 0: 
  for (dispCylinder = 0; dispCylinder < numCylinders; dispCylinder++)
  {
    runPeak[dispCylinder] = 0;  // must reset this. others are kept running for now
    /*   total[dispCylinder] = 0;
     average[dispCylinder] = 0;       
     for (int dispReading = 0; dispReading < numReadings; dispReading++)
     { readings [dispCylinder][dispReading] = 0;  
     }
     */
  }
}

void runCali(){
  //Serial.println("runCali");
  String outString;
  byte calibrateCylinder;             // 
  int calibrationMean;                 // the input adjustment added to each reading

  calibrationMean = 0;
  for (calibrateCylinder = 0; calibrateCylinder < numCylinders; calibrateCylinder++)
  {
    calibrationMean += average[calibrateCylinder];
  }
  calibrationMean /= numCylinders;
  for (calibrateCylinder = 0; calibrateCylinder < numCylinders; calibrateCylinder++)
  {
    calibrate [calibrateCylinder] = calibrationMean - average [calibrateCylinder];
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  outString = "Cylinders = ";  // this really should be number of sensors
  outString += numCylinders;
  lcd.print (outString);
  lcd.setCursor(0, 1);
  outString = "Calibrate = ";
  outString += calibrationMean; // average of the averages
  lcd.print (outString);
  delay (1000);

  lcd.clear();
  lcd.setCursor(0,0);
  outString="";
  outString += calibrate[0];
  outString += " ";
  outString += calibrate[2];
  outString += " ";
  outString += calibrate[4];
  lcd.print (outString);
  lcd.setCursor(0,1);
  outString="";
  outString += calibrate[1];
  outString += " ";
  outString += calibrate[3];
  outString += " ";
  outString += calibrate[5];
  lcd.print (outString);
  delay (1000);
}

void pinLocs(){
  // show where cylinders are (add pin data later)
  String outString = "Cy1 Cy3 Cy5 RPM";
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print (outString);
  outString = "Cy2 Cy4 Cy6 nnn";
  lcd.setCursor(0,1);
  lcd.print (outString);
  delay (1000);
}

void calcInhg(){
  //Serial.println ("calcInhg");
  // calc and display 1 point = x inches of mercury here
  lcd.clear();
  lcd.setCursor(0,0);
  float inchesHg=0;
  inchesHg = 33.9634 / (float)sensitivity; // sensor range 115 kPa in inHg / sensitivity
  String outString = "1pt ==";
  char buffer[10];
  String inHgstr = dtostrf(inchesHg, 7, 2, buffer);
  outString += (inHgstr);
  lcd.print (outString);
  lcd.setCursor(0, 1);    
  outString = " inches Hg";
  lcd.print (outString);
  delay (1000); 
}

void rpmInt(){
  //interrupt commands here
  rpmCnt++;  
}

ISR(TIMER1_COMPA_vect){  
  //interrupt commands here
  rpmTot=rpmCnt;
  rpmCnt=0;
}






