#include <elapsedMillis.h>
elapsedMillis commTimer;
elapsedMillis senseTimer;
/*  This code is to be used with the 'serial_comm.py' code
    (or its derivatives) of the PegasusArmOS repository.
    Allows data manipulation on the main microcontroller,
    while the local microcontroller captures & sends data
    as quickly as possible.
    Written by: Vincent Sebastiaan Boon (v_s_boon@live.nl)
    Date: 18-10-2021
*/
String command;
const int nCommands = 6; //Number of seperate groups of data.
int mSpeed[nCommands] = {0};
int mSpeedPrev[nCommands] = {0};
int rotCCW[nCommands]; //Desired direction

//Motor driving pins
const byte mPins[nCommands][2] = {};

//Encoder pins
const byte ePins[nCommands][2] = {{1,2}, {4,10}, {13,14}, {16, 17}, {19, 20}, {22, 23}};

//Current sensor pins
const byte cPins[nCommands] = {38, 39, 40, 41, 14, 15};

//Homing pins
const byte hPins[nCommands] = {3, 11, 9, 26, 28}; //Ensure connection to 3.3V pin actually goes to pin 9!!!

long totCount[nCommands] = {0l};
int rotDir[nCommands] = {0}; //Actual direction
int curr[nCommands] = {0}; 
int currErr = 0;
int currDes = 0;
int currVal = 0;
int currP = 0;
int currD = 0;
float kp = 1; //To be tweaked!
float kd = 0.01; //To be tweaked!
int currErrPrev = 0; 
volatile byte homing[nCommands] = {0}; //TODO: Check if 0 or 1.
char totCountBuff[nCommands][10]; //Supports long's up to +/-10.000.000 counts
char rotDirBuff[nCommands][2];
char currBuff[nCommands][5];
char homingBuff[nCommands][2];

int dtComm = 200; //In milliseconds. Make sure this aligns with dtComm in Python code!
int dtSense = 20; //In milliseconds
float errTime = 1.7; //In microseconds!


void setup() {
  /* NOTE: Serial.begin() is omitted because it is unnecessary for Teensy,
   * and only slows the startup down by up to several seconds.
   */
  for (int i = 0; i < nCommands; i++) {
    pinMode(mPins[i][0], OUTPUT);
    pinMode(mPins[i][1], OUTPUT);
    pinMode(ePins[i][0], INPUT_PULLUP);
    pinMode(ePins[i][1], INPUT_PULLUP);
    pinMode(cPins[i], INPUT_PULLUP); //Check if pull-up is logical
    pinMode(hPins[i], INPUT_PULLUP); //Note: Flipped logic value!
  }
  
  attachInterrupt(digitalPinToInterrupt(ePins[0][0]), ReadE1aHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(ePins[1][0]), ReadE2aHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(ePins[2][0]), ReadE3aHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(ePins[3][0]), ReadE4aHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(ePins[4][0]), ReadE5aHigh, RISING);
  attachInterrupt(digitalPinToInterrupt(ePins[5][0]), ReadE6aHigh, RISING);
}

void loop() {
  SerialReadAndParse();
  for (int i = 0; i < nCommands; i++) {
    if (mSpeed[i] != mSpeedPrev[i]) { //For efficiency
      //TODO: CHECK IF HOMING == 1 IMPLIES NEED FOR BREAK, OR HOMING == 0!
      if (mSpeed[i] == 0) { //Soft break, should change to hard
        analogWrite(mPins[i][0], 0);
        analogWrite(mPins[i][1], 0); 
      } else if (rotCCW[i] == 0) {
        analogWrite(mPins[i][1], mSpeed[i]);
        analogWrite(mPins[i][0], 0);
      } else if (rotCCW[i] == 1) {
        analogWrite(mPins[i][0], mSpeed[i]);
        analogWrite(mPins[i][1], 0);
      }
    }
    mSpeedPrev[i] = mSpeed[i];
  }
  
  if (senseTimer > dtSense) {
    //Read current- & homing sensor
    for(int i = 0; i < nCommands; i++) {
      homing[i] = digitalRead(hPins[i]);
      //Current PD loop:
      currErrPrev = currErr
      currDes = mSpeed[i]*(2/255) //<-- Check factor PWM -> I
      //TODO: make curr[i] a running average for noise suppression!
      curr[i] = (analogRead(cPins[i])-514)*(5/514)
      currErr = currDes - curr[i]
      currP = currErr*kp
      currD = ((currErr - currErrPrev)/dtSense)*kd
      currS[i] = currDes + currP + currD
      mSpeed[i] = mSpeed[i] + int(currS[i]*(255/2) + 0.5) //+0.5 for rounding
    }
    senseTimer = 0;
  }

  if (commTimer > dtComm) {
    //Send data to Raspberry Pi
    if (Serial.availableForWrite()) {
      for (int i = 0; i < nCommands; i++) {
        ltoa(totCount[i], totCountBuff[i], 10);
        itoa(rotDir[i], rotDirBuff[i], 10);
        itoa(curr[i], currBuff[i], 10);
        itoa(homing[i], homingBuff[i], 10);
        Serial.write('[');
        Serial.write(totCountBuff[i]);
        Serial.write('|');
        Serial.write(rotDirBuff[i]);
        Serial.write('|');
        Serial.write(currBuff[i]);
        Serial.write('|');
        Serial.write(homing[i]);
        Serial.write(']');
      }
      Serial.write('\r');
      Serial.write('\n');
    }
    commTimer = 0;
  }
}

void ParseCommand(String com) {
  /*Function to parse incoming data from the RPi. Incoming data should be
     of the type ['mSpeed0|rotCCW0|homing0', ..., 'mSpeedN|rotCWWN|homingN'].
     mSpeedn should be an integer between 0 and 255, rotCCW and homing should 
     be 0 or 1.
     :param com: Data that is sent over Serial.
  */
  int parseIndex = 0;
  int commandIt = 0;
  while (parseIndex < com.length() && commandIt < nCommands) {
    volatile byte startIndex = com.indexOf("'", parseIndex); //Finds first '
    volatile byte sepIndex1 = com.indexOf("|", parseIndex);
    volatile byte endIndex = com.indexOf("'", startIndex + 1); //Finds last '
    mSpeed[commandIt] = com.substring(startIndex + 1, sepIndex1).toInt(); //+1 to omit first "'"
    rotCCW[commandIt] = com.substring(sepIndex1 + 1, endIndex).toInt(); //removed +1!
    parseIndex = endIndex + 1; //Start parsing the next data group
    commandIt++;
  }
}

void SerialReadAndParse() {
  /* Checks for new data in the serial buffer until an end-of-line character.
     Consequently parses it using the parseCommand() function.
  */
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      //Reset & parse
      Serial.flush();
      ParseCommand(command);
      command = "";
    } else {
      command += c;
    }
  }
}

/* Encoder functions change the total encoder count if the interrupt pin
    for an encoder sensor is triggered, based on the direction of
    rotation, which is determined by use of quadrature encoder boolean logic.
    A digital low-pass filter is included with a very high cut-off frequency
    (in the order of 1 MHz, set by the errTime variable) to avoid interference 
    causing false positive encoder counts.
    Only functions for the rising of one sensor are declared, as the encoder 
    resolution is rather too high than to low.
*/

void ReadE1aHigh() {
  bool state = digitalRead(ePins[0][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[0][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[0] = state != digitalRead(ePins[0][1]);
    totCount[0] = (rotDir[0]) ? totCount[0]-1 : totCount[0]+1;
  }
  
}

void ReadE2aHigh() {
  bool state = digitalRead(ePins[1][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[1][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[1] = state != digitalRead(ePins[1][1]);
    totCount[1] = (rotDir[1]) ? totCount[1]-1 : totCount[1]+1;
  }
}

void ReadE3aHigh() {
  bool state = digitalRead(ePins[2][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[2][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[2] = state != digitalRead(ePins[2][1]);
    totCount[2] = (rotDir[2]) ? totCount[2]-1 : totCount[2]+1;
  }
}

void ReadE4aHigh() {
  bool state = digitalRead(ePins[3][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[3][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[3] = state != digitalRead(ePins[3][1]);
    totCount[3] = (rotDir[3]) ? totCount[3]-1 : totCount[3]+1;
  }
}

void ReadE5aHigh() {
  bool state = digitalRead(ePins[4][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[4][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[4] = state != digitalRead(ePins[4][1]);
    totCount[4] = (rotDir[4]) ? totCount[4]-1 : totCount[4]+1;
  }
}

void ReadE6aHigh() {
  bool state = digitalRead(ePins[5][0]);
  elapsedMicros errCheck;
  while (errCheck <= errTime) { //Wait for a very brief interval
    continue;
  }
  bool stateNew = digitalRead(ePins[5][0]);
  if (state == stateNew) { //Else, the interrupt was likely an interference pulse.
    rotDir[5] = state != digitalRead(ePins[5][1]);
    totCount[5] = (rotDir[5]) ? totCount[5]-1 : totCount[5]+1;
  }
}
