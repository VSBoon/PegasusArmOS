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
int homing[nCommands] = {0}; //Transmitted by the RPi!

//Motor driving pins
const byte mPins[nCommands][2] = {{0,1},{2,3},{4,5},{6,7},{8,9},{10,11}};

//Encoder pins
const byte ePins[nCommands][2] = {{12,13},{20,21},{22,23},{19,18},{16,17},{15,14}};

//Current sensor pins
//const byte cPins[nCommands] = {30}; //CURRENTLY NON-EXISTANT!

long totCount[nCommands] = {0l};
int rotDir[nCommands] = {0}; //Actual direction
//int curr[nCommands] = {0};
char totCountBuff[nCommands][10]; //Supports long's up to +/-10.000.000 counts
char rotDirBuff[nCommands][2];
//char mSpeedBuff[nCommands][4]; //debug
char rotCCWBuff[nCommands][2]; //debug
//char currBuff[nCommands][5];

int dtComm = 200; //In milliseconds. Make sure this aligns with dtComm in Python code!
//int dtCurr = 20;
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
    if (mSpeed[i] != mSpeedPrev[i]) {
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
  
//  if (senseTimer > dtCurr) {
//    //Read current sensor
//    for(int i = 0; i < nCommands; i++) {
//      curr[i] = analogRead(cPins[i]);
//    }
//    senseTimer = 0;
//  }

  if (commTimer > dtComm) {
    //Send data to Raspberry Pi
    if (Serial.availableForWrite()) {
      for (int i = 0; i < nCommands; i++) {
        ltoa(totCount[i], totCountBuff[i], 10);
        itoa(rotDir[i], rotDirBuff[i], 10);
        //itoa(mSpeed[i], mSpeedBuff[i], 10); //Debug
        itoa(rotCCW[i], rotCCWBuff[i], 10); //Debug
        Serial.write('[');
        Serial.write(totCountBuff[i]);
        Serial.write('|');
        Serial.write(rotDirBuff[i]);
        //Serial.write('|'); //Debug
        //Serial.write(mSpeedBuff[i]); //Debug
        Serial.write('|'); //Debug
        Serial.write(rotCCWBuff[i]); //Debug
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
    volatile byte sepIndex2 = com.indexOf("|", sepIndex1+1);
    volatile byte endIndex = com.indexOf("'", startIndex + 1); //Finds last '
    mSpeed[commandIt] = com.substring(startIndex + 1, sepIndex1).toInt(); //+1 to omit first "'"
    rotCCW[commandIt] = com.substring(sepIndex1 + 1, sepIndex2).toInt(); //removed +1!
    homing[commandIt] = com.substring(sepIndex2 + 1, endIndex).toInt();
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
