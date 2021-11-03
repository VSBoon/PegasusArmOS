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
const byte ePins[nCommands][2] = {{12,13},{20,21},{22,23},{24,25},{26,27},{28,29}};

//Current sensor pins
const byte cPins[nCommands] = {14,15,16,17,18,19};

long totCount[nCommands] = {0l};
int rotDir[nCommands] = {0}; //Actual direction
int curr[nCommands] = {0};
char totCountBuff[nCommands][10]; //Supports long's up to +/-10.000.000 counts
char rotDirBuff[nCommands][2];
char homingBuff[nCommands][2];
char currBuff[nCommands][5];

int dtComm = 5; //Make sure this aligns with dtComm in Python code!
int dtCurr = 20;


void setup() {
  /* NOTE: Serial.begin() is omitted because it is unnecessary for Teensy,
   * and only slows the startup down by up to several seconds.
   */
  for (int i = 0; i < nCommands; i++) {
    pinMode(mPins[i][0], OUTPUT);
    pinMode(mPins[i][1], OUTPUT);
    pinMode(ePins[i][0], INPUT_PULLUP);
    pinMode(ePins[i][1], INPUT_PULLUP);
    pinMode(cPins[i], INPUT);
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
      if (mSpeed[i] == 0 || homing[i] == 1) { //Hard break
        digitalWrite(mPins[i][0], HIGH);
        digitalWrite(mPins[i][1], HIGH); 
      } else if (rotCCW[i] == 0) {
        analogWrite(mPins[i][1], mSpeed[i]);
        digitalWrite(mPins[i][0], LOW);
      } else if (rotCCW[i] == 1) {
        analogWrite(mPins[i][0], mSpeed[i]);
        digitalWrite(mPins[i][1], LOW);
      }
    }
  }
  mSpeedPrev[0] = mSpeed[0];
  
  if (senseTimer > dtCurr) {
    //Read homing & curr sensor
    //TO BE CHANGED INTO FOR LOOP WHEN ALL PINS ARE KNOWN
    for(int i = 0; i < nCommands; i++) {
      curr[i] = analogRead(cPins[i]);
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
        Serial.write('[');
        Serial.write(totCountBuff[i]);
        Serial.write('|');
        Serial.write(rotDirBuff[i]);
        Serial.write('|');
        Serial.write(currBuff[i]);
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
     :param com: String that is sent over Serial.
  */
  int parseIndex = 0;
  int commandIt = 0;
  //TODO: REWRITE TO INCORPORATE HOMING READING!!!
  while (parseIndex < com.length() && commandIt < nCommands) {
    volatile byte startIndex = com.indexOf("'", parseIndex); //Finds first '
    volatile byte sepIndex1 = com.indexOf("|", parseIndex);
    volatile byte sepIndex2 = com.indexOf("|", sepIndex1);
    volatile byte endIndex = com.indexOf("'", startIndex + 1); //Finds last '
    mSpeed[commandIt] = com.substring(startIndex + 1, sepIndex1).toInt(); //+1 to omit first "'"
    rotCCW[commandIt] = com.substring(sepIndex1 + 1, sepIndex2).toInt();
    homing[commandIt] = com.substring(sepIndex2 + 1, endIndex).toInt();  //HAS NOT BEEN TESTED YET!!!
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
    Only functions for the rising of one sensor are declared, as the encoder 
    resolution is rather too high than to low.
*/

void ReadE1aHigh() {
  rotDir[0] = digitalRead(ePins[0][0]) != digitalRead(ePins[0][1]); // e1a == e1b
  totCount[0] = (rotDir[0]) ? totCount[0]-1 : totCount[0]+1;
}

void ReadE2aHigh() {
  rotDir[1] = digitalRead(ePins[1][0]) != digitalRead(ePins[1][1]);
  totCount[1] = (rotDir[1]) ? totCount[1]-1 : totCount[1]+1;
}

void ReadE3aHigh() {
  rotDir[2] = digitalRead(ePins[2][0]) != digitalRead(ePins[2][1]);
  totCount[2] = (rotDir[2]) ? totCount[2]-1 : totCount[2]+1;
}

void ReadE4aHigh() {
  rotDir[3] = digitalRead(ePins[3][0]) != digitalRead(ePins[3][1]);
  totCount[3] = (rotDir[3]) ? totCount[3]-1 : totCount[3]+1;
}

void ReadE5aHigh() {
  rotDir[4] = digitalRead(ePins[4][0]) != digitalRead(ePins[4][1]);
  totCount[4] = (rotDir[4]) ? totCount[4]-1 : totCount[4]+1;
}

void ReadE6aHigh() {
  rotDir[5] = digitalRead(ePins[5][0]) != digitalRead(ePins[5][1]);
  totCount[5] = (rotDir[5]) ? totCount[5]-1 : totCount[5]+1;
}
