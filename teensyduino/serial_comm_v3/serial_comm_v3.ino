#include <elapsedMillis.h>
#include<stdint.h>
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
const int nCommands = 5; //Number of seperate groups of data.
int mSpeed[nCommands];
int rotCCW[nCommands];
const byte homePin1 = 8;
const byte currPin1 = 14; //A0
const byte m1a = 6;
const byte m1b = 5;
const byte e1a = 2;
const byte e1b = 3;

int32_t totCount[nCommands] = {0}; //Couldn't get 'long' to work
int rotDir[nCommands] = {0};
int curr[nCommands] = {0};
int homing[nCommands] = {0};
char totCountBuff[nCommands][4];
char rotDirBuff[nCommands][2];
char homingBuff[nCommands][2];
char currBuff[nCommands][5];

int dtComm = 4; //Make sure this aligns with dtComm in Python code!
int dtCurrHome = 20;


void setup() {
  Serial.begin(115200);
  pinMode(currPin1, INPUT_PULLUP);
  pinMode(homePin1, INPUT_PULLUP);
  pinMode(e1a, INPUT_PULLUP);
  pinMode(e1b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(e1a), ReadE1a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e1b), ReadE1b, CHANGE);
  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);

}

void loop() {
  serialReadAndParse();
  if (mSpeed[0] == 0) { //Hard break
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, HIGH);
  } else if (rotDir[0] != rotCCW[0] && rotCCW[0] == 0) {
  digitalWrite(m1a, LOW);
  analogWrite(m1b, mSpeed[0]);
  } else if (rotDir[0] != rotCCW[0] && rotCCW[0] == 1) {
  analogWrite(m1a, mSpeed[0]);
  digitalWrite(m1b, LOW);
  }
  if (senseTimer > dtCurrHome) {
    //Read homing & curr sensor
    //TO BE CHANGED INTO FOR LOOP WHEN ALL PINS ARE KNOWN
    curr[0] = analogRead(currPin1);
    homing[0] = digitalRead(homePin1);
    senseTimer = 0;
  }
  if (commTimer > dtComm) {
    //Send data to Raspberry Pi
    if (Serial.availableForWrite()) {
      for (int i = 0; i < nCommands; i++) {
        itoa(totCount[i], totCountBuff[i], 10);
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
        Serial.write(homingBuff[i]);
        Serial.write(']');
      }
      Serial.write('\r');
      Serial.write('\n');
    }
    commTimer = 0;
  }
}

void parseCommand(String com) {
  /*Function to parse incoming data from the RPi. Incoming data should be
     of the type ['mSpeed0|rotCCW0', ..., 'mSpeed1|rotCWW1'].
     mSpeedn should be an integer between 0 and 255, and rotCCW should be 0 or 1.
     :param com: String that is sent over Serial.
  */
  int parseIndex = 0;
  int commandIt = 0;
  while (parseIndex < com.length() && commandIt < nCommands) {
    volatile byte startIndex = com.indexOf("'", parseIndex); //Finds first '
    volatile byte sepIndex = com.indexOf("|", parseIndex);
    volatile byte endIndex = com.indexOf("'", startIndex + 1); //Finds last '
    mSpeed[commandIt] = com.substring(startIndex + 1, sepIndex).toInt(); //+1 to omit first "'"
    rotCCW[commandIt] = com.substring(sepIndex + 1, endIndex).toInt();
    parseIndex = endIndex + 1; //Start parsing the next data group
    commandIt++;
  }
}

void serialReadAndParse() {
  /* Checks for new data in the serial buffer until an end-of-line character.
     Consequently parses it using the parseCommand() function.
  */
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      //Reset & parse
      Serial.flush();
      parseCommand(command);
      command = "";
    } else {
      command += c;
    }
  }
}

/* Encoder functions change the total encoder count if the interrupt pin
    for an encoder sensor is triggered, based on the direction of
    rotation, which is determined by use of quadrature encoder boolean logic.
*/
void ReadE1a() {
  rotDir[0] = digitalRead(e1a) == digitalRead(e1b);
  if (!rotDir[0]) {
    totCount[0]++;
  } else {
    totCount[0]--;
  }
}

void ReadE1b() {
  rotDir[0] = digitalRead(e1a) != digitalRead(e1b);
  if (!rotDir[0]) {
    totCount[0]++;
  } else {
    totCount[0]--;
  }
}
