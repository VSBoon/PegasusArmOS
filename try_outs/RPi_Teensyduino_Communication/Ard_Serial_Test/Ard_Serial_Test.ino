#include <elapsedMillis.h>
elapsedMillis sinceCheck;
/** A test for serial USB communication between the Arduino Uno R3
 *  and the Raspberry Pi Model 3b+. 
 *  Written by: Vincent Sebastiaan Boon (v_s_boon@live.nl)
 *  Date: 21-09-2021
*/
String command;
const int nCommands = 6; //Number of seperate groups of data.
long mSpeed[nCommands];
int rotCCW[nCommands];



const byte mSpeedPin = 5;
const byte m1a = 7;
const byte m1b = 6;
const byte e1a = 2;
const byte e1b = 3;

int totCount[nCommands] = {0};
int rotDir[nCommands] = {0};
char totCountBuff[nCommands][4];
char rotDirBuff[nCommands][2];

int waitTime = 10;


void setup() {
  Serial.begin(115200);
  pinMode(e1a, INPUT_PULLUP);
  pinMode(e1b, INPUT_PULLUP);
  pinMode(mSpeedPin, OUTPUT);
  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(e1a), ReadE1a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e1b), ReadE1b, CHANGE);
}

void loop() {
  serialReadAndParse();
  analogWrite(mSpeedPin, mSpeed[0]);
  if (rotDir[0] != rotCCW[0] && rotCCW[0] == 0) {
    digitalWrite(m1a, LOW);
    digitalWrite(m1b, HIGH);
    
  } else if (rotDir[0] != rotCCW[0] && rotCCW[0] == 1) {
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, LOW);
  }
  if (sinceCheck > waitTime) {
    //Send data to Raspberry Pi
    if (Serial.availableForWrite()) {
      for(int i = 0; i < nCommands; i++) {
        itoa(totCount[i], totCountBuff[i], 10);
        itoa(rotDir[i], rotDirBuff[i], 10);
        Serial.write('[');
        Serial.write(totCountBuff[i]);
        Serial.write('|');
        Serial.write(rotDirBuff[i]);
        Serial.write(']');
      }
      Serial.write('\r');
      Serial.write('\n');
    }
    sinceCheck = 0;
  }
}

void parseCommand(String com) {
  /*Function to parse incoming data from the RPi. Incoming data should be
   * of the type ['mSpeed0|rotCCW0', ..., 'mSpeed1|rotCWW1'].
   * mSpeedn should be an integer between 0 and 255, and rotCCW should be 0 or 1.
   * :param com: String that is sent over Serial.
   */
  int parseIndex = 0;
  int commandIt = 0;
  while (parseIndex < com.length() && commandIt < nCommands) {
    volatile byte startIndex = com.indexOf("'", parseIndex); //Finds first '
    volatile byte sepIndex = com.indexOf("|", parseIndex);
    volatile byte endIndex = com.indexOf("'", startIndex+1); //Finds last '
    mSpeed[commandIt] = com.substring(startIndex+1, sepIndex).toInt(); //+1 to omit first "'"
    rotCCW[commandIt] = com.substring(sepIndex+1, endIndex).toInt();
    parseIndex = endIndex+1;  //Start parsing the next data group
    commandIt++;
  }
  //DEBUG
  Serial.println(com);
  Serial.println(mSpeed[0]);
}

void serialReadAndParse() {
  if(Serial.available() > 0) {
    char c = Serial.read();
    if(c == '\n') {
      //Reset & parse
      Serial.flush();
      parseCommand(command);
      command = "";
    } else {
      command += c;
    }
  }
}

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
