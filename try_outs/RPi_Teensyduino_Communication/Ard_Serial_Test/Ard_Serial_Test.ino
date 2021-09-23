#include <elapsedMillis.h>

/** A testing serial USB communication between the Arduino Uno R3
 *  and the Raspberry Pi Model 3b+. 
 *  Written by: Vincent Sebastiaan Boon (v_s_boon@live.nl)
 *  Date: 21-09-2021
 */

String msg;
elapsedMillis sinceCheck;

void readSerialPort() {
  msg = "";
  if (Serial.available()) {
    while (Serial.available() > 0) {
      msg += (char)Serial.read(); //Assuming characters here!
      delay(10); //Brief delay to allow the buffer to be filled
    }
    Serial.flush(); //Waits until outgoing transmission is complete.
  }
}

void echoReceive() {
  if (msg != "") {
    Serial.print("Arduino message: ");
    Serial.print(msg);
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (sinceCheck >= 100) {
    sinceCheck = 0;
    readSerialPort();
    echoReceive();
  }
}
