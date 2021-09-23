#include "encoder.h"

void Encoder::setPinA(byte pinA_) {
    pinA = pinA_;
}
void Encoder::setPinB(byte pinB_) {
    pinB = pinB_;
}
void Encoder::setName(String name_) {
    name = name_;
}
void Encoder::setCpr(int cpr_) {
    cpr = cpr_;
}
void Encoder::setVoltage(int voltage_) {
    voltage = voltage_;
}

//Getters
byte Encoder::getPinA() {
    return pinA;
}
byte Encoder::getPinB() {
    return pinB;
}
String Encoder::getName() {
    return name;
}
int Encoder::getCpr() {
    return cpr;
}
int Encoder::getVoltage() {
    return voltage;
}

//Class methods


//Constructor
Encoder::Encoder(byte pinA_, byte pinB_, String name_, int cpr_, int voltage_) {
    setPinA(pinA_);
    setPinB(pinB_);
    setName(name_);
    setCpr(cpr_);
    setVoltage(voltage_);
}

