/* The generic encoder class contains all functionality necessary to acquire useful data from a quadrature rotary
 * encoder.
 */

#ifndef ARDUINO_DEKUT_ENCODER_H
#define ARDUINO_DEKUT_ENCODER_H

#endif //ARDUINO_DEKUT_ENCODER_H

class Encoder {
private:
    byte pinA;
    byte pinB;
    String name;
    int cpr; //Counts per revolution
    int voltage;

public:
    //Setters
    void setPinA(byte pinA_);
    void setPinB(byte pinB_);
    void setName(String name_);
    void setCpr(int cpr_);
    void setVoltage(int voltage_);

    //Getters
    byte getPinA();
    byte getPinB();
    String getName();
    int getCpr();
    int getVoltage();

    //Class methods
    

    //Constructor
    Encoder(byte pinA_, byte pinB_, String name_, int cpr_, int voltage_);
};