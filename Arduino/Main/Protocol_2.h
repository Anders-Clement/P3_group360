#ifndef PROTOCAL_H
#define PROTOCAL_H
#include <Arduino.h>

#define RTS_Pin 11
#define motorBaudrate 1000000


class ProtocolController {
  public:

    //Init thing til classen. (Constructor)
    ProtocolController();

    //Pinger den given motor.
    bool ping(unsigned char address);

    //Set den gældende led til ledStatus boolean.
    void setLed(unsigned char address, bool ledStatus);
    //Henter ledens status fra motoren.
    bool getLed(unsigned char address);

    //tænd eller sluk for torque indistillingen.
    void toggleTorque(unsigned char address, bool onTrue);
    //set operations mode, 0 = current mode, 1 = velocity mode, 3 = position mode (default).
    void setOperatingMode(unsigned char address, unsigned char mode);

    //Function til at sætte motor torque.
    void setGoalCurrent(unsigned char address, short current);
    //Function til at sætte motor torque.
    void setCurrentLimit(unsigned char address, short current);
    //set torque, requires current speed of joint
    void setTorque(unsigned char address, short torque, short current_omega);

    //Set position af den pågældende motor.
    void setPos(unsigned char address, long setPosition);
    //Get motor position
    long getPos(unsigned char address);
    //get motor velocity
    long getVel(unsigned char address);

    //Set position PGain
    void ProtocolController::setPosPGain(unsigned char addres, int pGain);




  private:
    //Function til at opdatere crc felterne i vores besked. Den bliver brugt intern i send og read functionerne.
    unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

    //Function til at skrive over RS485 med Protocol 2.0.
    //Bruges med Address, {instruks, param1, param2, .... , paramN}, størrelse på byte.
    bool writeFunction(unsigned char address, unsigned char *data_blk_ptr, unsigned short data_blk_size);

    //Function til at læse fra en motor, kommer tilbage med værdien af feltet.
    long readFunction(unsigned char address, int tableAdress, int dataLength);

    //Fortæller om den sidste besked blev modtaget okay.
    bool writeReturn = false;
    //Array der holder den sidste modtaget pakke.
    unsigned char packageBuffer[255];
    //Int der fortæller hvor lang beskeden i bufferen er.
    int packageBufferLength = 0;
};

#endif
