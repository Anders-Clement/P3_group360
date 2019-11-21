#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>



class Display {
  public:
    Display();
    void setStatus(char* data);
    void setMode(char* data);
    void setConnect(char* data);
    void setLastLine(char* data);


  private:
    LiquidCrystal_I2C* lcd;
    void reset();
};
