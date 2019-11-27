#include "Arduino.h"
#ifndef BUZZER_H
#define BUZZER_H


class Buzzer
{
public:
  Buzzer(int _pin);
  void buzz(int millis);
  void update();
private:
  long endTime;
  int pin;
};





#endif
