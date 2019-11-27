#include "Buzzer.h"

Buzzer::Buzzer(int _pin)
{
  pin = _pin;
  pinMode(pin, OUTPUT);
}

void Buzzer::buzz(int mill)
{
  endTime = millis() + mill;
  analogWrite(pin, 5);
}

void Buzzer::buzz(int mill, int pwm)
{
  endTime = millis() + mill;
  analogWrite(pin, pwm);
}

void Buzzer::update()
{
  if(endTime < millis())
    digitalWrite(pin, LOW);
}
