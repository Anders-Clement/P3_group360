#include "Buzzer.h"

Buzzer::Buzzer(int _pin)
{
  pin = _pin;
  pinMode(pin, OUTPUT);
}

void Buzzer::buzz(int mill)
{
  endTime = millis() + mill;
  digitalWrite(pin, HIGH);
}

void Buzzer::update()
{
  if(endTime < millis())
    digitalWrite(pin, LOW);
}
