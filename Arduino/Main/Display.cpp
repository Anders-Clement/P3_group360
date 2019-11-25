#include "Display.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Display::Display()
{
  lcd = new LiquidCrystal_I2C(0x27, 2, 1, 0, 4, 5, 6, 7);  // Set the LCD I2C address
  lcd->begin(16, 4);              // initialize the lcd

  reset();
}

void Display::reset()
{
  lcd->clear();
  lcd->print("Status: ");
  lcd->setCursor(8, 0);

  lcd->setCursor(0, 1);
  lcd->print("Mode ");
  lcd->setCursor(6, 1);

  lcd->setCursor(0,2);
  lcd->print("Connected: ");
  lcd->setCursor(11, 2);
}

void Display::setMode(char* data)
{
  lcd->setCursor(5, 1);
  lcd->print("           ");
  lcd->setCursor(5, 1);
  lcd->print(data);
}

void Display::setStatus(char* data)
{
  lcd->setCursor(8, 0);
  lcd->print("        ");
  lcd->setCursor(8, 0);
  lcd->print(data);
}

void Display::setConnect(char* data)
{
  lcd->setCursor(11, 2);
  lcd->print("      ");
  lcd->setCursor(11, 2);
  lcd->print(data);
}

void Display::setLastLine(char* data)
{
  lcd->setCursor(0,3);
  lcd->print("                ");
  lcd->setCursor(0,3);
  lcd->print(data);
}
