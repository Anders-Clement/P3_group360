//Motor baudrate
#define motorBaudrate 115200
//Pin for Request to Send.
#define RTS_pin 11

#include "Protocol_2.h"

ProtocolController* controler_ptr;

long oldTime = 0;

//used for initMotors and playMotors -- makes the arm move a little
int a = 0;
bool up = true;


void setup() {
  controler_ptr = new ProtocolController(RTS_pin, motorBaudrate);

  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  delay(100); //wait a bit to ensure Serial is ready.
}

void loop() {




  digitalWrite(13, !digitalRead(13));
}







void startMotors()
{
  for (int i = 1; i < 6; i++) {
    controler_ptr->toggleTorque(i, true);
  }
}

void stopMotors()
{
  for (int i = 1; i < 6; i++)
    controler_ptr->toggleTorque(i, false);
}

void playMotors()
{
  if (up)
    a += 2;
  else
    a -= 2;

  if (a > 300)
  {
    up = !up;
    for (int i = 1; i < 6; i++)
      controler_ptr->setLed(i, 0);
  }
  if (a < 0)
  {
    up = !up;
    for (int i = 1; i < 6; i++)
      controler_ptr->setLed(i, 1);
  }


  controler_ptr->setPos(0x01, 1300 + a);

  controler_ptr->setPos(0x02, 500 + a);
  controler_ptr->setPos(0x03, 2000 + a);

  controler_ptr->setPos(0x04, 1200 + a);
  controler_ptr->setPos(0x05, 2300 - a);
}


void killMySelf()
{
  startMotors();
  for (int i = 1; i < 6; i++)
    controler_ptr->setPosPGain(i, 50);

  long posGoalA[5]; //goal A (upright position)
  long posGoalB[5]; //goal B (by the stop button

  posGoalA[0] = 1300;
  posGoalA[1] = 500;
  posGoalA[2] = 2000;
  posGoalA[3] = 1200;
  posGoalA[4] = 2300;

  posGoalB[0] = 1814;
  posGoalB[1] = 1500;
  posGoalB[2] = 2920;
  posGoalB[3] = 300;
  posGoalB[4] = 3100;

  for (int i = 0; i < 5; i++)
    controler_ptr->setPos(i + 1, posGoalA[i]);

  delay(5000);

  for (int i = 0; i < 5; i++)
    controler_ptr->setPos(i + 1, posGoalB[i]);

  delay(1500);
  stopMotors();
}
