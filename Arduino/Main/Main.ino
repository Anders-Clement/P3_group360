
#include "Protocol_2.h"
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"

#define buzz_pin 48


void enableTorque();
void disableTorque();
void publishAngleVel();
long lastMessageTime;

bool protectiveStop = false;
ProtocolController* controler_ptr;

//ROS specific data:
std_msgs::Float64MultiArray angleVel_msg;
void setTorque_callback(const std_msgs::Float64MultiArray& msg);
void setPWMMode();

ros::NodeHandle  nh;
ros::Publisher getAngleVel_pub("/crustcrawler/getAngleVel", &angleVel_msg);
ros::Subscriber<std_msgs::Float64MultiArray> setTorques_sub("/crustcrawler/setTorques", &setTorque_callback);


<<<<<<< HEAD
static float motorOffsets[5] = {M_PI*(4.0/3.0) - 0.66, M_PI/4.0, M_PI, M_PI * (3.0/4.0), M_PI};
=======
static float motorOffsets[5] = {-M_PI/2.0, M_PI/4.0, M_PI, M_PI * (3.0/4.0), M_PI};
>>>>>>> 7c1f8938c8786f9b58cccbd07ca6dbeb39e7bbca

void setup()
{
  Serial.begin(250000);

  controler_ptr = new ProtocolController();

  nh.getHardware()->setBaud(250000);

  nh.initNode();
  nh.advertise(getAngleVel_pub);
  nh.subscribe(setTorques_sub);

  //allocate memory for anglevel message
  angleVel_msg.data = (float*) malloc(sizeof(float) * 10);
  //set length of msg, otherwise everything will not be sent
  angleVel_msg.data_length = 10;

  lastMessageTime = millis();
}

bool led = true;

void loop()
{
  nh.spinOnce();

  if(millis() - lastMessageTime > 250) //no message received for 250ms, sound the alarm!
  {
    digitalWrite(buzz_pin, HIGH);
    disableTorque();
    delay(1000);
    digitalWrite(buzz_pin, LOW);
  }
}

void setTorque_callback(const std_msgs::Float64MultiArray& msg)
{
  lastMessageTime = millis();

  float thetas[5];
  float velocities[5];

  for (int i = 1; i < 6; i++)
  {
    thetas[i - 1] = (controler_ptr->getPos(i) * 0.0015336f) - motorOffsets[i-1];
    velocities[i - 1] = (float)controler_ptr->getVel(i) * 0.0229; //convert to rad/s (0.229rpm/step * 0.1(rad/sec)/rpm
  } //this for loop takes 18.3ms

  if (!protectiveStop)
  {
    for (int i = 1; i < 6; i++)
    {
      controler_ptr->setTorque(i, msg.data[i], velocities[i-1]);
    }

  }

  //publish thetas and velocities:
  for (int i = 0; i < 5; i++)
  {
    angleVel_msg.data[i * 2] = thetas[i];
    angleVel_msg.data[i * 2 + 1] = velocities[i];
  }

  getAngleVel_pub.publish(&angleVel_msg);
}

void enableTorque()
{
  for (int i = 1; i < 6; i++) {
    controler_ptr->toggleTorque(i, true);
  }
}

void setPWMMode()
{
  for(int i = 1; i < 6; i++)
    controler_ptr->setOperatingMode(i, 0xF);
}

void disableTorque()
{
  for (int i = 1; i < 6; i++)
    controler_ptr->toggleTorque(i, false);
}


void publishAngleVel()
{
  for (int i = 0; i < 5; i++)
  {
    angleVel_msg.data[i * 2] = controler_ptr->getPos(i + 1);
    angleVel_msg.data[i * 2 + 1] = controler_ptr->getVel(i + 1);
  }

  getAngleVel_pub.publish(&angleVel_msg);
}
