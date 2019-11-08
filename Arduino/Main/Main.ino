
#include "Protocol_2.h"
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"



void enableTorque();
void disableTorque();
void publishAngleVel();

bool protectiveStop = false;
ProtocolController* controler_ptr;

//ROS specific data:
std_msgs::Float64MultiArray angleVel_msg;
void setTorque_callback(const std_msgs::Float64MultiArray& msg);

ros::NodeHandle  nh;
ros::Publisher getAngleVel_pub("getAngleVel", &angleVel_msg);
ros::Subscriber<std_msgs::Float64MultiArray> setTorques_sub("setTorques", &setTorque_callback);


void setup()
{
  Serial.begin(230400);

  controler_ptr = new ProtocolController();

  nh.getHardware()->setBaud(230400);

  nh.initNode();
  nh.advertise(getAngleVel_pub);
  nh.subscribe(setTorques_sub);

  //allocate memory for anglevel message
  angleVel_msg.data = (float*) malloc(sizeof(float) * 8);
  //set length of msg, otherwise everything will not be sent
  angleVel_msg.data_length = 10;
}

bool led = true;

void loop()
{


  //for(int i = 1; i < 6; i++)
    //controler_ptr->setLed(i, led);

  led = !led;
  

  publishAngleVel();
  nh.spinOnce();
}

void setTorque_callback(const std_msgs::Float64MultiArray& msg)
{
  /*
  if(msg.data_length != 5)
  {
    //gotta die here!
    Serial.println("Data_length != 5, delaying to infinity!");
    delay(10000000);
  }
  else
  {
    if(!protectiveStop)
    {

    }
  }
  */
}

void enableTorque()
{
  for (int i = 1; i < 6; i++) {
    controler_ptr->toggleTorque(i, true);
  }
}

void disableTorque()
{
  for (int i = 1; i < 6; i++)
    controler_ptr->toggleTorque(i, false);
}


void publishAngleVel()
{
  for(int i = 0; i < 5; i++)
  {
      angleVel_msg.data[i*2] = controler_ptr->getPos(i+1);
      angleVel_msg.data[i*2 +1] = controler_ptr->getVel(i+1);
  }
  
  getAngleVel_pub.publish(&angleVel_msg);
}
