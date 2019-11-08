/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

int received_int = -1;

void pc_callback(const std_msgs::Int16& msg)
{
  digitalWrite(12, msg.data);
  received_int = msg.data;
}

ros::NodeHandle  nh;

std_msgs::Int16 int_msg;
ros::Publisher pub("arduino_to_pc", &int_msg);
ros::Subscriber<std_msgs::Int16> sub("pc_to_arduino", &pc_callback);


void setup()
{
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  pinMode(12, OUTPUT);
}

void loop()
{
  int_msg.data = received_int;
  pub.publish( &int_msg );
  nh.spinOnce();
  delay(1000);
}
