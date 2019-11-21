



///////////////////////////////////////////////////////////////////////////////
/*                  TESTING PURPOSES ONLY                                    */
///////////////////////////////////////////////////////////////////////////////



#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"



void arduino_callback(const std_msgs::Int16::ConstPtr& msg)
{
  std::cout << msg->data << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_serial");
  ros::NodeHandle n;

  std_msgs::Int16 msg;

  ros::Subscriber serial_sub = n.subscribe("getAngleVel", 10, arduino_callback);
  ros::Publisher serial_pub = n.advertise<std_msgs::Int16>("pc_to_arduino", 10);


  ros::Rate loop_rate(1);
  int led = 1;
  while(ros::ok())
  {
    if(led == 1)
      led = 0;
    else
      led = 1;

    msg.data = led;

    serial_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
