



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

void trajectory_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_serial");
  ros::NodeHandle n;

  ros::Publisher serial_sub = n.advertise<std_msgs::Float32MultiArray>("/crustcrawler/getAngleVel", 1);
  ros::Subscriber serial_pub = n.subscribe("/crustcrawler/trajectory", 1, &trajectory_callback);


  ros::Rate loop_rate(100);
  while(ros::ok())
  {


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void trajectory_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  
}
