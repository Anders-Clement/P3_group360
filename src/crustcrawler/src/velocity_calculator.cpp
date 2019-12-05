#include <ros/ros.h>
#include "std_msgs/Int16MultiArray.h"
#include "math.h"
#include <vector>
#include <time.h>
#include <iostream>
#include <fstream>


struct Vector3
{
  float x;
  float y;
  float z;
};

Vector3 operator-(Vector3 a, Vector3 b)
{
  Vector3 result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}

Vector3 f_kin(Vector3 thetas);
void check_for_zero(Vector3 &input);

void angleVel_callback(const std_msgs::Int16MultiArray::ConstPtr &msg);


std::vector<double> speeds;
Vector3 lastPos;
double lastTime;
double maxSpeed;
std::ofstream outputFile;
std::ofstream outputFileAverage;


int main(int argc, char  *argv[]) {

  lastPos.x = 0;
  lastPos.y = 0;
  lastPos.z = 0;
  lastTime = 0.001;
  maxSpeed = 0;
  ros::init(argc, argv, "velocity_calculator");
  ros::NodeHandle n;
  ros::Subscriber angleVel_sub = n.subscribe("/crustcrawler/getAngleVel", 1, &angleVel_callback);
  ros::Rate loop_rate(100);

  outputFile.open("velocities.txt");
  outputFileAverage.open("velocities_average.txt");


  std::cout << "done init" << std::endl;
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


void angleVel_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  Vector3 jointPos;
  jointPos.x = msg->data[0] / 1000.0;
  jointPos.y = msg->data[2] / 1000.0;
  jointPos.z = msg->data[4] / 1000.0;

  Vector3 curPos = f_kin(jointPos);
  Vector3 distance = curPos - lastPos;

  double length_moved = sqrt(distance.x*distance.x + distance.y*distance.y + distance.z*distance.z);
  double now = ros::Time::now().toSec();
  double d_time = lastTime - now;
  double speed = abs((length_moved/d_time)*1000.0);
  double averageSpeed;

  speeds.push_back(speed);
  std::cout << "speeds.size(): " << speeds.size() << std::endl;
  if(speeds.size() >= 4)
  {
    int endPos = speeds.size()-1;
    averageSpeed = (speeds.at(endPos) + speeds.at(endPos-1) + speeds.at(endPos-2))/3.0;
    if(abs(averageSpeed) > abs(maxSpeed))
      maxSpeed = averageSpeed;

    outputFileAverage << averageSpeed << ",";
    outputFile << speed << ",";
    //std::cout << "Avg. velocity: " << averageSpeed << ", current velocity: " << speed <<", max speed: " << maxSpeed << std::endl;
  }
  
  lastPos = curPos;
  lastTime = now;
}



void check_for_zero(Vector3 &input)
{
  float zero_value = 0.0001;
  if (input.x == 0.0)
    input.x = zero_value;
  if (input.y == 0.0)
    input.y = zero_value;
  if (input.z == 0.0)
    input.z = zero_value;
}

Vector3 f_kin(Vector3 thetas)
{
  Vector3 result;
  float pi = 3.1416;
  result.x = (11*cos(thetas.x)*cos(thetas.y + pi/2))/50 - (3*cos(thetas.x)*sin(thetas.z)*sin(thetas.y + pi/2))/20 + (3*cos(thetas.x)*cos(thetas.z)*cos(thetas.y + pi/2))/20;
  result.y = (11*cos(thetas.y + pi/2)*sin(thetas.x))/50 + (3*cos(thetas.z)*cos(thetas.y + pi/2)*sin(thetas.x))/20 - (3*sin(thetas.x)*sin(thetas.z)*sin(thetas.y + pi/2))/20;
  result.z = (11*sin(thetas.y + pi/2))/50 + (3*cos(thetas.z)*sin(thetas.y + pi/2))/20 + (3*cos(thetas.y + pi/2)*sin(thetas.z))/20 + 11.0/200.0;

  return result;
}
