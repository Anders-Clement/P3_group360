
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <cmath>

_Float64 posRobot[5];
_Float64 velRobot[5];
_Float64 posDesired[5];
_Float64 velDesired[5];
_Float64 accDesired[5];

float kp[3] = {1.0, 1.0, 1.0};
float kv[3] = {0.0, 0.0, 0.0};
float ki[3] = {0.0, 0.0, 0.0};
float errorSum[3] = {0.0, 0.0, 0.0};

struct Vector3
{
  float x;
  float y;
  float z;
};
//gets the robots current angeles/velocities and puts into 2 arrays.
void angleFunk(const std_msgs::Float64MultiArray &robotAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 2;
    posRobot[i] = robotAngles_incomming.data[dataindex];
    velRobot[i] = robotAngles_incomming.data[dataindex + 1];
  }
}
//gets the robots desired pos,vel,acc for diffrernt joints and puts into 3 different arrays.
void trajectoryFunk(const std_msgs::Float64MultiArray &trajectoryAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 3;
    posDesired[i] = trajectoryAngles_incomming.data[dataindex];
    velDesired[i] = trajectoryAngles_incomming.data[dataindex + 1];
    accDesired[i] = trajectoryAngles_incomming.data[dataindex + 2];
  }
}
// calculates error of desired - actual position
Vector3 getErrorPos()
{
  Vector3 posError;
  posError.x = posDesired[0] - posRobot[0];
  posError.y = posDesired[1] - posRobot[1];
  posError.z = posDesired[2] - posRobot[2];
  return posError;
}
//calculates the same for velocities
Vector3 getErrorVel()
{
  Vector3 velError;
  velError.x = velDesired[0] - velRobot[0];
  velError.y = velDesired[1] - velRobot[1];
  velError.z = velDesired[2] - velRobot[2];
  return velError;
}

void addError(Vector3 posError)
{
  float currentError[3];
  currentError[0] = posError.x;
  currentError[1] = posError.y;
  currentError[2] = posError.z;

  for (int i = 0; i < 3; i++)
  {
    errorSum[i] = errorSum[i] + currentError[i];
  }
}

//calculates the torque of the 3 differnt joints and puts it into a vector
Vector3 calculateTorque(Vector3 posError, Vector3 velError)
{
  Vector3 tau;
  Vector3 tmark;
  tmark.x = kp[0] * posError.x + kv[0] * velError.x + accDesired[0] + ki[0] * errorSum[0];
  tmark.y = kp[1] * posError.y + kv[1] * velError.y + accDesired[1] + ki[1] * errorSum[1];
  tmark.z = kp[2] * posError.z + kv[2] * velError.z + accDesired[2] + ki[2] * errorSum[2];
  float H11, H12, H13, H21, H22, H23, H31, H32, H33, VG1, VG2, VG3;

  H11 = 0.032 + 0.026 * cos(2.0 * posRobot[1] + posRobot[2]) + 0.024 * cos(2.0 * posRobot[1]) + 3.3e-6 * sin(2.0 * posRobot[1]) + 0.026 * cos(posRobot[2]) - 1.0e-35 * cos(2.0 * posRobot[1] - 2.0 * posRobot[2]) + 8.4e-3 * cos(2.0 * posRobot[1] + 2.0 * posRobot[2]) + 5.6e-6 * sin(2.0 * posRobot[1] + 2.0 * posRobot[2]);
  H12 = -4.8e-6 * cos(posRobot[1]) - 5.8e-7 * sin(posRobot[1]) + 3.5e-7 * cos(posRobot[1]) * cos(posRobot[2]) + 2.7e-6 * cos(posRobot[1]) * sin(posRobot[2]) + 2.7e-6 * cos(posRobot[2]) * sin(posRobot[1]) - 3.5e-7 * sin(posRobot[1]) * sin(posRobot[2]);
  H21 = H12;
  H13 = 3.5e-7 * cos(posRobot[1]) * cos(posRobot[2]) + 2.7e-6 * cos(posRobot[1]) * sin(posRobot[2]) + 2.7e-6 * cos(posRobot[2]) * sin(posRobot[1]) - 3.5e-7 * sin(posRobot[1]) * sin(posRobot[2]);
  H31 = H13;
  H22 = 0.064 + 0.052 * cos(posRobot[2]);
  H23 = 0.017 + 0.026 * cos(posRobot[2]);
  H32 = H23;
  H33 = 0.017;

  VG1 = 4.8e-6 * pow(velRobot[1],2) * sin(posRobot[1]) + 2.7e-6 * pow(velRobot[1],2) * cos(posRobot[1] + posRobot[2]) + 2.7e-6 * pow(velRobot[2],2) * cos(posRobot[1] + posRobot[2]) - 3.5e-7 * pow(velRobot[1],2) * sin(posRobot[1] + posRobot[2]) - 3.5e-7 * pow(velRobot[2],2) * sin(posRobot[1] + posRobot[2]) - 5.8e-7 * pow(velRobot[1],2) * cos(posRobot[1]) - 0.026 * velRobot[0] * velRobot[2] * sin(posRobot[2]) + 1.1e-5 * velRobot[0] * velRobot[1] * cos(2.0 * posRobot[1] + 2.0 * posRobot[2]) + 1.1e-5 * velRobot[0] * velRobot[2] * cos(2.0 * posRobot[1] + 2.0 * posRobot[2]) - 0.017 * velRobot[0] * velRobot[1] * sin(2.0 * posRobot[1] + 2.0 * posRobot[2]) - 0.017 * velRobot[0] * velRobot[2] * sin(2.0 * posRobot[1] + 2.0 * posRobot[2]) - 0.052 * velRobot[0] * velRobot[1] * sin(2.0 * posRobot[1] + posRobot[2]) - 0.026 * velRobot[0] * velRobot[2] * sin(2.0 * posRobot[1] + posRobot[2]) + 6.7e-6 * velRobot[0] * velRobot[1] * cos(2.0 * posRobot[1]) - 0.048 * velRobot[0] * velRobot[1] * sin(2.0 * posRobot[1]) + 5.4e-6 * velRobot[1] * velRobot[2] * cos(posRobot[1] + posRobot[2]) - 6.9e-7 * velRobot[1] * velRobot[2] * sin(posRobot[1] + posRobot[2]);
  VG2 = -0.026 * pow(velRobot[2],2) * sin(posRobot[2]) - 3.3e-6 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) + 0.22 * 9.82 * cos(posRobot[1]) + 0.024 * pow(velRobot[0],2) * sin(2.0 * posRobot[1]) - 0.052 * velRobot[1] * velRobot[2] * sin(posRobot[2]) - 5.6e-6 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * cos(2.0 * posRobot[2]) + 8.4e-3 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * sin(2.0 * posRobot[2]) + 8.4e-3 * pow(velRobot[0],2) * cos(2.0 * posRobot[2]) * sin(2.0 * posRobot[1]) + 5.6e-6 * pow(velRobot[0],2) * sin(2.0 * posRobot[1]) * sin(2.0 * posRobot[2]) + 0.12 * 9.82 * cos(posRobot[1]) * cos(posRobot[2]) + 0.026 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * sin(posRobot[2]) + 0.026 * pow(velRobot[0],2) * sin(2.0 * posRobot[1]) * cos(posRobot[2]) - 0.12 * 9.82 * sin(posRobot[1]) * sin(posRobot[2]);
  VG3 = +0.013 * pow(velRobot[0],2) * sin(posRobot[2]) + 0.026 * pow(velRobot[1],2)  * sin(posRobot[2]) - 5.6e-6 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * cos(2.0 * posRobot[2]) + 8.4e-3 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * sin(2.0 * posRobot[2]) + 8.4e-3 * pow(velRobot[0],2) * cos(2.0 * posRobot[2]) * sin(2.0 * posRobot[1]) + 5.6e-6 * pow(velRobot[0],2) * sin(2.0 * posRobot[1]) * sin(2.0 * posRobot[2]) + 0.12 * 9.82 * cos(posRobot[1]) * cos(posRobot[2]) + 0.013 * pow(velRobot[0],2) * cos(2.0 * posRobot[1]) * sin(posRobot[2]) + 0.013 * pow(velRobot[0],2) * sin(2.0 * posRobot[1]) * cos(posRobot[2]) - 0.12 * 9.82 * sin(posRobot[1]) * sin(posRobot[2]);

  tau.x = H12 * tmark.x + H12 * tmark.y + H13 * tmark.z + VG1;
  tau.y = H21 * tmark.x + H22 * tmark.y + H23 * tmark.z + VG2;
  tau.z = H31 * tmark.x + H32 * tmark.y + H33 * tmark.z + VG3;

  return tau;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher torque_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/setTorques", 1); //publishes torque
  ros::Subscriber angleGetter_sub = n.subscribe("/crustcrawler/getAngleVel", 1, angleFunk);            //gets curent angles
  ros::Subscriber desiredAngle_sub = n.subscribe("/crustcrawler/trajectory", 1, trajectoryFunk);       //get desired angles

  ros::Rate loop_rate(30);

  int count = 0;

  while (ros::ok())
  {
    Vector3 posError = getErrorPos();
    Vector3 velError = getErrorVel();
    addError(posError);

    Vector3 tau = calculateTorque(posError, velError);

    std_msgs::Float64MultiArray msg;
    msg.data.push_back(tau.x);
    msg.data.push_back(tau.y);
    msg.data.push_back(tau.z);
    msg.data.push_back(0);
    msg.data.push_back(0);
    torque_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}