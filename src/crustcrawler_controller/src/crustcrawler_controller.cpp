
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
float kp = 20;
float kv = 0;

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
void trajectoryFunk(const std_msgs::Float64MultiArray &trajectoryangles_incomming) 
{
  for (int i; i < 5; i++)
  {
    int dataindex = i * 3;
    posDesired[i] = trajectoryangles_incomming.data[dataindex];
    velDesired[i] = trajectoryangles_incomming.data[dataindex + 1];
    accDesired[i] = trajectoryangles_incomming.data[dataindex + 2];
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

//calculates the torque of the 3 differnt joints and puts it into a vector
Vector3 calculateTorque(Vector3 posError, Vector3 velError)
{
  Vector3 tau;
  Vector3 tmark;

  tmark.x = kp * posError.x + kv * velError.x + accDesired[0];
  tmark.y = kp * posError.y + kv * velError.y + accDesired[1];
  tmark.z = kp * posError.z + kv * velError.z + accDesired[2];

  float H11, H12, H13, H21, H22, H23, H31, H32, H33, VG1, VG2, VG3;

  
  H11 = 0.026 * cos(2.0 * posRobot[1] + posRobot[2]) - 0.17 * cos(2.0 * posRobot[1]) - 0.013 * sin(2.0 * posRobot[1]) + 0.026 * cos(posRobot[2]) + 0.044 * cos(2.0 * posRobot[1] + 2.0 * posRobot[2]) + 0.0016 * sin(2.0 * posRobot[1] + 2.0 * posRobot[2]) + 0.88;
  H12 = 0.26 * cos(posRobot[1]) - 0.012 * sin(posRobot[1] + posRobot[2]) - 0.025 * cos(posRobot[1] + posRobot[2]) - 0.015 * sin(posRobot[1]);
  H21 = H12;
  H13 = -0.025 * cos(posRobot[1] + posRobot[2]) - 0.012 * sin(posRobot[1] + posRobot[2]);
  H31 = H13;
  H22 = 0.052 * cos(posRobot[2]) + 0.61;
  H23 = 0.026 * cos(posRobot[2]);
  H32 = H23;
  H33 = 0.12;

  VG1 = -0.26*velRobot[1]*velRobot[1]*sin(posRobot[1])-0.012*pow(velRobot[1],2)*cos(posRobot[1] + posRobot[2]) - 0.012*pow(velRobot[2],2)*cos(posRobot[1] + posRobot[2]) + 0.025*pow(velRobot[1],2)*sin(posRobot[1] + posRobot[2]) + 0.025*pow(velRobot[2],2)*sin(posRobot[1] + posRobot[2]) - 0.015*pow(velRobot[1],2)*cos(posRobot[1]) - 0.026*velRobot[0]*velRobot[2]*sin(posRobot[2]) + 3.1e-3*velRobot[0]*velRobot[1]*cos(2.0*posRobot[1] + 2.0*posRobot[2]) + 3.1e-3*velRobot[0]*velRobot[2]*cos(2.0*posRobot[1] + 2.0*posRobot[2]) - 0.088*velRobot[0]*velRobot[1]*sin(2.0*posRobot[1] + 2.0*posRobot[2]) - 0.088*velRobot[0]*velRobot[2]*sin(2.0*posRobot[1] + 2.0*posRobot[2]) - 0.052*velRobot[0]*velRobot[1]*sin(2.0*posRobot[1] + posRobot[2]) - 0.026*velRobot[0]*velRobot[2]*sin(2.0*posRobot[1] + posRobot[2]) - 0.025*velRobot[0]*velRobot[1]*cos(2.0*posRobot[1]) + 0.33*velRobot[0]*velRobot[1]*sin(2.0*posRobot[1]) - 0.024*velRobot[1]*velRobot[2]*cos(posRobot[1] + posRobot[2]) + 0.05*velRobot[1]*velRobot[2]*sin(posRobot[1] + posRobot[2]);
  VG2 = -0.026*velRobot[2]*velRobot[2]*sin(posRobot[2]) + 0.013*pow(velRobot[0],2)*cos(2.0*posRobot[1]) + 0.22*9.81*cos(posRobot[1]) - 0.17*pow(velRobot[0],2)*sin(2.0*posRobot[1]) - 0.052*velRobot[1]*velRobot[2]*sin(posRobot[2]) - 1.6e-3*pow(velRobot[0],2)*cos(2.0*posRobot[1])*cos(2.0*posRobot[2]) + 0.044*pow(velRobot[0],2)*cos(2.0*posRobot[1])*sin(2.0*posRobot[2]) + 0.044*pow(velRobot[0],2)*cos(2.0*posRobot[2])*sin(2.0*posRobot[1]) + 1.6e-3*pow(velRobot[0],2)*sin(2.0*posRobot[1])*sin(2.0*posRobot[2])  + 0.12*9.81*cos(posRobot[1])*cos(posRobot[2]) + 0.026*pow(velRobot[0],2)*cos(2.0*posRobot[1])*sin(posRobot[2]) + 0.026*pow(velRobot[0],2)*sin(2.0*posRobot[1])*cos(posRobot[2]) - 0.12*9.81*sin(posRobot[1])*sin(posRobot[2]) ;
  VG3 =  0.013*velRobot[0]*velRobot[0]*sin(posRobot[2]) + 0.026*pow(velRobot[1],2)*sin(posRobot[2])  - 1.6e-3*pow(velRobot[0],2)*cos(2.0*posRobot[1])*cos(2.0*posRobot[2])  + 0.044*pow(velRobot[0],2)*cos(2.0*posRobot[1])*sin(2.0*posRobot[2]) + 0.044*pow(velRobot[0],2)*cos(2.0*posRobot[2])*sin(2.0*posRobot[1]) + 1.6e-3*pow(velRobot[0],2)*sin(2.0*posRobot[1])*sin(2.0*posRobot[2])  + 0.12*9.81*cos(posRobot[1])*cos(posRobot[2]) + 0.013*pow(velRobot[0],2)*cos(2.0*posRobot[1])*sin(posRobot[2]) + 0.013*pow(velRobot[0],2)*sin(2.0*posRobot[1])*cos(posRobot[2])  - 0.12*9.81*sin(posRobot[1])*sin(posRobot[2]);

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
  ros::Subscriber angleGetter_sub = n.subscribe("crustcravler/getAngleVel", 1, angleFunk); //gets curent angles
  ros::Subscriber desiredAngle_sub = n.subscribe("crustcravler/trajectory", 1, trajectoryFunk); //get desired angles

  ros::Rate loop_rate(30);
  int count = 0;

  while (ros::ok())
  {

    Vector3 posError = getErrorPos();
    Vector3 velError = getErrorVel();

    calculateTorque(posError, velError);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}