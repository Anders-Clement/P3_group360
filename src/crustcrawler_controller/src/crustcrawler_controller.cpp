
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>
#include <cmath>

float posRobot[5];
float velRobot[5];
float posDesired[5];
float velDesired[5];
float accDesired[5];

float kp[5] = {50.0, 10.0, 11.0, 25.0, 15.0};
float kv[5] = {1.0, 2.0, 4.4, 0.0, 0.0};
float ki[5] = {0.0, 0.2, 0.2, 0.0, 0.0};
float errorSum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float clampOff[5] = {1.0, 1.0, 1.0, 1.0, 1.0};

//gets the robots current angeles/velocities and puts into 2 arrays.
void angleFunk(const std_msgs::Int16MultiArray &robotAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 2;
    posRobot[i] = robotAngles_incomming.data[dataindex]/1000.0;
    velRobot[i] = robotAngles_incomming.data[dataindex + 1]/1000.0;
  }
}
//gets the robots desired pos,vel,acc for diffrernt joints and puts into 3 different arrays.
void trajectoryFunk(const std_msgs::Int16MultiArray &trajectoryAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 3;
    posDesired[i] = trajectoryAngles_incomming.data[dataindex]/1000.0;
    velDesired[i] = trajectoryAngles_incomming.data[dataindex + 1]/1000.0;
    accDesired[i] = trajectoryAngles_incomming.data[dataindex + 2]/1000.0;
  }
}
// calculates error of desired - actual position
float *getErrorPos()
{
  static float posError[5];
  for (int i = 0; i < 5; i++)
  {
    posError[i] = posDesired[i] - posRobot[i];
  }
  return posError;
}
//calculates the same for velocities
float *getErrorVel()
{
  static float velError[5];
  for (int i = 0; i < 5; i++)
  {
    velError[i] = velDesired[i] - velRobot[i];
  }
  return velError;
}

void addError()
{
  float *posErrorptr = getErrorPos();
  float posError[5];
  for (int i = 0; i < 5; i++)
  {
    posError[i] = posErrorptr[i];
  }

  for (int i = 0; i < 5; i++)
  {
    errorSum[i] = errorSum[i] + posError[i] * ki[i] * clampOff[i];
  }
}

//calculates the torque of the 3 differnt joints and puts it into a vector
float *calculateTorque()
{

  //this section gets the error of position and velocity
  float *posErrorptr = getErrorPos();
  float posError[5];
  for (int i = 0; i < 5; i++)
  {
    posError[i] = posErrorptr[i];
  }

  float *velErrorptr = getErrorVel();
  float velError[5];
  for (int i = 0; i < 5; i++)
  {
    velError[i] = velErrorptr[i];
  }

  //this section calculates t'
  static float tau[5];
  float tmark[5];

  for (int i = 0; i < 5; i++)
  {
    tmark[i] = kp[i] * posError[i] + kv[i] * velError[i] + errorSum[i] + accDesired[i];
  }

  //calculates the dynamic part
  float H11, H12, H13, H21, H22, H23, H31, H32, H33;

  float correctedPosOne = posRobot[1] + M_PI / 2.0; //compensate for wrong 0-position

  H11 = 0.032 + 0.026 * cos(2.0 * correctedPosOne + posRobot[2]) + 0.024 * cos(2.0 * correctedPosOne) + 3.3e-6 * sin(2.0 * correctedPosOne) + 0.026 * cos(posRobot[2]) - 1.0e-35 * cos(2.0 * correctedPosOne - 2.0 * posRobot[2]) + 8.4e-3 * cos(2.0 * correctedPosOne + 2.0 * posRobot[2]) + 5.6e-6 * sin(2.0 * correctedPosOne + 2.0 * posRobot[2]);
  H12 = -4.8e-6 * cos(correctedPosOne) - 5.8e-7 * sin(correctedPosOne) + 3.5e-7 * cos(correctedPosOne) * cos(posRobot[2]) + 2.7e-6 * cos(correctedPosOne) * sin(posRobot[2]) + 2.7e-6 * cos(posRobot[2]) * sin(correctedPosOne) - 3.5e-7 * sin(correctedPosOne) * sin(posRobot[2]);
  H21 = H12;
  H13 = 3.5e-7 * cos(correctedPosOne) * cos(posRobot[2]) + 2.7e-6 * cos(correctedPosOne) * sin(posRobot[2]) + 2.7e-6 * cos(posRobot[2]) * sin(correctedPosOne) - 3.5e-7 * sin(correctedPosOne) * sin(posRobot[2]);
  H31 = H13;
  H22 = 0.064 + 0.052 * cos(posRobot[2]);
  H23 = 0.017 + 0.026 * cos(posRobot[2]);
  H32 = H23;
  H33 = 0.017;

  static float g = 9.82;
  static float L2 = 0.22;
  static float L3 = 0.15;
  static float Lcp = L3; //originally Lcp = 0.0
  static float Lc2 = 0.17433;
  static float Lc3 = 0.12078;
  static float m2 = 0.17226;
  static float m3 = 0.18391;
  static float mp = 0.0002;

  float G1 = 0;
  float G2 = 1.0 * L2 * g * m3 * cos(correctedPosOne) + 1.0 * L2 * g * mp * cos(correctedPosOne) + 1.0 * Lc2 * g * m2 * cos(correctedPosOne) + 1.0 * Lc3 * g * m3 * cos(correctedPosOne + posRobot[2]) + 1.0 * Lcp * g * mp * cos(correctedPosOne + posRobot[2]);
  float G3 = 1.0 * Lc3 * g * m3 * cos(correctedPosOne + posRobot[2]) + 1.0 * Lcp * g * mp * cos(correctedPosOne + posRobot[2]);

  float V1 = -1.0 * pow(L2, 2) * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 1.0 * pow(L2, 2) * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 2.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[2] * sin(posRobot[2]) - 2.0 * L2 * Lcp * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lcp * mp * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lcp * mp * velRobot[0] * velRobot[2] * sin(posRobot[2]) - 1.0 * pow(Lc2, 2) * m2 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 1.0 * pow(Lc3, 2) * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 1.0 * pow(Lc3, 2) * m3 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 1.0 * pow(Lcp, 2) * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) -
             1.0 * pow(Lcp, 2) * mp * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 2.6469779601696886e-23 * pow(cos(2 * posRobot[0]) - 1, 2) * pow(velRobot[1], 2) * cos(correctedPosOne) - 0.00022544253000000001 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 0.00055856354 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) + 1.1210459999999999e-5 * velRobot[0] * velRobot[1] * cos(2 * correctedPosOne + 2 * posRobot[2]) + 6.6719999999999998e-6 * velRobot[0] * velRobot[1] * cos(2 * correctedPosOne) - 0.00022544253000000001 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) + 1.1210459999999999e-5 * velRobot[0] * velRobot[2] * cos(2 * correctedPosOne + 2 * posRobot[2]) - 3.4514999999999999e-7 * pow(velRobot[1], 2) * sin(correctedPosOne + posRobot[2]) + 4.77121e-6 * pow(velRobot[1], 2) * sin(correctedPosOne) - 2.6469779601696886e-23 * pow(velRobot[1], 2) * cos(2 * posRobot[0] - correctedPosOne) - 2.6469779601696886e-23 * pow(velRobot[1], 2) * cos(2 * posRobot[0] + correctedPosOne) + 2.7200799999999999e-6 * pow(velRobot[1], 2) * cos(correctedPosOne + posRobot[2]) - 5.8232999999999989e-7 * pow(velRobot[1], 2) * cos(correctedPosOne) - 6.9029999999999998e-7 * velRobot[1] * velRobot[2] * sin(correctedPosOne + posRobot[2]) + 5.4401599999999998e-6 * velRobot[1] * velRobot[2] * cos(correctedPosOne + posRobot[2]) - 3.4514999999999999e-7 * pow(velRobot[2], 2) * sin(correctedPosOne + posRobot[2]) + 2.7200799999999999e-6 * pow(velRobot[2], 2) * cos(correctedPosOne + posRobot[2]);

  float V2 = 0.5 * pow(L2, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 0.5 * pow(L2, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 1.0 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) - 2.0 * L2 * Lc3 * m3 * velRobot[1] * velRobot[2] * sin(posRobot[2]) - 1.0 * L2 * Lc3 * m3 * pow(velRobot[2], 2) * sin(posRobot[2]) + 1.0 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) - 2.0 * L2 * Lcp * mp * velRobot[1] * velRobot[2] * sin(posRobot[2]) - 1.0 * L2 * Lcp * mp * pow(velRobot[2], 2) * sin(posRobot[2]) + 0.5 * pow(Lc2, 2) * m2 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 0.5 * pow(Lc3, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.5 * pow(Lcp, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) - 2.6469779601696886e-23 * pow(1 - cos(2 * posRobot[0]), 2) * velRobot[0] * velRobot[1] * cos(correctedPosOne) + 0.00011272126500000001 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.00027928177 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) - 5.6052299999999997e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne + 2 * posRobot[2]) - 3.3359999999999999e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne) - 2.6469779601696886e-23 * velRobot[0] * velRobot[1] * cos(2 * posRobot[0] - correctedPosOne) - 2.6469779601696886e-23 * velRobot[0] * velRobot[1] * cos(2 * posRobot[0] + correctedPosOne) + 5.2939559203393771e-23 * velRobot[0] * velRobot[1] * cos(correctedPosOne);

  float V3 = 0.5 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) + 0.5 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(posRobot[2]) + 1.0 * L2 * Lc3 * m3 * pow(velRobot[1], 2) * sin(posRobot[2]) + 0.5 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) + 0.5 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(posRobot[2]) + 1.0 * L2 * Lcp * mp * pow(velRobot[1], 2) * sin(posRobot[2]) + 0.5 * pow(Lc3, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.5 * pow(Lcp, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.00011272126500000001 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) - 5.6052299999999997e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne + 2 * posRobot[2]);

  tau[0] = H11 * tmark[0] + H12 * tmark[0] + H13 * tmark[0] + G1 + V1;
  tau[1] = H21 * tmark[1] + H22 * tmark[1] + H23 * tmark[1] + G2 + V2;
  tau[2] = H31 * tmark[2] + H32 * tmark[2] + H33 * tmark[2] + G3 + V3;
  tau[3] = tmark[3] * 0.0004618954 * 100;
  tau[4] = tmark[4] * 0.0004618954 * 100;

  // calmping part
  float limit_upper[5] = {3.0, 4.0, 3.0, 2.0, 2.0};
  float limit_lower[5] = {-3.0, -4.0, -3.0, -2.0, -2.0};
  static float output[5];
  bool limit_bool[5];
  bool sign_bool[5];

  for (int i = 0; i < 5; i++)
  {
    output[i] = tau[i];
    limit_bool[i] = false;
    sign_bool[i] = false;
  }

  for (int i = 0; i < 5; i++)
  {
    if (tau[i] > limit_upper[i])
    {
      output[i] = limit_upper[i];
      limit_bool[i] = true;
    }
    else if (tau[i] < limit_lower[i])
    {
      output[i] = limit_lower[i];
      limit_bool[i] = true;
    }
    else
    {
      limit_bool[i] = false;
    }

    if (posError[i] >= 0 && output[i] >= 0 || posError[i] < 0 && output < 0)
    {
      sign_bool[i] = true;
    }

    if (limit_bool[i] && sign_bool[i])
    {
      clampOff[i] = 0.0;
    }
    else
    {
      clampOff[i] = 1.0;
    }
  }

  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  ros::Publisher torque_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/setTorques", 1); //publishes torque
  ros::Subscriber angleGetter_sub = n.subscribe("/crustcrawler/getAngleVel", 1, angleFunk);            //gets curent angles
  ros::Subscriber desiredAngle_sub = n.subscribe("/crustcrawler/trajectory", 1, trajectoryFunk);       //get desired angles

  ros::Rate loop_rate(20);

  int count = 0;

  while (ros::ok())
  {

    addError();

    float *tauPtr = calculateTorque();
    float tauCalculated[5];
    for (int i = 0; i < 5; i++)
    {
      tauCalculated[i] = tauPtr[i];
    }

    std_msgs::Float64MultiArray msg;
    msg.data.push_back(tauCalculated[0]);
    msg.data.push_back(tauCalculated[1]);
    msg.data.push_back(tauCalculated[2]);
    msg.data.push_back(tauCalculated[3]);
    msg.data.push_back(tauCalculated[4]);
    torque_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
}
