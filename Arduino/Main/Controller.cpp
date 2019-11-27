#include "Controller.h"

//gets the robots current angeles/velocities and puts into 2 arrays.
PID_Controller::PID_Controller(float* positionArray, float* velocityArray)
{
    posRobot = positionArray;
    velRobot = velocityArray;
}
//gets the robots desired pos,vel,acc for diffrernt joints and puts into 3 different arrays.
void PID_Controller::trajectoryFunk(int* trajectoryAngles_incomming)
{
  for (int i = 0; i < 5; i++)
  {
    int dataindex = i * 3;
    posDesired[i] = (float)(trajectoryAngles_incomming[dataindex] /1000.0);
    velDesired[i] = (float)(trajectoryAngles_incomming[dataindex + 1] /1000.0);
    accDesired[i] = (float)(trajectoryAngles_incomming[dataindex + 2] /1000.0);
  }

  /*
  for (int i = 3; i < 5; i++) {
    posDesired[i] = -posDesired[i];
  }
  */


}
// calculates error of desired - actual position
float* PID_Controller::getErrorPos()
{
  static float posError[5];
  for (int i = 0; i < 5; i++)
  {
    posError[i] = posDesired[i] - posRobot[i];
  }
  return posError;
}
//calculates the same for velocities
float* PID_Controller::getErrorVel()
{
  static float velError[5];
  for (int i = 0; i < 5; i++)
  {
    velError[i] = velDesired[i] - velRobot[i];
  }
  return velError;
}

void PID_Controller::resetErrorSum()
{
  for(int i = 0; i < 5; i++)
    errorSum[i] = 0.0;
}

void PID_Controller::addError()
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
float* PID_Controller::calculateTorque()
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
  float tau[5];
  float tmark[5];

  for (int i = 0; i < 5; i++)
  {
    tmark[i] = kp[i] * posError[i] + kv[i] * velError[i] + errorSum[i] + accDesired[i];
  }
  //tmark[3] = kp[3] * posError[3] + accDesired[3];
  //tmark[4] = kp[4] * posError[4] + accDesired[4];

  //calculates the dynamic part
  float H11, H12, H13, H21, H22, H23, H31, H32, H33, VG1, VG2, VG3;

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

  float G1 = 0;
  float G2 = 1.0 * L2 * g * m3 * cos(correctedPosOne) + 1.0 * L2 * g * mp * cos(correctedPosOne) + 1.0 * Lc2 * g * m2 * cos(correctedPosOne) + 1.0 * Lc3 * g * m3 * cos(correctedPosOne + posRobot[2]) + 1.0 * Lcp * g * mp * cos(correctedPosOne + posRobot[2]);
  float G3 = 1.0 * Lc3 * g * m3 * cos(correctedPosOne + posRobot[2]) + 1.0 * Lcp * g * mp * cos(correctedPosOne + posRobot[2]);

  float V1 =  -1.0 * pow(L2, 2) * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 1.0 * pow(L2, 2) * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 2.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lc3 * m3 * velRobot[0] * velRobot[2] * sin(posRobot[2]) - 2.0 * L2 * Lcp * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lcp * mp * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + posRobot[2]) - 1.0 * L2 * Lcp * mp * velRobot[0] * velRobot[2] * sin(posRobot[2]) - 1.0 * pow(Lc2, 2) * m2 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) - 1.0 * pow(Lc3, 2) * m3 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 1.0 * pow(Lc3, 2) * m3 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 1.0 * pow(Lcp, 2) * mp * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) -
             1.0 * pow(Lcp, 2) * mp * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 2.6469779601696886e-23 * pow(cos(2 * posRobot[0]) - 1, 2) * pow(velRobot[1], 2) * cos(correctedPosOne) - 0.00022544253000000001 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne + 2 * posRobot[2]) - 0.00055856354 * velRobot[0] * velRobot[1] * sin(2 * correctedPosOne) + 1.1210459999999999e-5 * velRobot[0] * velRobot[1] * cos(2 * correctedPosOne + 2 * posRobot[2]) + 6.6719999999999998e-6 * velRobot[0] * velRobot[1] * cos(2 * correctedPosOne) - 0.00022544253000000001 * velRobot[0] * velRobot[2] * sin(2 * correctedPosOne + 2 * posRobot[2]) + 1.1210459999999999e-5 * velRobot[0] * velRobot[2] * cos(2 * correctedPosOne + 2 * posRobot[2]) - 3.4514999999999999e-7 * pow(velRobot[1], 2) * sin(correctedPosOne + posRobot[2]) + 4.77121e-6 * pow(velRobot[1], 2) * sin(correctedPosOne) - 2.6469779601696886e-23 * pow(velRobot[1], 2) * cos(2 * posRobot[0] - correctedPosOne) - 2.6469779601696886e-23 * pow(velRobot[1], 2) * cos(2 * posRobot[0] + correctedPosOne) + 2.7200799999999999e-6 * pow(velRobot[1], 2) * cos(correctedPosOne + posRobot[2]) - 5.8232999999999989e-7 * pow(velRobot[1], 2) * cos(correctedPosOne) - 6.9029999999999998e-7 * velRobot[1] * velRobot[2] * sin(correctedPosOne + posRobot[2]) + 5.4401599999999998e-6 * velRobot[1] * velRobot[2] * cos(correctedPosOne + posRobot[2]) - 3.4514999999999999e-7 * pow(velRobot[2], 2) * sin(correctedPosOne + posRobot[2]) + 2.7200799999999999e-6 * pow(velRobot[2], 2) * cos(correctedPosOne + posRobot[2]);

  float V2 = 0.5 * pow(L2, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 0.5 * pow(L2, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 1.0 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) - 2.0 * L2 * Lc3 * m3 * velRobot[1] * velRobot[2] * sin(posRobot[2]) - 1.0 * L2 * Lc3 * m3 * pow(velRobot[2], 2) * sin(posRobot[2]) + 1.0 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) - 2.0 * L2 * Lcp * mp * velRobot[1] * velRobot[2] * sin(posRobot[2]) - 1.0 * L2 * Lcp * mp * pow(velRobot[2], 2) * sin(posRobot[2]) + 0.5 * pow(Lc2, 2) * m2 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) + 0.5 * pow(Lc3, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.5 * pow(Lcp, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) - 2.6469779601696886e-23 * pow(1 - cos(2 * posRobot[0]), 2) * velRobot[0] * velRobot[1] * cos(correctedPosOne) + 0.00011272126500000001 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.00027928177 * pow(velRobot[0], 2) * sin(2 * correctedPosOne) - 5.6052299999999997e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne + 2 * posRobot[2]) - 3.3359999999999999e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne) - 2.6469779601696886e-23 * velRobot[0] * velRobot[1] * cos(2 * posRobot[0] - correctedPosOne) - 2.6469779601696886e-23 * velRobot[0] * velRobot[1] * cos(2 * posRobot[0] + correctedPosOne) + 5.2939559203393771e-23 * velRobot[0] * velRobot[1] * cos(correctedPosOne);

  float V3 = 0.5 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) + 0.5 * L2 * Lc3 * m3 * pow(velRobot[0], 2) * sin(posRobot[2]) + 1.0 * L2 * Lc3 * m3 * pow(velRobot[1], 2) * sin(posRobot[2]) + 0.5 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + posRobot[2]) + 0.5 * L2 * Lcp * mp * pow(velRobot[0], 2) * sin(posRobot[2]) + 1.0 * L2 * Lcp * mp * pow(velRobot[1], 2) * sin(posRobot[2]) + 0.5 * pow(Lc3, 2) * m3 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.5 * pow(Lcp, 2) * mp * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) + 0.00011272126500000001 * pow(velRobot[0], 2) * sin(2 * correctedPosOne + 2 * posRobot[2]) - 5.6052299999999997e-6 * pow(velRobot[0], 2) * cos(2 * correctedPosOne + 2 * posRobot[2]);

  tau[0] = H11 * tmark[0] + H12 * tmark[0] + H13 * tmark[0] + G1 + V1;
  tau[1] = H21 * tmark[1] + H22 * tmark[1] + H23 * tmark[1] + G2 + V2;
  tau[2] = H31 * tmark[2] + H32 * tmark[2] + H33 * tmark[2] + G3 + V3;
  tau[3] = tmark[3] * 0.0004618954*100;
  tau[4] = tmark[4] * 0.0004618954*100;

  // clamping part
  const float limit_upper[5] = {1.0, 2.0,1.0, 1.0, 1.0};//OG: {3.0, 4.0, 3.0, 2.0, 2.0};
  const float limit_lower[5] = {-1.0,-2.0,-1.0,-1.0,-1.0};//OG: {-3.0, -4.0, -3.0, -2.0, -2.0};
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
float* PID_Controller::update()
{
  addError();
  return calculateTorque();
}
