#include "Arduino.h"
#include <std_msgs/Float32MultiArray.h>

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define g 9.82
#define L2 0.22
#define L3 0.15
#define Lcp L3 //originally Lcp = 0.0
#define Lc2 0.17433
#define Lc3 0.12078
#define m2 0.17226
#define m3 0.18391
#define mp 0.0002

class PID_Controller {
    float* posRobot; //Length 5
    float* velRobot; //Length 5
    float posDesired[5];
    float velDesired[5];
    float accDesired[5];

    const float kp[5] = {5.0, 10.0, 11.0, 15.0, 15.0};
    const float kv[5] = {0.0, 2.0, 4.4, 0.0, 0.0};
    const float ki[5] = {0.0, 0.2, 0.2, 0.0, 0.0};

    float errorSum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

public:

    PID_Controller(float* positionArray, float* velocityArray);
    void trajectoryFunk(float* trajectoryAngles_incomming);
    float *getErrorPos();
    float *getErrorVel();
    void addError();
    float *calculateTorque();
    float *update();
};

#endif
