#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include <std_msgs/UInt8.h>
#include "math.h"

#define UPDATE_RATE 5
#define move_pose 0.05
#define tf 2.0

#ifndef MASTER
#define MASTER

using namespace std;


class masterIntelligence {
public:
    masterIntelligence();
    // intialising the virables that will be used
    int gesture = 0;
    ros::Time gen_time;
    ros::Time count_time;
    float a[4][5] = {0};
    bool update_angle_vel = true;
    float macro[4][4] = {0};
    float goalang[5];
    float goalvel[5];
    int mode = 0;

    float pos[5] = {0};
    float vel[5] = {0};
    float ang[5] = {0};


    // constucting functions
    void myo_raw_gest_str_callback(const std_msgs::String::ConstPtr& msg);
    void get_angle_vel_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);

    void checkMyo();

private:

    ros::NodeHandle n;
    ros::Publisher trajectory_pub;
    ros::Publisher joint_pub;
    ros::Publisher vibrate_pub;
    ros::Publisher mode_pub;
    ros::Publisher gesture_pub;
    ros::Subscriber gest_str_sub;
    ros::Subscriber get_angle_vel;

    // message declarations
    std_msgs::Int16MultiArray trajectories;
    sensor_msgs::JointState joint_state;
    std_msgs::UInt8 vibrate;
    std_msgs::Int16 current_mode;
    std_msgs::Int16 current_gesture;

};


#endif
