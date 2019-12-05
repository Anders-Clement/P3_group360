#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include <std_msgs/UInt8.h>
#include "math.h"
#include <geometry_msgs/Twist.h>

#define UPDATE_RATE 20
#define move_pose 0.025
#ifndef CRUSTCRAWLER_MASTER_H
#define CRUSTCRAWLER_MASTER_H

class masterIntelligence {
public:
    masterIntelligence(ros::NodeHandle* n, int type);
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
    bool key_control = false;

    double tf = 8.0;

    float pos[5] = {0};
    float vel[5] = {0};
    float acc[5] = {0};

    // constucting functions
    void myo_raw_gest_str_callback(const std_msgs::String::ConstPtr& msg);
    void get_angle_vel_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
    void key_callback(const geometry_msgs::Twist::ConstPtr& msg);

    void handleGesture();

    //no longer used:
    //void myo_raw_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);


private:

    ros::NodeHandle* n;
    ros::Publisher trajectory_pub;
    ros::Publisher joint_pub;
    ros::Publisher vibrate_pub;
    ros::Publisher mode_pub;
    ros::Publisher gesture_pub;
    ros::Subscriber gest_str_sub;
    ros::Subscriber get_angle_vel;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;

    // message declarations
    std_msgs::Int16MultiArray trajectories_msg;
    sensor_msgs::JointState joint_state_msg;
    std_msgs::UInt8 vibrate_msg;
    std_msgs::Int16 mode_msg;
    std_msgs::Int16 gesture_msg;

};


#endif
