#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"
#include "math.h"


#ifndef MASTER
#define MASTER

struct Vector3{
float x;
float y;
float z;
};

using namespace std;

class masterIntelligence {
public:
    masterIntelligence();
    int gesture = 0;
    float theta[5];
    float thetadot[5];
    ros::Time gen_time;
    ros::Time count_time;
    float a0[4];
    float a1[4];
    float a2[4];
    float a3[4];
    float angles[3];
    float oldAngles[3];
    bool first_oldangles = true;
    float macro[4][4];

    float pos[4] = {0,0,0,0};
    bool firstRead = true;

    int mode = 0;
    float move_pose = 0.02;

    void myo_raw_gest_str_callback(const std_msgs::String::ConstPtr& msg);
    void get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void myo_raw_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void checkMyo();
    void calc_traj();

private:

    Vector3 inv_kin_closest(Vector3 pos, Vector3 angles);
    void check_for_zero(Vector3 &input);
    Vector3 f_kin(Vector3 thetas);

    ros::NodeHandle n;

    ros::Publisher trajectory_pub;
    ros::Publisher joint_pub;
    ros::Subscriber gest_str_sub;
    ros::Subscriber get_angle_vel;
    ros::Subscriber pose_sub;
};


#endif