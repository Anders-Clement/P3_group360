#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"

using namespace std;

float angles[3];
uint8_t gesture = 0;

//Takes the quaternions from the imu and calculates the roll, pitch and yaw
void myo_raw_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  float x = msg->pose.orientation.x;
  float y = msg->pose.orientation.y;
  float z = msg->pose.orientation.z;
  float w = msg->pose.orientation.w;

  float roll, pitch, yaw;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
  pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

//checks what gesture the myo detects
void myo_raw_gest_str_callback(const std_msgs::String::ConstPtr& msg){
  string data = msg->data;
  const string known_gestures[] = {"UNKNOWN","REST","FIST","FINGERS_SPREAD","WAVE_IN","WAVE_OUT","THUMB_TO_PINKY"};

  for(int i = 0; i < 6; i++){
    if(data == known_gestures[i]){
      gesture = i;
      ROS_INFO_STREAM(data);
      return;
    }
  }
}

//Takes the current joint angles and velocities
void get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  float data[] = msg->data;
  float theta[5];
  float thetadot[5];
  int j = 0;
  for (int i = 0; i < 10; i+=2){
    data[i] = theta[j];
    data[i+1] = thetadot[j];
    j++;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber pose_sub = n.subscribe("myo_raw/pose", 10, myo_raw_pose_callback);
  ros::Subscriber gest_str_sub = n.subscribe("myo_raw/myo_gest_str", 10, myo_raw_gest_str_callback);
  ros::Subscriber get_angle_vel = n.subscribe("/crustcrawler/getAngleVel", 10, get_angle_vel_callback);

  ros::Rate loop_rate(30);

  //const double degree = M_PI/180;

  // message declarations
  sensor_msgs::JointState joint_state;

  float pos3 = 0;
  float grip = 0;

  while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(5);
    joint_state.position.resize(5);
    joint_state.name[0] ="joint1";
    joint_state.position[0] = -angles[0];
    joint_state.name[1] ="joint2";
    joint_state.position[1] = -angles[1];
    joint_state.name[2] ="joint3";
    joint_state.position[2] = pos3;
    joint_state.name[3] ="joint4";
    joint_state.position[3] = grip;
    joint_state.name[4] ="joint5";
    joint_state.position[4] = -grip;

    if (gesture == 4) {
      pos3 += 0.02;
    }
    else if (gesture == 5) {
      pos3 -= 0.02;
    }

    if (gesture == 2) {
      grip += 0.02;
    }
    else if (gesture == 3) {
      grip -= 0.02;
    }


    float a0 = theta[i];
    float a1 = thetadot[i];
    float a2 = 3/(tf12^2)*(theta[i+1]-theta[i])-2/tf12*thetadot[i]-1/tf12*thetadot[i+1];
    float a3 = -2/(tf12^3)*(theta[i+1]-theta[i])+1/(tf12^2)*(thetadot[i+1]+thetadot[i]);
    float time = 1;
    newtheta=a0+a1*t+a2*t.^2+a3*t.^3;
    newthetaDot = a1+2*a2*t+3*a3*t.^2;
    newthetaDotDot = 2*a2+6*a3*t;


    //send the joint state and transform
    joint_pub.publish(joint_state);

    // updeate and check for news in the subscribers
    ros::spinOnce();

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }


  return 0;
}
