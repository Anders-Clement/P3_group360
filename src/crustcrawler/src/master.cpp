#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"
#include "math.h"

using namespace std;

//float angles[3];
uint8_t gesture = 0;
float theta[5];
float thetadot[5];
ros::Time gen_time;
float a0[4];
float a1[4];
float a2[4];
float a3[4];
ros::Publisher trajectory_pub;


struct Vector3
{
  float x;
  float y;
  float z;
};


Vector3 f_kin(Vector3 thetas);
Vector3 inv_kin_closest(Vector3 position, Vector3 angles);
void check_for_zero(Vector3 &input);

/*
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
*/
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


void calc_traj(){
  float t = ros::Time::now().sec - gen_time.sec;
  std_msgs::Float64MultiArray trajectories;
  trajectories.data.resize(12);
  for (int i = 0; i < 4; i++) {
    trajectories.data[i*4] = a0[i]+a1[i]*t+a2[i]*pow(t,2)+a3[i]*pow(t,3);
    trajectories.data[i*4+1] = a1[i]+2*a2[i]*t+3*a3[i]*pow(t,2);
    trajectories.data[i*4+2] = 2*a2[i]+6*a3[i]*t;
  }
  trajectory_pub.publish(trajectories);
}

//Takes the current joint angles and velocities
void get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  vector<double> data = msg->data;
  int j = 0;
  for (int i = 0; i < 10; i+=2){
    data[i] = theta[j];
    data[i+1] = thetadot[j];
    j++;
  }
  calc_traj();
}

/*
<<<<<<< HEAD
=======

>>>>>>> 4329c16904ef11c190a62d8236f99545fd7f4cb6
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;

  trajectory_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/trajectory",1);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  //ros::Subscriber pose_sub = n.subscribe("myo_raw/pose", 10, myo_raw_pose_callback);
  ros::Subscriber gest_str_sub = n.subscribe("myo_raw/myo_gest_str", 10, myo_raw_gest_str_callback);
  ros::Subscriber get_angle_vel = n.subscribe("/crustcrawler/getAngleVel", 10, get_angle_vel_callback);

  ros::Rate loop_rate(1);

  //const double degree = M_PI/180;

  // message declarations
  sensor_msgs::JointState joint_state;

  int mode = 0;
  float pos[4];
  float move_pose = 0.02;

  while (ros::ok()) {
    gen_time = ros::Time::now();

    if (gesture == 6){
      if (mode == 3){
        mode = 1;
      }
      else{
        mode += 1;
      }
      ROS_INFO_STREAM(mode);
    }
    if(gesture != 0 && gesture != 1 && gesture != 6){
      switch (mode) {
        case 1: switch(gesture){
          case 2: pos[0] += move_pose; break;
          case 3: pos[0] -= move_pose; break;
          case 4: pos[1] += move_pose; break;
          case 5: pos[1] -= move_pose; break;
        } break;
        case 2: switch(gesture){
          case 2: pos[2] += move_pose; break;
          case 3: pos[2] -= move_pose; break;
          case 4: pos[3] += move_pose; break;
          case 5: pos[3] -= move_pose; break;
        } break;
        case 3: switch(gesture){
          case 2:
            float goalang[4] = {0,0,0,0};
            float goalvel[4] = {0,0,0,0};
            for (size_t i = 0; i < 4; i++) {
            float tf = 1;
            a0[i] = theta[i];
            a1[i] = thetadot[i];
            a2[i] = 3/(pow(tf,2))*(goalang[i]-theta[i])-2/tf*thetadot[i]-1/tf*goalvel[i];
            a3[i] = -2/(pow(tf,3))*(goalang[i]-theta[i])+1/(pow(tf,2))*(goalvel[i]+thetadot[i]);

          } break;
        } break;
      }
    }

    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(5);
    joint_state.position.resize(5);
    joint_state.name[0] ="joinposRobot[0]";
    joint_state.position[0] = pos[0];
    joint_state.name[1] ="joint2";
    joint_state.position[1] = pos[1];
    joint_state.name[2] ="joint3";
    joint_state.position[2] = pos[2];
    joint_state.name[3] ="joint4";
    joint_state.position[3] = pos[3];
    joint_state.name[4] ="joint5";
    joint_state.position[4] = -pos[3];

//send the joint state and transform
    joint_pub.publish(joint_state);

// updeate and check for news in the subscribers
    ros::spinOnce();

// This will adjust as needed per iteration
    loop_rate.sleep();
  }


  return 0;
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

Vector3 inv_kin_closest(Vector3 pos, Vector3 angles)
{
  check_for_zero(angles);
  check_for_zero(pos);

  //constants
  float pi = 3.14159;
  float L2 = 0.150;
  float L1 = 0.220;
  float z1 = 0.055;

  //extra angles and lenghs
  float a1 = sqrt(pos.x*pos.x + pos.y*pos.y);
  float d1 = sqrt((pos.z-z1)*(pos.z-z1) + a1*a1);

  //more angles to calculate a solution
  float alpha = acos((d1*d1 + L1*L1 - L2*L2)/(2*L1*d1));
  float tmp = (L1*L1 + L2*L2 - d1*d1)/(2*L1*L2);
  if(tmp < -1.0)
    if(tmp + 1.0 < -0.00001)
      std::cout << "ACOS FAILURE, LESS THAN -1, OUT OF REACH? MAYBE?" << std::endl;
    else
      tmp = -1.0;
  if(tmp > 1.0)
    if(tmp - 1.0 > 0.00001)
      std::cout << "ACOS FAILURE, GREATER THAN 1, OUT OF REACH? MAYBE?" << std::endl;
    else
      tmp = 1.0;


  float beta = acos(tmp);
  float delta = atan2((pos.z - z1),a1);

  //calculate the four solutions
  Vector3 solutions[4];

  solutions[0].x = atan2(pos.y,pos.x) + pi;
  solutions[0].y = (pi/2)-(alpha+delta);
  solutions[0].z = pi-beta;

  solutions[1].x = atan2(pos.y,pos.x);
  solutions[1].y = (-pi/2)+(alpha+delta);
  solutions[1].z = -pi+beta;

  solutions[2].x = atan2(pos.y,pos.x);
  solutions[2].y = (-pi/2)-(alpha-delta);
  solutions[2].z = pi-beta;

  solutions[3].x = atan2(pos.y,pos.x) + pi;
  solutions[3].y = -((-pi/2)-(alpha-delta));
  solutions[3].z = -pi+beta;

  //find the closest solution:

  float distance[4];

  for (size_t i = 0; i < 4; i++) {
    distance[i] = 0;
    distance[i] += std::abs(solutions[i].x - angles.x);
    distance[i] += std::abs(solutions[i].y - angles.y);
    distance[i] += std::abs(solutions[i].z - angles.z);
  }

  int lowest = 0;

  for (size_t i = 0; i < 4; i++) {
    if(distance[i] < distance[lowest])
      lowest = i;
  }

  return solutions[lowest];
}
