#include "master.h"

using namespace std;

masterIntelligence::masterIntelligence(){

  trajectory_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/trajectory", 10);
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  gest_str_sub = n.subscribe("/myo_raw/myo_gest_str", 10, &masterIntelligence::myo_raw_gest_str_callback, this);
  get_angle_vel = n.subscribe("/crustcrawler/getAngleVel", 10, &masterIntelligence::get_angle_vel_callback, this);
  pose_sub = n.subscribe("/myo_raw/pose", 10, &masterIntelligence::myo_raw_pose_callback, this);

  ros::spinOnce();
}

// Takes the quaternions from the imu and calculates the euler angles (roll, pitch and yaw)
void masterIntelligence::myo_raw_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
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

  eulerAng[0] = roll;
  eulerAng[1] = pitch;
  eulerAng[2] = yaw;
}

//checks what gesture the myo detects and changes it from string to int representation
void masterIntelligence::myo_raw_gest_str_callback(const std_msgs::String::ConstPtr &msg){

  
  string data = msg->data;
  const string known_gestures[] = {"UNKNOWN", "REST", "FIST", "FINGERS_SPREAD", "WAVE_IN", "WAVE_OUT", "THUMB_TO_PINKY"};

  // checks which gesture is currently held and gives it and int representation
  for (int i = 0; i < 7; i++){
    if (data == known_gestures[i]){
      gesture = i;
      ROS_INFO_STREAM(data);
      break;
    }
  }
  // if the gesture held is "THUMB_TO_PINKY" then change mode from 1 to 5 and then reset to 1
  if (gesture == 6){
    if (mode == 5)
      mode = 1;
    else{
      mode += 1;
    }
    modeChanged = true;
    ROS_INFO_STREAM(mode);
  }
}

//Takes the current joint angles and velocities
void masterIntelligence::get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr &msg){
  vector<double> data = msg->data;

  // takes the current pos and vel and divides them into 2 different vectors
  int j = 0;
  for (int i = 0; i < 10; i += 2){
    theta[j] = data[i];
    thetadot[j] = data[i + 1];
    j++;
  }
  /*
  // for testing in rviz
  if (firstRead){
    firstRead = false;
    for (int i = 0; i < 4; i++){
     pos[i] = theta[i];
    }
  }
  */
}

/*
void masterIntelligence::check_for_zero(Vector3 &input){
  float zero_value = 0.0001;
  if (input.x == 0.0)
  {
    input.x = zero_value;
  }
  if (input.y == 0.0)
  {
    input.y = zero_value;
  }
  if (input.z == 0.0)
  {
    input.z = zero_value;
  }
}

Vector3 masterIntelligence::f_kin(Vector3 thetas){

  Vector3 result;
  float pi = 3.1416;
  result.x = (11 * cos(thetas.x) * cos(thetas.y + pi / 2)) / 50 - (3 * cos(thetas.x) * sin(thetas.z) * sin(thetas.y + pi / 2)) / 20 + (3 * cos(thetas.x) * cos(thetas.z) * cos(thetas.y + pi / 2)) / 20;
  result.y = (11 * cos(thetas.y + pi / 2) * sin(thetas.x)) / 50 + (3 * cos(thetas.z) * cos(thetas.y + pi / 2) * sin(thetas.x)) / 20 - (3 * sin(thetas.x) * sin(thetas.z) * sin(thetas.y + pi / 2)) / 20;
  result.z = (11 * sin(thetas.y + pi / 2)) / 50 + (3 * cos(thetas.z) * sin(thetas.y + pi / 2)) / 20 + (3 * cos(thetas.y + pi / 2) * sin(thetas.z)) / 20 + 11.0 / 200.0;

  return result;
}

Vector3 masterIntelligence::inv_kin_closest(Vector3 pos, Vector3 angles){
  check_for_zero(angles);
  check_for_zero(pos);

  //constants
  float pi = 3.14159;
  float L2 = 0.150;
  float L1 = 0.220;
  float z1 = 0.055;

  //extra angles and lenghs
  float a1 = sqrt(pos.x * pos.x + pos.y * pos.y);
  float d1 = sqrt((pos.z - z1) * (pos.z - z1) + a1 * a1);

  //more angles to calculate a solution
  float alpha = acos((d1 * d1 + L1 * L1 - L2 * L2) / (2 * L1 * d1));
  float tmp = (L1 * L1 + L2 * L2 - d1 * d1) / (2 * L1 * L2);
  if (tmp < -1.0)
  {
    if (tmp + 1.0 < -0.00001)
    {
      std::cout << "ACOS FAILURE, LESS THAN -1, OUT OF REACH? MAYBE?" << std::endl;
    }
    else
    {
      tmp = -1.0;
    }
  }
  if (tmp > 1.0)
  {
    if (tmp - 1.0 > 0.00001)
    {
      std::cout << "ACOS FAILURE, GREATER THAN 1, OUT OF REACH? MAYBE?" << std::endl;
    }
    else
    {
      tmp = 1.0;
    }
  }

  float beta = acos(tmp);
  float delta = atan2((pos.z - z1), a1);

  //calculate the four solutions
  Vector3 solutions[4];

  solutions[0].x = atan2(pos.y, pos.x) + pi;
  solutions[0].y = (pi / 2) - (alpha + delta);
  solutions[0].z = pi - beta;

  solutions[1].x = atan2(pos.y, pos.x);
  solutions[1].y = (-pi / 2) + (alpha + delta);
  solutions[1].z = -pi + beta;

  solutions[2].x = atan2(pos.y, pos.x);
  solutions[2].y = (-pi / 2) - (alpha - delta);
  solutions[2].z = pi - beta;

  solutions[3].x = atan2(pos.y, pos.x) + pi;
  solutions[3].y = -((-pi / 2) - (alpha - delta));
  solutions[3].z = -pi + beta;

  //find the closest solution:

  float distance[4];

  for (size_t i = 0; i < 4; i++)
  {
    distance[i] = 0;
    distance[i] += std::abs(solutions[i].x - angles.x);
    distance[i] += std::abs(solutions[i].y - angles.y);
    distance[i] += std::abs(solutions[i].z - angles.z);
  }

  int lowest = 0;

  for (size_t i = 0; i < 4; i++)
  {
    if (distance[i] < distance[lowest])
      lowest = i;
  }

  return solutions[lowest];
}
*/

void masterIntelligence::checkMyo(){

  if (gesture != 0 && gesture != 6){ // checking that gesture is not unknown or pinky_to_thumb because pinky to thumb changes modes
    switch (mode){ // then check what mode it is in
      case 1:{ // mode 1 controlls the first two joints with the four remaining gestures that is not rest
        switch (gesture){ 
          case 1:{ break;}
          case 2:{ pos[1] += move_pose; break;}
          case 3:{ pos[1] -= move_pose; break;}
          case 4:{ pos[0] += move_pose; break;}
          case 5:{ pos[0] -= move_pose; break;}
        }
        break;
      }

      case 2:{ // mode 2 controlls the third joint and the gripper with the four remaining gestures that is not rest
        switch (gesture){
          case 1:{ break;}
          case 2:{ pos[3] += move_pose; break;}
          case 3:{ pos[3] -= move_pose; break;}
          case 4:{ pos[2] += move_pose; break;}
          case 5:{ pos[2] -= move_pose; break;}
        }
        break;
      }

      case 3:{ // mode 3 is able to set a the current joint angles macro to a gesture by holding the gesture for 2 seconds
        switch(gesture){
          case 1:{ break;} // if gesture is rest break
          case 2:{ 
            count_time = ros::Time::now(); // resets counter timer
            while (gesture == 2){ // enters a while loop to be able check if the gesture is held
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){ // if the gesture is held for 2 sec 
                for (size_t i = 0; i < 4; i++){ // set the macro to the current position for all joints
                  macro[0][i] = theta[i];
                 // macro[0][i] = 0.0;
                }
                ROS_INFO_STREAM("macro 0 set:"); // 
                break;
              }
              ros::spinOnce();
            }
            break;
          }
          case 3:{ // same as case 2 just with gesture 3
            count_time = ros::Time::now();
            while (gesture == 3){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 4; i++){
                  macro[1][i] = theta[i];
                  //macro[1][i] = pos[i];
                }
                ROS_INFO_STREAM("macro 1 set:");
                break;
              }
              ros::spinOnce();
            }
            break;
          }
          case 4:{ // same as case 2 just with gesture 4
            count_time = ros::Time::now();
            while (gesture == 4){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 4; i++){
                  macro[2][i] = theta[i];
                  //macro[2][i] = pos[i];
                }
                ROS_INFO_STREAM("macro 2 set:");
                break;
              }
              ros::spinOnce();
            }
            break;
          }
          case 5:{ // same as case 2 just with gesture 5
            count_time = ros::Time::now();
            while (gesture == 5){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 4; i++){
                  macro[3][i] = theta[i];
                  //macro[3][i] = pos[i];
                }
                ROS_INFO_STREAM("macro 3 set:");
                break;
              }
              ros::spinOnce();
            }
            break;
          }
        }
        break;
      }
      // mode 4 can recall saved macros by using the gesture is have been saved under
      case 4:{ 
        if (ros::Time::now().toSec() - gen_time.toSec() >= 1.0){ // to ensure this mode only updates once per second we check the timer
          gen_time = ros::Time::now(); // resets timer
          for (size_t i = 0; i < 4; i++){ // sets the goal position to current position
            goalang[i] = theta[i];
            goalvel[i] = 0;

              //for testing in rviz
            //goalang[i] = pos[i];
          }
          switch (gesture){
            case 1:{ break;}
            case 2:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 4; i++){ 
                goalang[i] = 0.0;
              }
              break;
            }
            case 3:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[1][i];
              }
              break;
            }
            case 4:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[2][i];
              }
              break;
            }
            case 5:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[3][i];
              }
              break;   
            }
          }
          for (size_t i = 0; i < 4; i++){ // calculates the a coefficients using goal ang and vel
            float tf = 1.0;
            
            a[0][i] = theta[i];
            a[1][i] = 0.0;
            a[2][i] = 3.0 / (pow(tf, 2.0)) * (goalang[i] - theta[i]) - 2.0 / tf * thetadot[i] - 1.0 / tf * goalvel[i];
            a[3][i] = -2.0 / (pow(tf, 3.0)) * (goalang[i] - theta[i]) + 1.0 / (pow(tf, 2.0)) * (goalvel[i] + thetadot[i]);
            /*
            // for testing in rviz
            a[0][i] = pos[i];
            a[1][i] = 0.0;
            a[2][i] = 3.0 / (pow(tf, 2.0)) * (goalang[i] - pos[i]) - 2.0 / tf * 0.0 - 1.0 / tf * goalvel[i];
            a[3][i] = -2.0 / (pow(tf, 3.0)) * (goalang[i] - pos[i]) + 1.0 / (pow(tf, 2.0)) * (goalvel[i] + 0.0);
            */
          }
          float t = ros::Time::now().toSec() - gen_time.toSec();
          for (int i = 0; i < 4; i++){
            pos[i] = a[0][i] + a[1][i] * t + a[2][i] * pow(t, 2.0) + a[3][i] * pow(t, 3.0);
            vel[i] = a[1][i] + 2.0 * a[2][i] * t + 3.0 * a[3][i] * pow(t, 2.0);
            ang[i] = 2.0 * a[2][i] + 6.0 * a[3][i] * t;
          }
        }
        break;
      }
      
      // mode 5 can controls the first two joints using the IMU from the Myo and then the four gestures to control the 3rd joint and the gripper
      case 5:{
        if (modeChanged){ 
          modeChanged = false;
          for (int i = 0; i<3; i++){
            old_eulerAng[i] = eulerAng[i];
          }
        }
        pos[0] -= eulerAng[0] - old_eulerAng[0];
        pos[1] -= eulerAng[1] - old_eulerAng[1];
        switch (gesture){
          case 1:{ break;}
          case 2:{ pos[3] += move_pose; break;}
          case 3:{ pos[3] -= move_pose; break;}
          case 4:{ pos[2] += move_pose; break;}
          case 5:{ pos[2] -= move_pose; break;}
        }
        //old angles is used for setting the current joint angles when switching to this mode as start point so it only is controlled by the movement and not the direction
        for (int i = 0; i<3; i++){
           old_eulerAng[i] = eulerAng[i];
        }
        break;
      }
    }
  }


// setting the joint limits
  if (pos[1] > 3.14)
    pos[1] = 3.14;
  else if (pos[1] < 0)
    pos[1] = 0;
  if (pos[2] < -3.14/2)
    pos[2] = -3.14/2;
  else if (pos[2] > 3.14/2)
    pos[2] = 3.14/2;
  if (pos[3] < -3.15/2)
    pos[3] = -3.15/2;
  else if (pos[3] > 0)
    pos[3] = 0;
  
  // message declarations
  std_msgs::Float64MultiArray trajectories;

  // update trajectories
  for (size_t i = 0; i < 4; i++)
  {
    trajectories.data.push_back(pos[i]);
    trajectories.data.push_back(vel[i]);
    trajectories.data.push_back(ang[i]);
  }
  // publish trajectories
  trajectory_pub.publish(trajectories);

  // message declarations
  sensor_msgs::JointState joint_state;

  //update joint_state
  joint_state.header.stamp = ros::Time::now();

  joint_state.name.push_back("joint1");
  joint_state.name.push_back("joint2");
  joint_state.name.push_back("joint3");
  joint_state.name.push_back("joint4");
  joint_state.name.push_back("joint5");

  joint_state.position.push_back(pos[0]);
  joint_state.position.push_back(pos[1]);
  joint_state.position.push_back(pos[2]);
  joint_state.position.push_back(pos[3]);
  joint_state.position.push_back(-pos[3]);

  //send the joint state and transform
  joint_pub.publish(joint_state);

  
}


// main where it all begins
int main(int argc, char **argv){
  ros::init(argc, argv, "master_node");
  masterIntelligence master_node;
  ros::Rate loop_rate(30);

  while (ros::ok()){
    master_node.checkMyo();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
