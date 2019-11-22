#include "master.h"

using namespace std;

masterIntelligence::masterIntelligence(){

  trajectory_pub = n.advertise<std_msgs::Int16MultiArray>("/crustcrawler/trajectory", 10);
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  vibrate_pub = n.advertise<std_msgs::UInt8>("/myo_raw/vibrate", 10);

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
    vibrate.data = 1;
    vibrate_pub.publish(vibrate);
  }
}

//Takes the current joint angles and velocities
void masterIntelligence::get_angle_vel_callback(const std_msgs::Int16MultiArray::ConstPtr &msg){
  vector<short int> data_int = msg->data;
  vector<double> data;
  data.resize(data_int.size());

  for(int i = 0; i < data_int.size(); i++)
    data[i] = data_int[i] /1000.0;


  // takes the current pos and vel and divides them into 2 different vectors
  if (update_angle_vel){
    update_angle_vel = false;
    int j = 0;
    for (int i = 0; i < 12; i += 2){
      pos[j] = data[i];
      vel[j] = data[i + 1];
      j++;
    }
  }
}

void masterIntelligence::checkMyo(){
  if (gesture != 0 && gesture != 6){ // checking that gesture is not unknown or pinky_to_thumb because pinky to thumb changes modes
    switch (mode){ // then check what mode it is in
      case 1:{ // mode 1 controlls the first two joints with the four remaining gestures that is not rest
        switch (gesture){
          case 1:{
            for (size_t i = 0; i < 5; i++){
              vel[i] = 0.0;
            }
            break;
          }
          case 2:{
            pos[1] += move_pose;
            vel[1] = move_pose*UPDATE_RATE;
            break;
          }
          case 3:{
            pos[1] -= move_pose;
            vel[1] = -move_pose*UPDATE_RATE;
          break;}
          case 4:{
            pos[0] += move_pose;
            vel[0] = move_pose*UPDATE_RATE;
            break;
          }
          case 5:{
            pos[0] -= move_pose;
            vel[0] = -move_pose*UPDATE_RATE;
            break;
          }
        }
        break;
      }
      case 2:{ // mode 2 controlls the third joint and the gripper with the four remaining gestures that is not rest
        if (modeChanged){ // if first time since mode change
          modeChanged = false;
          for (int i = 0; i < 3; i++){ // set roll pitch yaw to current position
            old_eulerAng[i] = eulerAng[i];
          }
        }
        // tracks the difference from current IMU position to previous

        switch (gesture){
          case 1:{
            for (size_t i = 0; i < 5; i++){
              vel[i] = 0.0;
            }
            break;
          }
          case 2:{
            if (eulerAng[2] > old_eulerAng[2] && pos[3] < pos[4]){ // if grippers are to close and trying to get closer break
              break;
            }
            pos[3] -= eulerAng[2] - old_eulerAng[2];
            pos[4] += eulerAng[2] - old_eulerAng[2];
            break;
          }
          case 3:{
            pos[3] += eulerAng[2] - old_eulerAng[2];
            pos[4] += eulerAng[2] - old_eulerAng[2];
            break;
          }
          case 4:{
            pos[2] += move_pose;
            vel[2] = move_pose*UPDATE_RATE;
            break;
          }
          case 5:{
            pos[2] -= move_pose;
            vel[2] = -move_pose*UPDATE_RATE;
            break;
          }
        }

        // sets new eulerAng to old eulerAng
          for (int i = 0; i < 3; i++){
            old_eulerAng[i] = eulerAng[i];
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
                for (size_t i = 0; i < 5; i++){ // set the macro to the current position for all joints
                  macro[0][i] = pos[i];
                }
                //ROS_INFO_STREAM("macro 0 set:"); //
                break;
              }
              ros::spinOnce();
            }
          break;}
          case 3:{ // same as case 2 just with gesture 3
            count_time = ros::Time::now();
            while (gesture == 3){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 5; i++){
                  macro[1][i] = pos[i];
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
                for (size_t i = 0; i < 5; i++){
                  macro[2][i] = pos[i];
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
                for (size_t i = 0; i < 5; i++){
                  macro[3][i] = pos[i];
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
        if (ros::Time::now().toSec() - gen_time.toSec() >= tf){ // to ensure this mode only updates once per tf we check the timer
          gen_time = ros::Time::now(); // resets timer
          for (size_t i = 0; i < 5; i++){ // sets the goal position to current position
            goalang[i] = pos[i];
            goalvel[i] = 0.0;
          }
          switch (gesture){
            case 1:{ break;}
            case 2:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++){
                goalang[i] = 0.0;
              }

              break;
            }
            case 3:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++){
                goalang[i] = macro[1][i];
              }
              break;
            }
            case 4:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++){
                goalang[i] = macro[2][i];
              }
              break;
            }
            case 5:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++){
                goalang[i] = macro[3][i];
              }
              break;
            }
          }
          for (size_t i = 0; i < 5; i++){ // calculates the 'a' coefficients using goal ang and vel
              a[0][i] = pos[i];
              a[1][i] = 0.0;
              a[2][i] = 3.0 / (pow(tf, 2.0)) * (goalang[i] - pos[i]) - 2.0 / tf * 0.0 - 1.0 / tf * goalvel[i];
              a[3][i] = -2.0 / (pow(tf, 3.0)) * (goalang[i] - pos[i]) + 1.0 / (pow(tf, 2.0)) * (goalvel[i] + 0.0);
          }
        }
        // calculates the theta, thetadot and thetadotdot for all joints
        float t = ros::Time::now().toSec() - gen_time.toSec();
        for (int i = 0; i < 5; i++){
          pos[i] = a[0][i] + a[1][i] * t + a[2][i] * pow(t, 2.0) + a[3][i] * pow(t, 3.0);
          vel[i] = a[1][i] + 2.0 * a[2][i] * t + 3.0 * a[3][i] * pow(t, 2.0);
          ang[i] = 2.0 * a[2][i] + 6.0 * a[3][i] * t;
        }
      break;
      }

      // mode 5 can controls the first two joints using the IMU from the Myo and then the four gestures to control the 3rd joint and the gripper
      case 5:{
        if (modeChanged){ // if first time since mode change
          modeChanged = false;
          for (int i = 0; i < 3; i++){ // set roll pitch yaw to current position
            old_eulerAng[i] = eulerAng[i];
          }
        }
        // tracks the difference from current IMU position to previous
        pos[0] -= eulerAng[0] - old_eulerAng[0];
        pos[1] -= eulerAng[1] - old_eulerAng[1];
        switch (gesture){
          case 1:{
            for (size_t i = 0; i < 5; i++){
              vel[i] = 0.0;
            }
            break;
          }
          case 2:{
            pos[3] += move_pose;
            vel[3] = move_pose*UPDATE_RATE;
            pos[4] -= move_pose;
            vel[4] = move_pose*UPDATE_RATE;
            break;
          }
          case 3:{
            pos[3] -= move_pose;
            vel[3] = -move_pose*UPDATE_RATE;
            pos[4] += move_pose;
            vel[4] = -move_pose*UPDATE_RATE;
            break;
          }
          case 4:{
            pos[2] += move_pose;
            vel[2] = move_pose*UPDATE_RATE;
            break;
          }
          case 5:{
            pos[2] -= move_pose;
            vel[2] = -move_pose*UPDATE_RATE;
            break;
          }
        }
        // sets new eulerAng to old eulerAng
        for (int i = 0; i < 3; i++){
           old_eulerAng[i] = eulerAng[i];
        }
        break;
      }
    }
  }

// setting the joint limits
  if (pos[1] > 3.14/2)
    pos[1] = 3.14/2;
  else if (pos[1] < -3.14/2)
    pos[1] = -3.14/2;
  if (pos[2] < -3.14/2)
    pos[2] = -3.14/2;
  else if (pos[2] > 3.14/2)
    pos[2] = 3.14/2;
  if (pos[3] > 3.14/2)
    pos[3] = 3.14/2;
  else if (pos[3] < -0.6)
    pos[3] = -0.6;
  if (pos[4] < -3.14/2)
    pos[4] = -3.14/2;
  else if (pos[4] > 0.6)
    pos[4] = 0.6;





  // update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.clear();
  joint_state.name.push_back("joint1");
  joint_state.name.push_back("joint2");
  joint_state.name.push_back("joint3");
  joint_state.name.push_back("joint4");
  joint_state.name.push_back("joint5");

  // clear previous message
  joint_state.position.clear();

  // joint_state.position
  joint_state.position.push_back(pos[0]);
  joint_state.position.push_back(pos[1]);
  joint_state.position.push_back(pos[2]);
  joint_state.position.push_back(pos[3]);
  joint_state.position.push_back(pos[4]);

  // send the joint state and transform
  joint_pub.publish(joint_state);

  // clear previous message
  trajectories.data.clear();

  // update trajectories
  for (size_t i = 0; i < 5; i++)
  {
    trajectories.data.push_back((int16_t)(pos[i]*1000));
    trajectories.data.push_back((int16_t)(vel[i]*1000));
    trajectories.data.push_back((int16_t)(ang[i]*1000));
  }

  // publish trajectories
  trajectory_pub.publish(trajectories);
}


// main where it all begins
int main(int argc, char **argv){
  ros::init(argc, argv, "master_node");
  masterIntelligence master_node;
  ros::Rate loop_rate(UPDATE_RATE);

  while (ros::ok()){
    master_node.checkMyo();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
