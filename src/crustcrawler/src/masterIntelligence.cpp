#include "masterIntelligence.h"

using namespace std;

masterIntelligence::masterIntelligence(ros::NodeHandle* n, int type){

  trajectory_pub = n->advertise<std_msgs::Int16MultiArray>("/crustcrawler/trajectory", 10);
  joint_pub = n->advertise<sensor_msgs::JointState>("joint_states", 10);
  vibrate_pub = n->advertise<std_msgs::UInt8>("/myo_raw/vibrate", 10);
  mode_pub = n->advertise<std_msgs::Int16>("/crustcrawler/mode", 10);
  gesture_pub = n->advertise<std_msgs::Int16>("/crustcrawler/gesture", 10);

  get_angle_vel = n->subscribe("/crustcrawler/getAngleVel", 2, &masterIntelligence::get_angle_vel_callback, this);

  if(type == 0)
    gest_str_sub = n->subscribe("/myo_raw/myo_gest_str", 2, &masterIntelligence::myo_raw_gest_str_callback, this);
  else if(type == 1)
    joy_sub = n->subscribe("/joy", 2, &masterIntelligence::joy_callback, this);
  else if(type == 2){
    key_sub = n->subscribe("/cmd_vel", 2, &masterIntelligence::key_callback, this);
    key_control = true;}
  else
    ROS_INFO_STREAM("UNKNOWN TYPE FOR MASTERINTELLIGENCE!");

  while (update_angle_vel)
    ros::spinOnce();
}

/*
void masterIntelligence::getChar(){
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

  gesture = 1;

  if (c == 'x'){ //change mode with x button
    mode++;
    if(mode > 4)
      mode = 1;

    //ROS_INFO_STREAM(mode);
    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
    ROS_INFO_STREAM(mode);

    return;
  }

  if (c == 'a')
    gesture = 2;
  else if (c == 'd')
    gesture = 3;
  else if (c == 'w')
    gesture = 4;
  else if (c == 's')
    gesture = 5;
  ROS_INFO_STREAM(gesture);
}*/


void masterIntelligence::key_callback(const geometry_msgs::Twist::ConstPtr& msg){
  gesture = 1;
  if(msg->linear.x == -0.5 &&  msg->angular.z == 1){ //change mode with '.' button{
    mode++;
    if(mode > 4)
      mode = 1;

    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
    ROS_INFO_STREAM("mode: " << mode);

    return;
  }
  else if (msg->linear.x == 0.5 && msg->angular.z == 0)
    gesture = 2;
  else if (msg->linear.x == -0.5 && msg->angular.z == 0)
    gesture = 3;
  else if (msg->linear.x == 0 && msg->angular.z == 1)
    gesture = 4;
  else if (msg->linear.x == 0 && msg->angular.z == -1)
    gesture = 5;
  ROS_INFO_STREAM("gesture:" << gesture);
}


void masterIntelligence::joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  if(msg->buttons[0]) //change mode with x button
  {
    mode++;
    if(mode > 4)
      mode = 1;

    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
    ROS_INFO_STREAM("mode: " << mode);

    return;
  }

  gesture = 1;

  for(int i = 0; i < 4; i++)
  {
      if(msg->buttons[i + 13])
        gesture = i + 2; //0:unknown, 1:rest, 2-5 - wanted gestures
  }
  if(mode == 1)
  {
    if(gesture == 2)
      gesture = 3;
    else if(gesture == 3)
      gesture = 2;
  }
  else if(mode == 2)
  {
    if(gesture == 2)
      gesture = 5;
    else if(gesture == 3)
      gesture = 4;
    else if(gesture == 4)
      gesture = 3;
    else if(gesture == 5)
      gesture = 2;
  }
  //ROS_INFO_STREAM(gesture);
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
      gesture_msg.data = gesture;
      gesture_pub.publish(gesture_msg);
      break;
    }
  }
  // if the gesture held is "THUMB_TO_PINKY" then change mode from 1 to 5 and then reset to 1
  if (gesture == 6){
    if (mode == 4)
      mode = 0;
    else
      mode += 1;
    ROS_INFO_STREAM(mode);
    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
    vibrate_msg.data = 1;
    vibrate_pub.publish(vibrate_msg);
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
  handleGesture();
}

void masterIntelligence::handleGesture(){
  if (gesture != 0 && gesture != 6){ // checking that gesture is not unknown or pinky_to_thumb because pinky to thumb changes modes
    switch (mode){ // then check what mode it is in
      case 1:{ // mode 1 controlls the first two joints with the four remaining gestures that is not rest
        switch (gesture){
          case 1:{
            for (size_t i = 0; i < 5; i++)
              vel[i] = 0.0;
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
            break;
          }
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
        switch (gesture){
          case 1:{
            for (size_t i = 0; i < 5; i++)
              vel[i] = 0.0;
            break;
          }
          case 2:{
            pos[3] += move_pose;
            vel[3] = move_pose*UPDATE_RATE;
            pos[4] -= move_pose;
            vel[4] = -move_pose*UPDATE_RATE;
            break;
          }
          case 3:{
            pos[3] -= move_pose;
            vel[3] = -move_pose*UPDATE_RATE;
            pos[4] += move_pose;
            vel[4] = move_pose*UPDATE_RATE;
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
        break;
      }
      case 3:{ // mode 3 is able to set a the current joint angles macro to a gesture by holding the gesture for 2 seconds
        if(lastMacroSetTime.toSec() + 1 < ros::Time::now().toSec())
          lastMacroSet = -1;

        switch(gesture){
          case 1:{ break;} // if gesture is rest break
          case 2:{
            /*
            if(lastMacroSet != 2)
            {
              lastMacroSet = 2;
              lastMacroSetTime = ros::Time::now();
              for (size_t i = 0; i < 5; i++) // set the macro to the current position for all joints
                macro[0][i] = pos[i];
              ROS_INFO_STREAM("macro 0 set:");
              vibrate_msg.data = 2;
              vibrate_pub.publish(vibrate_msg);
            }
            */
            /*
            count_time = ros::Time::now(); // resets counter timer
            ROS_INFO_STREAM("gesture 3, case 2");
            while (gesture == 2){ // enters a while loop to be able check if the gesture is held
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){ // if the gesture is held for 2 sec
                for (size_t i = 0; i < 5; i++) // set the macro to the current position for all joints
                  macro[0][i] = pos[i];
                ROS_INFO_STREAM("macro 0 set:");
                vibrate_msg.data = 2;
                vibrate_pub.publish(vibrate_msg);
                break;
              }
              ros::spinOnce();
            }
            */
            break;
          }
          case 3:{ // same as case 2 just with gesture 3
            if(lastMacroSet < 0)
            {
              lastMacroSet = 3;
              lastMacroSetTime = ros::Time::now();
              for (size_t i = 0; i < 5; i++) // set the macro to the current position for all joints
                macro[1][i] = pos[i];
              ROS_INFO_STREAM("macro 1 set:");
              vibrate_msg.data = 2;
              vibrate_pub.publish(vibrate_msg);
            }
            /*
            count_time = ros::Time::now();
            while (gesture == 3){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 5; i++)
                  macro[1][i] = pos[i];
                ROS_INFO_STREAM("macro 1 set:");
                vibrate_msg.data = 2;
                vibrate_pub.publish(vibrate_msg);
                break;
              }
              ros::spinOnce();
            }
            */
            break;
          }
          case 4:{ // same as case 2 just with gesture 4
            if(lastMacroSet < 0)
            {
              lastMacroSet = 4;
              lastMacroSetTime = ros::Time::now();
              for (size_t i = 0; i < 5; i++) // set the macro to the current position for all joints
                macro[2][i] = pos[i];
              ROS_INFO_STREAM("macro 2 set:");
              vibrate_msg.data = 2;
              vibrate_pub.publish(vibrate_msg);
            }
            /*
            count_time = ros::Time::now();
            while (gesture == 4){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 5; i++)
                  macro[2][i] = pos[i];
                ROS_INFO_STREAM("macro 2 set:");
                vibrate_msg.data = 2;
                vibrate_pub.publish(vibrate_msg);
                break;
              }
              ros::spinOnce();
            }
            */
            break;
          }
          case 5:{ // same as case 2 just with gesture 5
            if(lastMacroSet < 0)
            {
              lastMacroSet = 5;
              lastMacroSetTime = ros::Time::now();
              for (size_t i = 0; i < 5; i++) // set the macro to the current position for all joints
                macro[3][i] = pos[i];
              ROS_INFO_STREAM("macro 3 set:");
              vibrate_msg.data = 2;
              vibrate_pub.publish(vibrate_msg);
            }
          /*
            count_time = ros::Time::now();
            while (gesture == 5){
              if(ros::Time::now().toSec() - count_time.toSec() >= 2.0){
                for (size_t i = 0; i < 5; i++)
                  macro[3][i] = pos[i];
                ROS_INFO_STREAM("macro 3 set:");
                vibrate_msg.data = 2;
                vibrate_pub.publish(vibrate_msg);
                break;
              }
              ros::spinOnce();
            }
            */
            break;
          }
        }
        break;
      }
      // mode 4 can recall saved macros by using the gesture is have been saved under
      case 4:{
        if (ros::Time::now().toSec() - gen_time.toSec() >= tf)
        { // to ensure this mode only updates once per tf we check the timer
          for (size_t i = 0; i < 5; i++){ // sets the goal position to current position
            goalang[i] = pos[i];
            goalvel[i] = 0.0;
          }
          switch (gesture){
            case 1:{ break;}
            case 2:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++)
                goalang[i] = 0.0;
              gen_time = ros::Time::now(); // resets timer
              break;
            }
            case 3:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++)
                goalang[i] = macro[1][i];
              gen_time = ros::Time::now(); // resets timer
              break;
            }
            case 4:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++)
                goalang[i] = macro[2][i];
              gen_time = ros::Time::now(); // resets timer
              break;
            }
            case 5:{ // sets goal position to the corresponding macro
              for (size_t i = 0; i < 5; i++)
                goalang[i] = macro[3][i];
              gen_time = ros::Time::now(); // resets timer
              break;
            }
          }
          
          double maxDistance = 0;
          for(int i = 0; i < 3; i++)
          {
            double dist = abs(goalang[i] - pos[i]);
            if(dist > maxDistance)
              maxDistance = dist;
          }

          tf = maxDistance * 3.0;

          if(tf < 2)
            tf = 2;

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
          acc[i] = 2.0 * a[2][i] + 6.0 * a[3][i] * t;
        }
      break;
      }
    }
  }


// setting the joint limits

  if (pos[1] > 1.9)
    pos[1] = 1.9;
  else if (pos[1] < -1.9)
    pos[1] = -1.9;
  if (pos[2] < -3.14/1.8)
    pos[2] = -3.14/1.8;
  else if (pos[2] > 3.14/1.8)
    pos[2] = 3.14/1.8;
  if (pos[3] < -3.14/2.0)
    pos[3] = -3.14/2.0;
  else if (pos[3] > 0.0)
    pos[3] = 0.0;
  if (pos[4] > 3.14/2.0)
    pos[4] = 3.14/2.0;
  else if (pos[4] < 0.0)
    pos[4] = 0.0;

    /*
  // Joint state publisher for RVIZ debugging
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name.clear();
  joint_state_msg.name.push_back("joint1");
  joint_state_msg.name.push_back("joint2");
  joint_state_msg.name.push_back("joint3");
  joint_state_msg.name.push_back("joint4");
  joint_state_msg.name.push_back("joint5");
  // clear previous message
  joint_state_msg.position.clear();
  // joint_state_msg.position
  joint_state_msg.position.push_back(pos[0]);
  joint_state_msg.position.push_back(pos[1]);
  joint_state_msg.position.push_back(pos[2]);
  joint_state_msg.position.push_back(pos[3]);
  joint_state_msg.position.push_back(pos[4]);
  // send the joint state and transform
  joint_pub.publish(joint_state_msg);
  */



  // clear previous message
  trajectories_msg.data.clear();
  // update trajectories_msg
  for (size_t i = 0; i < 5; i++)
  {
    trajectories_msg.data.push_back((int16_t)(pos[i]*1000));
    trajectories_msg.data.push_back((int16_t)(vel[i]*1000));
    trajectories_msg.data.push_back((int16_t)(acc[i]*1000));
  }
  // publish trajectories_msg
  trajectory_pub.publish(trajectories_msg);
  if (key_control)
    gesture = 1;
}
