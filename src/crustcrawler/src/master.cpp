#include "master.h"

using namespace std;

masterIntelligence::masterIntelligence(){

  trajectory_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/trajectory", 10);
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  gest_str_sub = n.subscribe("myo_raw/myo_gest_str", 10, &masterIntelligence::myo_raw_gest_str_callback, this);
  get_angle_vel = n.subscribe("/crustcrawler/getAngleVel", 10, &masterIntelligence::get_angle_vel_callback, this);
  pose_sub = n.subscribe("myo_raw/pose", 10, &masterIntelligence::myo_raw_pose_callback, this);

  ros::spinOnce();

  gen_time = ros::Time::now();

  for (int i = 0; i < 4; i++) {
    a0[i] = 0;
    a1[i] = 0;
    a2[i] = 0;
    a3[i] = 0;
  }
}

// Takes the quaternions from the imu and calculates the roll, pitch and yaw
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

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

//checks what gesture the myo detects
void masterIntelligence::myo_raw_gest_str_callback(const std_msgs::String::ConstPtr &msg){
  string data = msg->data;
  const string known_gestures[] = {"UNKNOWN", "REST", "FIST", "FINGERS_SPREAD", "WAVE_IN", "WAVE_OUT", "THUMB_TO_PINKY"};

  for (int i = 0; i < 7; i++){
    if (data == known_gestures[i]){
      gesture = i;
      ROS_INFO_STREAM(data);
      break;
    }
  }
  if (gesture == 6){
    if (mode == 4)
      mode = 1;
    else{
      mode += 1;
      first_oldangles = true;
    }
    ROS_INFO_STREAM(mode);
  }
}

//Calculates the trajectory and publishes
void masterIntelligence::calc_traj(){
  float t = ros::Time::now().nsec - gen_time.nsec;
  t = t/1000000000.0;
  std_msgs::Float64MultiArray trajectories;
  ROS_INFO("TRAJ");
  ROS_INFO_STREAM("time 0: " << t);
  ROS_INFO_STREAM("Pos 0: " << pos[0]);
  for (int i = 0; i < 4; i++){
    trajectories.data.push_back(a0[i] + a1[i] * t + a2[i] * pow(t, 2) + a3[i] * pow(t, 3));
    trajectories.data.push_back(a1[i] + 2 * a2[i] * t + 3 * a3[i] * pow(t, 2));
    trajectories.data.push_back(2 * a2[i] + 6 * a3[i] * t);

    // for testing in rviz
    // pos[i] = a0[i] + a1[i] * t + a2[i] * pow(t, 2.0) + a3[i] * pow(t, 3.0);
  }
  ROS_INFO_STREAM("Pos 0: " << pos[0]);
  //trajectory_pub.publish(trajectories);
}

//Takes the current joint angles and velocities
void masterIntelligence::get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr &msg){
  vector<double> data = msg->data;
  int j = 0;
  for (int i = 0; i < 10; i += 2){
    theta[j] = data[i];
    thetadot[j] = data[i + 1];
    j++;
  }

  if (firstRead){
    firstRead = false;
    for (int i = 0; i < 4; i++)
    {
     pos[i] = theta[i];
    }
  }
}


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

void masterIntelligence::checkMyo(){
  if (gesture != 0 && gesture != 6){
    switch (mode){
      case 1:
        switch (gesture){
          case 1: break;
          case 2: pos[1] += move_pose; break;
          case 3: pos[1] -= move_pose; break;
          case 4: pos[0] += move_pose; break;
          case 5: pos[0] -= move_pose; break;
        }
      break;
      case 2:
        switch (gesture){
          case 1: break;
          case 2: pos[3] += move_pose; break;
          case 3: pos[3] -= move_pose; break;
          case 4: pos[2] += move_pose; break;
          case 5: pos[2] -= move_pose; break;
        }
      break;
      case 3: 
        if (first_oldangles){
          first_oldangles = false;
          for (int i = 0; i<3; i++){
            oldAngles[i] = angles[i];
          }
        }
        pos[0] -= angles[0] - oldAngles[0];
        pos[1] -= angles[1] - oldAngles[1];
        switch (gesture){
          case 1: break;
          case 2: pos[3] += move_pose; break;
          case 3: pos[3] -= move_pose; break;
          case 4: pos[2] += move_pose; break;
          case 5: pos[2] -= move_pose; break;
        }
        for (int i = 0; i<3; i++){
           oldAngles[i] = angles[i];
        }
      break;
      case 4:
        if (ros::Time::now().nsec - gen_time.nsec >= 1000000000.0){
          gen_time = ros::Time::now();
          float goalang[4] = {pos[0], pos[1], pos[2], pos[3]};
          float goalvel[4] = {0, 0, 0, 0};
          switch (gesture){
            case 1: break;
            case 2:
              goalang[0] = 0;
            break;
            case 3:
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[3][i];
              }
            case 4:
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[4][i];
              }
            break;
            case 5:
              for (size_t i = 0; i < 4; i++){
                goalang[i] = macro[5][i];
              }
            break;        
              
          }
          for (size_t i = 0; i < 4; i++){
                float tf = 1.0;
                a0[i] = theta[i];
                a1[i] = thetadot[i];
                a2[i] = 3 / (pow(tf, 2)) * (goalang[i] - theta[i]) - 2 / tf * thetadot[i] - 1 / tf * goalvel[i];
                a3[i] = -2 / (pow(tf, 3)) * (goalang[i] - theta[i]) + 1 / (pow(tf, 2)) * (goalvel[i] + thetadot[i]);

                // for testing in rviz
                //a0[i] = pos[i];
                //a1[i] = 0.0;
                //a2[i] = 3.0 / (pow(tf, 2.0)) * (goalang[i] - pos[i]) - 2.0 / tf * 0.0 - 1.0 / tf * goalvel[i];
                //a3[i] = -2.0 / (pow(tf, 3.0)) * (goalang[i] - pos[i]) + 1.0 / (pow(tf, 2.0)) * (goalvel[i] + 0.0);
              }
              //ROS_INFO_STREAM(a0[0]);
              //ROS_INFO("GENERATING POS");
        }
        calc_traj();
      break; 
      case 5:
        switch(gesture){
          case 1: break;
          case 2: break;
          case 3:
            count_time = ros::Time::now();
            while (gesture == 3){
              if(ros::Time::now().nsec - count_time.nsec >= 1000000000.0){
                for (size_t i = 0; i < 4; i++){
                  macro[3][i] = theta[i];
                }
                break;
              }
            }
          break;
          case 4:
            count_time = ros::Time::now();
            while (gesture == 4){
              if(ros::Time::now().nsec - count_time.nsec >= 1000000000.0){
                for (size_t i = 0; i < 4; i++){
                  macro[4][i] = theta[i];
                }
                break;
              }
            }
          break;
          case 5:
            count_time = ros::Time::now();
            while (gesture == 5){
              if(ros::Time::now().nsec - count_time.nsec >= 1000000000.0){
                for (size_t i = 0; i < 4; i++){
                  macro[5][i] = theta[i];
                }
                break;
              }
            }
          break;
        }
      break;
    }
  }

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
  if (pos[4] < 0)
    pos[4] = 0;
  else if (pos[4] > 3.14/2)
    pos[4] = 3.14/2;
  

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
