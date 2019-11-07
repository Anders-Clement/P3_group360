#include "master.h"

using namespace std;

masterIntelligence::masterIntelligence(int argc, char **argv)
{
  ros::init(argc, argv, "master_node");

  trajectory_pub = n.advertise<std_msgs::Float64MultiArray>("/crustcrawler/trajectory", 10);
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  gest_str_sub = n.subscribe("myo_raw/myo_gest_str", 10, &masterIntelligence::myo_raw_gest_str_callback, this);
  get_angle_vel = n.subscribe("/crustcrawler/getAngleVel", 10, &masterIntelligence::get_angle_vel_callback, this);

  ros::spinOnce();

  gen_time = ros::Time::now();
}

//checks what gesture the myo detects
void masterIntelligence::myo_raw_gest_str_callback(const std_msgs::String::ConstPtr &msg)
{
  string data = msg->data;
  const string known_gestures[] = {"UNKNOWN", "REST", "FIST", "FINGERS_SPREAD", "WAVE_IN", "WAVE_OUT", "THUMB_TO_PINKY"};

  for (int i = 0; i < 6; i++)
  {
    if (data == known_gestures[i])
    {
      gesture = i;
      ROS_INFO_STREAM(data);
      return;
    }
  }
}

//Calculates the trajectory and publishes
void masterIntelligence::calc_traj()
{
  float t = ros::Time::now().sec - gen_time.sec;
  std_msgs::Float64MultiArray trajectories;
  trajectories.data.resize(12);
  for (int i = 0; i < 4; i++)
  {
    trajectories.data[i * 4] = a0[i] + a1[i] * t + a2[i] * pow(t, 2) + a3[i] * pow(t, 3);
    trajectories.data[i * 4 + 1] = a1[i] + 2 * a2[i] * t + 3 * a3[i] * pow(t, 2);
    trajectories.data[i * 4 + 2] = 2 * a2[i] + 6 * a3[i] * t;
  }
  trajectory_pub.publish(trajectories);
}

//Takes the current joint angles and velocities
void masterIntelligence::get_angle_vel_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  vector<double> data = msg->data;
  int j = 0;
  for (int i = 0; i < 10; i += 2)
  {
    theta[j] = data[i];
    thetadot[j] = data[i + 1];
    j++;
  }
  calc_traj();
}

<<<<<<< HEAD
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
    joint_state.name[0] ="joint1";
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

    Vector3 input;
    input.x = 0.0;
    input.y = 0.0;
    input.z = 0.0;

    Vector3 pos = f_kin(input);

    Vector3 result = inv_kin_closest(pos, input);

// updeate and check for news in the subscribers
    ros::spinOnce();

// This will adjust as needed per iteration
    loop_rate.sleep();
  }


  return 0;
}



void check_for_zero(Vector3 &input)
=======
void masterIntelligence::check_for_zero(Vector3 &input)
>>>>>>> ad318009dd968e324940eac1bd90406236d2569d
{
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

Vector3 masterIntelligence::f_kin(Vector3 thetas)
{

  Vector3 result;
  float pi = 3.1416;
  result.x = (11 * cos(thetas.x) * cos(thetas.y + pi / 2)) / 50 - (3 * cos(thetas.x) * sin(thetas.z) * sin(thetas.y + pi / 2)) / 20 + (3 * cos(thetas.x) * cos(thetas.z) * cos(thetas.y + pi / 2)) / 20;
  result.y = (11 * cos(thetas.y + pi / 2) * sin(thetas.x)) / 50 + (3 * cos(thetas.z) * cos(thetas.y + pi / 2) * sin(thetas.x)) / 20 - (3 * sin(thetas.x) * sin(thetas.z) * sin(thetas.y + pi / 2)) / 20;
  result.z = (11 * sin(thetas.y + pi / 2)) / 50 + (3 * cos(thetas.z) * sin(thetas.y + pi / 2)) / 20 + (3 * cos(thetas.y + pi / 2) * sin(thetas.z)) / 20 + 11.0 / 200.0;

  return result;
}

Vector3 masterIntelligence::inv_kin_closest(Vector3 pos, Vector3 angles)
{
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

void masterIntelligence::checkMyo()
{
  float pos[4] = {theta[0], theta[1], theta[2], theta[3]};
  if (ros::Time::now().sec - gen_time.sec >= 1.0)
  {
    gen_time = ros::Time::now();

    if (gesture == 6)
    {
      if (mode == 3)
      {
        mode = 1;
      }
      else
      {
        mode += 1;
      }
      ROS_INFO_STREAM(mode);
    }
    if (gesture != 0 && gesture != 1 && gesture != 6)
    {
      switch (mode)
      {
      case 1:
        switch (gesture)
        {
        case 2:
          pos[0] += move_pose;
          break;
        case 3:
          pos[0] -= move_pose;
          break;
        case 4:
          pos[1] += move_pose;
          break;
        case 5:
          pos[1] -= move_pose;
          break;
        }
        break;
      case 2:
        switch (gesture)
        {
        case 2:
          pos[2] += move_pose;
          break;
        case 3:
          pos[2] -= move_pose;
          break;
        case 4:
          pos[3] += move_pose;
          break;
        case 5:
          pos[3] -= move_pose;
          break;
        }
        break;
      case 3:
        switch (gesture)
        {
        case 2:
          float goalang[4] = {0, 0, 0, 0};
          float goalvel[4] = {0, 0, 0, 0};
          for (size_t i = 0; i < 4; i++)
          {
            float tf = 1;
            a0[i] = theta[i];
            a1[i] = thetadot[i];
            a2[i] = 3 / (pow(tf, 2)) * (goalang[i] - theta[i]) - 2 / tf * thetadot[i] - 1 / tf * goalvel[i];
            a3[i] = -2 / (pow(tf, 3)) * (goalang[i] - theta[i]) + 1 / (pow(tf, 2)) * (goalvel[i] + thetadot[i]);
          }
          break;
        }
        break;
      }
    }
  }

  //update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(5);
  joint_state.position.resize(5);
  joint_state.name[0] = "joint1";
  joint_state.position[0] = pos[0];
  joint_state.name[1] = "joint2";
  joint_state.position[1] = pos[1];
  joint_state.name[2] = "joint3";
  joint_state.position[2] = pos[2];
  joint_state.name[3] = "joint4";
  joint_state.position[3] = pos[3];
  joint_state.name[4] = "joint5";
  joint_state.position[4] = -pos[3];

  //send the joint state and transform
  joint_pub.publish(joint_state);
}

int main(int argc, char **argv)
{
  masterIntelligence master_node(argc, argv);
  while (ros::ok())
  {
    master_node.checkMyo();
    
    ros::spinOnce();
  }

  return 0;
}