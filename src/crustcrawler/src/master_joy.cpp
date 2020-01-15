
#include "ros/ros.h"
#include "masterIntelligence.h"

// main where it all begins
int main(int argc, char **argv){
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  masterIntelligence* master_node = new masterIntelligence(&n, 1);

  ros::Rate loop_rate(UPDATE_RATE);


  while (ros::ok()){
    //master_node->handleGesture();

    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}
