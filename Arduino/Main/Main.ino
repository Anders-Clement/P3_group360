#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Protocol_2.h"
#include "Controller.h"
#include <std_msgs/Int16.h>
#include "Display.h"

#define buzz_pin 48

void enableTorque();
void disableTorque();
long lastMessageTime;

bool protectiveStop = false;
ProtocolController* controler_ptr;
PID_Controller *PID_Controller_ptr;

void setPWMMode(); //Sets PWM mode on motors
void publishAngVel(); //Publishes our angel and velocities to ROS
void getPositionsVelocities();

Display *display_ptr;

//ROS specific data:
std_msgs::Int16MultiArray angleVel_msg;
void trajectory_sub(const std_msgs::Int16MultiArray& incoming_msg); //subscribes to the trajectory topic
void command_callback(const std_msgs::Int16& msg); //Gets the command topic from ROS

int freeRam ()
{
extern int __heap_start, *__brkval;
int v;
return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

ros::NodeHandle  nh;
ros::Publisher getAngleVel_pub("/crustcrawler/getAngleVel", &angleVel_msg);
ros::Subscriber<std_msgs::Int16MultiArray> trajectorySubscriber("/crustcrawler/trajectory", &trajectory_sub);
ros::Subscriber<std_msgs::Int16> commandSub("/crustcrawler/command", &command_callback);


float motorOffsets[5] = {0, M_PI / 4.0, M_PI, M_PI * (3.0 / 4.0), M_PI};
float joint0_offset = 0;
bool setOffset = false;
bool rosConnected = false;

//PID Controller
float thetas[5];
float velocities[5];

void setup()
{
  controler_ptr = new ProtocolController();
  display_ptr = new Display();
  //Serial.begin(57600);
  //Serial.println("Setup");
  display_ptr->setStatus("Setup");


  nh.getHardware()->setBaud(57600);

  nh.initNode();
  nh.advertise(getAngleVel_pub);
  nh.subscribe(trajectorySubscriber);
  nh.subscribe(commandSub);

  delay(1000);
  display_ptr->setConnect("False"); //connected with PC (usually has not happened yet)


  //allocate memory for anglevel message
  angleVel_msg.data = (int*) malloc(sizeof(int) * 10);
  //set length of msg, otherwise everything will not be sent
  angleVel_msg.data_length = 10;

  float temp_zeros[5] = {0, 0, 0, 0, 0};
  updateTorques(temp_zeros);



  PID_Controller_ptr = new PID_Controller(thetas, velocities);


  int trajectory[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  PID_Controller_ptr->trajectoryFunk(trajectory);

  /*
  lastMessageTime = millis();

  Serial.print("trajectory: ");
  for(int i = 0; i < 5; i++)
  {
    Serial.print(PID_Controller_ptr->posDesired[i]);
    Serial.print(", ");
  }
  Serial.println("");
  Serial.println("Getting posVels");
  getPositionsVelocities();
  float* torques = PID_Controller_ptr->update();
  Serial.print("Torques: ");
  for(int i = 0; i < 5; i++)
  {
    Serial.print(torques[i]);
    Serial.print(", ");
  }
  */
  setPWMMode();
  //enableTorque(); //turn on all motors

  display_ptr->setStatus("Running");
}

void loop()
{

  getPositionsVelocities();
  float* torques = PID_Controller_ptr->update();
  updateTorques(torques);

  publishAngVel();


  nh.spinOnce();
  /*
  int freem = freeRam();
  String fr = String(freem);
  char* msg = (char*) malloc(sizeof(char)*16);
  fr.toCharArray(msg,16);
  //nh.loginfo(msg);
  
  Serial.print(counter);
  Serial.print(", ");
  Serial.println(fr);

  delete msg;
  */
  //Check if connected to ros, and keep the status printed on the screen
  if(rosConnected == false)
  {
    if(nh.connected())
    {
      rosConnected = true;
      display_ptr->setConnect("True");
    }
    else
      display_ptr->setConnect("False");
  }
  if(!nh.connected())
    rosConnected = false;
}

void trajectory_sub(const std_msgs::Int16MultiArray& incoming_msg)
{
  PID_Controller_ptr->trajectoryFunk(incoming_msg.data);
}

void command_callback(const std_msgs::Int16& msg)
{
  if (msg.data == 0) //stop command
  {
    display_ptr->setLastLine("command");
    disableTorque();
  }
  else if (msg.data == 1)
  {
    setOffset = true;
    setPWMMode();
    enableTorque();
  }
  else if(msg.data == 2)
    display_ptr->setLastLine("two");

}


void getPositionsVelocities()
{
  for (int i = 1; i < 6; i++)
  {
    thetas[i - 1] = (controler_ptr->getPos(i) * 0.0015336f) - motorOffsets[i - 1];
    velocities[i - 1] = (float)controler_ptr->getVel(i) * 0.0229; //convert to rad/s (0.229rpm/step * 0.1(rad/sec)/rpm
  } //this for loop takes 18.3ms

  if (setOffset) {
    joint0_offset = thetas[0];
    setOffset = false;
  }
  thetas[0] = thetas[0] - joint0_offset;
}

void updateTorques(float* torqueArray)
{
  if (!protectiveStop)
  {
    for (int i = 1; i < 6; i++)
    {
      controler_ptr->setTorque(i, torqueArray[i - 1], velocities[i - 1]);
    }
  }
}

void publishAngVel() {
  //publish thetas and velocities:
  for (int i = 0; i < 5; i++)
  {
    angleVel_msg.data[i * 2] = thetas[i]*1000.0;
    angleVel_msg.data[i * 2 + 1] = velocities[i]*1000.0;
  }

  getAngleVel_pub.publish(&angleVel_msg);
}


void enableTorque()
{
  for (int i = 1; i < 6; i++) {
    controler_ptr->toggleTorque(i, true);
  }
}

void setPWMMode()
{
  for (int i = 1; i < 6; i++)
    controler_ptr->setOperatingMode(i, 0x10);
}

void disableTorque()
{
  for (int i = 1; i < 6; i++)
    controler_ptr->toggleTorque(i, false);
}
