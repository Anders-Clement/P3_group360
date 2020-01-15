#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Protocol_2.h"
#include "Controller.h"
#include <std_msgs/Int16.h>
#include "Display.h"
#include "Buzzer.h"

#define buzz_pin 11

ProtocolController* controler_ptr;
PID_Controller *PID_Controller_ptr;
Display *display_ptr;   //Display controller
Buzzer* buzzer;

void setPWMMode(); //Sets PWM mode on motors
void publishAngVel(); //Publishes our angel and velocities to ROS
void getPositionsVelocities(); //gets positions and velocities from all motors
void enableTorque(); //on all motors
void disableTorque(); //on all motors
void updateTorques(float* torqueArray); //sets torques on all motors

//ROS specific data:
std_msgs::Int16MultiArray angleVel_msg;
void trajectory_sub(const std_msgs::Int16MultiArray& incoming_msg); //subscribes to the trajectory topic
void command_callback(const std_msgs::Int16& msg); //Gets the command topic from ROS
void mode_callback(const std_msgs::Int16& msg); //Gets the mode topic from ROS

ros::NodeHandle  nh;
ros::Publisher getAngleVel_pub("/crustcrawler/getAngleVel", &angleVel_msg);
ros::Subscriber<std_msgs::Int16MultiArray> trajectorySubscriber("/crustcrawler/trajectory", &trajectory_sub);
ros::Subscriber<std_msgs::Int16> commandSub("/crustcrawler/command", &command_callback);
ros::Subscriber<std_msgs::Int16> modeSub("/crustcrawler/mode", &mode_callback);

float motorOffsets[5] = {0, M_PI / 4.0, M_PI, M_PI * (3.0 / 4.0), M_PI};
float joint0_offset = 0;
bool setOffset = false;
bool rosConnected = false;
unsigned char checkTimer = 0;
unsigned long runTime;


//PID Controller
float thetas[5];
float velocities[5];

int freeRam()  //only used for testing, don't worry about it
{
extern int __heap_start, *__brkval;
int v;
return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void setup()
{
  controler_ptr = new ProtocolController();
  display_ptr = new Display();
  display_ptr->setStatus("Setup");
  display_ptr->setMode("0: WFI");
  PID_Controller_ptr = new PID_Controller(thetas, velocities);
  buzzer = new Buzzer(buzz_pin);
  buzzer->buzz(750);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(getAngleVel_pub);
  nh.subscribe(trajectorySubscriber);
  nh.subscribe(commandSub);
  nh.subscribe(modeSub);

  delay(750);
  display_ptr->setConnect("False"); //connected with PC (usually has not happened yet)

  //allocate memory for angleVel message
  angleVel_msg.data = (int*) malloc(sizeof(int) * 10);
  //set length of msg, otherwise everything will not be sent
  angleVel_msg.data_length = 10;

  buzzer->update();
  display_ptr->setStatus("Boot Act");

  bool init = false;
  while(!init)
  {
    init = true;
    for(int i = 1; i < 6; i++)
      if(!controler_ptr->ping(i))
        init = false;
  }

  setPWMMode(); //set motors to pwm mode

  setOffset = true;
  getPositionsVelocities();

  int tmp_trajectory[15] = {thetas[0]*1000.0, 0,0, thetas[1]*1000.0, 0,0,thetas[2]*1000.0, 0,0,thetas[3]*1000.0, 0,0,thetas[4]*1000.0, 0,0,};
  PID_Controller_ptr->trajectoryFunk(tmp_trajectory); //saves the trajectory
  float* torques = PID_Controller_ptr->update();  //calculate our tf torques with the controller
  enableTorque();
  updateTorques(torques); //send the torques to our motors

  display_ptr->setStatus("Running"); //good to go!
  runTime = millis();
}

void loop()
{
  unsigned long now = millis();
  if(now > runTime)
  {
    runTime += 33;
    buzzer->update();
    getPositionsVelocities();  //update data from motors
    float* torques = PID_Controller_ptr->update();  //calculate our torques with the controller
    updateTorques(torques); //send the torques to our motors
    nh.spinOnce();
    publishAngVel(); //publish to ros angles and velocities
    nh.spinOnce();

    //unsigned long start = micros();
     //process callback queue
    //unsigned long timeSpent = micros()-start;

    //runTime = millis()-now;
    //char result[8]; // Buffer big enough for 7-character float
    //dtostrf(runTime, 6, 2, result); // Leave room for too large numbers!
    //nh.loginfo(result);
    /*
    //Check if connected to ROS, and keep the status printed on the screen
    if(rosConnected == false)
    {
      if(nh.connected())
      {
        rosConnected = true;
        display_ptr->setConnect("True");
        buzzer->buzz(200);
      }
      else
        display_ptr->setConnect("False");
    }
    else if (checkTimer > 30) {
      if(!nh.connected()) //check if disconnected from ROS
      {
        rosConnected = false;
        int tmp_trajectory[15] = {thetas[0]*1000.0, 0,0, thetas[1]*1000.0, 0,0,thetas[2]*1000.0, 0,0,thetas[3]*1000.0, 0,0,thetas[4]*1000.0, 0,0,};
        PID_Controller_ptr->trajectoryFunk(tmp_trajectory); //saves the trajectory
      }

      checkTimer = 0;
    }
    checkTimer++;
    */
  }
}

void trajectory_sub(const std_msgs::Int16MultiArray& incoming_msg)
{
  PID_Controller_ptr->trajectoryFunk(incoming_msg.data); //saves the trajectory
}

void command_callback(const std_msgs::Int16& msg)
{
  if (msg.data == 0) //stop command
  {
    display_ptr->setLastLine("Stopped");
    disableTorque();
  }
  else if (msg.data == 1) // start command
  {
    setOffset = true;
    display_ptr->setLastLine("Started");
    PID_Controller_ptr->resetErrorSum();
    setPWMMode();
    enableTorque();
  }
  else if (msg.data == 2) // start command
  {
    setOffset = true;
    display_ptr->setLastLine("Started");
    PID_Controller_ptr->resetErrorSum();
  }
}

void mode_callback(const std_msgs::Int16& msg)
{
  buzzer->buzz(10, 1);
  switch (msg.data) {
    case 0:
      display_ptr->setMode("0: Waiting");
    break;
    case 1:
      display_ptr->setMode("1: Bas.Sho.");
    break;
    case 2:
      display_ptr->setMode("2: Elb.Grp.");
    break;
    case 3:
      display_ptr->setMode("3: Set Mac.");
    break;
    case 4:
      display_ptr->setMode("4: Use Mac.");
    break;
    case 5:
      display_ptr->setMode("5: IMU");
    break;
  }
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
  for (int i = 1; i < 6; i++)
  {
    controler_ptr->setTorque(i, torqueArray[i - 1], velocities[i - 1]);
  }
}

void publishAngVel() {
  //publish thetas and velocities:
  for (int i = 0; i < 5; i++)
  {
    angleVel_msg.data[i * 2] = thetas[i]*1000.0;      //factor 1000 because we send data as ints instead of floats
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
