#include <smcDriver.h>
#include <PID_v1.h>
#include <controller_input.h>
#include <controller_param.h>
#include <thruster.h>
#include <ros.h>
#include <SoftwareSerial.h>

//ROS initialize
ros::NodeHandle nh;

void updateControllerParam(const bbauv_msgs::controller_param &msg);
ros::Subscriber<bbauv_msgs::controller_param> pidconst_sub("controller_config",&updateControllerParam);

void updateControllerInput(const bbauv_msgs::controller_input &msg);
ros::Subscriber<bbauv_msgs::controller_input> controller_sub("controller_input",&updateControllerInput);

//smcDriver initialize
#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin
smcDriver mDriver= smcDriver(rxPin,txPin);

//PID initialize
double depth_setpoint,depth_input,depth_output;
PID depthPID(&depth_input, &depth_output, &depth_setpoint,1,0,0, DIRECT);
bool inPID;

void setup()
{
   pinMode(13, OUTPUT); 
  //initialize value for variables
  depth_input=0;
  depth_setpoint=0;
  depth_output=0;
  //initialize Motor driver and ROS
  mDriver.init();
  nh.initNode();
  nh.subscribe(controller_sub);
  nh.subscribe(pidconst_sub);

  //setup depth PID
  depthPID.SetMode(AUTOMATIC);
  depthPID.SetSampleTime(10);
  depthPID.SetOutputLimits(-3200,3200);

}

void loop()
{
  //all is written in ISR
  depthPID.SetMode(inPID);
  if(inPID)
  {
    depthPID.Compute();
    digitalWrite(13, HIGH);   // set the LED on
    inPID=0;
  }
  else
  {
     depth_output=0;
     digitalWrite(13, LOW);    // set the LED off
     inPID=1;
  }
  /*mDriver.setMotorSpeed(1,depth_output);
  mDriver.setMotorSpeed(2,depth_output);
  delay(100);*/
  nh.spinOnce();
  delay(10);
}    


void updateControllerParam(const bbauv_msgs::controller_param &msg)
{
  float ratio[6]={msg.ratio_t1, msg.ratio_t2, msg.ratio_t3, 
                           msg.ratio_t4, msg.ratio_t5,msg.ratio_t6};
  mDriver.setThrusterRatio(ratio);
  depthPID.SetTunings(msg.depth_kp,msg.depth_ki,msg.depth_kd);
  inPID= msg.mode;
}

void updateControllerInput(const bbauv_msgs::controller_input &msg)
{
  depth_input=msg.depth_input;
  depth_setpoint=msg.depth_setpoint;
}


/*
int manual_speed;
void teleopControl(const bbauv_msgs::thruster &msg)
{
   manual_speed = msg.speed5;
}
ros::Subscriber<bbauv_msgs::thruster> control_sub("teleop_controller",&teleopControl);
*/
