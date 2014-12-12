//using for 3rd integration
#include <ros.h>
#include <smcDriver.h>
#include <PID_v1.h> //based on http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ 
#include <controller_input.h>
#include <controller_translational_constants.h>
#include <controller_rotational_constants.h>
#include <controller_onoff.h>
#include <thruster.h>
#include <SoftwareSerial.h>
#include <ArduinoHardware.h>


int manual_speed[6];
long time_elapsed;

/***********/

//initialize subscribers and publishers in ROS

ros::NodeHandle nh;

//teleopControl
void teleopControl(const bbauv_msgs::thruster &msg);
ros::Subscriber<bbauv_msgs::thruster> teleopcontrol_sub("teleop_controller",&teleopControl);

//Controller Mode
void updateControllerMode (const bbauv_msgs::controller_onoff &msg);
ros::Subscriber<bbauv_msgs::controller_onoff> controller_mode("controller_mode", &updateControllerMode);

//update PID Controller Constants; seperated into Rotation and Translational;
//Reason for separation: msg file cannot be too large, restriction from rosserial client
void updateTranslationalControllerConstants(const bbauv_msgs::controller_translational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_translational_constants> pidconst_trans_sub("translational_constants", &updateTranslationalControllerConstants);
void updateRotationalControllerConstants(const bbauv_msgs::controller_rotational_constants &msg);
ros::Subscriber<bbauv_msgs::controller_rotational_constants> pidconst_rot_sub("rotational_constants", &updateRotationalControllerConstants);

// Update Controller Input; contains both the setpoitn and sensor feedback
void updateControllerInput(const bbauv_msgs::controller_input &msg);
ros::Subscriber<bbauv_msgs::controller_input> controller_sub("controller_input",&updateControllerInput);

//Publish thruster speed
bbauv_msgs::thruster thrusterSpeed;
ros::Publisher thruster_pub("thruster_feedback",&thrusterSpeed);

/***********/

//smcDriver initialize
#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin
smcDriver mDriver= smcDriver(rxPin,txPin);

/***********/

//PID initialize
bool inDepthPID, inHeadingPID, inForwardPID, inSidemovePID, inTopside, inSuperimpose, inTeleop;

double depth_setpoint,depth_input,depth_output;
PID depthPID(&depth_input, &depth_output, &depth_setpoint,1,0,0, DIRECT);

double heading_setpoint,heading_input,heading_output;
PID headingPID(&heading_input, &heading_output, &heading_setpoint,1,0,0, DIRECT);

double forward_setpoint,forward_input,forward_output;
PID forwardPID(&forward_input, &forward_output, &forward_setpoint,1,0,0, DIRECT);

double sidemove_setpoint,sidemove_input,sidemove_output;
PID sidemovePID(&sidemove_input, &sidemove_output, &sidemove_setpoint,1,0,0, DIRECT);

/***********/

void prog_setup()
{

  //initialize value for variables
  inTopside=true;
  inTeleop=false;
  
  inDepthPID= false;
  inHeadingPID= false;
  inForwardPID=false;
  inSidemovePID=false; 
  
  //Initialize thruster ratio to 1:1:1:1:1:1
  float ratio[6]={0.8471, 0.9715, 0.9229, 0.9708, 0.8858, 1}; //see excel file in bbauv/clan folder
                          
  for(int i=0;i<6;i++)
    manual_speed[i]=0;
  
  time_elapsed=0;
    
  //initialize Motor driver and ROS
  mDriver.init();
  nh.initNode();
  nh.subscribe(controller_mode);
  nh.subscribe(controller_sub);
  nh.subscribe(pidconst_trans_sub);
  nh.subscribe(pidconst_rot_sub);
  nh.subscribe(teleopcontrol_sub);
  nh.advertise(thruster_pub);
  mDriver.setThrusterRatio(ratio);
  
  //Note the sample time here is 50ms. ETS SONIA's control loop runs at 70ms.

  depthPID.SetMode(AUTOMATIC);
  depthPID.SetSampleTime(5);
  depthPID.SetOutputLimits(-2560,2560);
  depthPID.SetControllerDirection(REVERSE);
  
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetSampleTime(5);
// too high a limit will result in too much overshoot.
  headingPID.SetOutputLimits(-800,800);
  headingPID.SetControllerDirection(DIRECT);
  
  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetSampleTime(5);
  forwardPID.SetOutputLimits(-1000,1260);
  forwardPID.SetControllerDirection(DIRECT);
  
  sidemovePID.SetMode(AUTOMATIC);
  sidemovePID.SetSampleTime(5);
  sidemovePID.SetOutputLimits(-500,500);
  sidemovePID.SetControllerDirection(DIRECT);
  
  pinMode(13, OUTPUT); 
  
}

/************************************************************/

void runThruster()
{
  
  mDriver.setMotorSpeed(1,thrusterSpeed.speed1);
  mDriver.setMotorSpeed(2,thrusterSpeed.speed2);
  mDriver.setMotorSpeed(3,thrusterSpeed.speed3);
  mDriver.setMotorSpeed(4,thrusterSpeed.speed4);
  mDriver.setMotorSpeed(5,thrusterSpeed.speed5);
  mDriver.setMotorSpeed(6,thrusterSpeed.speed6);

}

void getTeleopControllerUpdate()
{
  if(!inTeleop)
  {
  manual_speed[0]=0;
  manual_speed[1]=0;
  manual_speed[2]=0;
  manual_speed[3]=0;
  manual_speed[4]=0;
  manual_speed[5]=0;
  }
}

void getDepthPIDUpdate()
{
  depthPID.SetMode(inDepthPID);
  if(inDepthPID)
  {
    depthPID.Compute();
      /*
    if(inDepthPID && depth_setpoint == float(int(depth_input*10))/10) // && depth_setpoint-depth_input>-0.01)
    {
      
      thrusterSpeed.speed5=-1725;
      thrusterSpeed.speed6=-1725;
    }
    else
    {
      thrusterSpeed.speed5=depth_output+manual_speed[4];
      thrusterSpeed.speed6=depth_output+manual_speed[5];
    }*/
  }
  else
  {
    depth_output=0;
  }
}

void getHeadingPIDUpdate()
{
  
    if(heading_setpoint==0)
    {
      heading_setpoint=360;
    }
    if (heading_setpoint > (heading_input+180))
    {
      heading_input+=360;
    }  
  
  headingPID.SetMode(inHeadingPID);
  if(inHeadingPID)
  {
    headingPID.Compute();
  }
  else
  {
    heading_output=0;
  }
}

void getForwardPIDUpdate()
{
  forwardPID.SetMode(inForwardPID);
  if(inForwardPID)
  {
    forwardPID.Compute();
  }
  else
  {
    forward_output=0;
  }
}

void getSidemovePIDUpdate()
{
  sidemovePID.SetMode(inSidemovePID);
  if(inSidemovePID)
  {
    sidemovePID.Compute();
  }
  else
  {
    sidemove_output=0;
  }
  
}

void setZeroHorizThrust()
{

  thrusterSpeed.speed1=0;
  thrusterSpeed.speed2=0;
  thrusterSpeed.speed3=0;
  thrusterSpeed.speed4=0;
  
}

void setZeroVertThrust()
{
  thrusterSpeed.speed5=0;
  thrusterSpeed.speed6=0;

}

void setHorizThrustSpeed()
{
  thrusterSpeed.speed1=heading_output-forward_output+sidemove_output+manual_speed[0];
  thrusterSpeed.speed2=-heading_output-forward_output-sidemove_output+manual_speed[1];
  thrusterSpeed.speed3=heading_output+forward_output-sidemove_output+manual_speed[2];
  thrusterSpeed.speed4=-heading_output+forward_output+sidemove_output+manual_speed[3];   
}

void setVertThrustSpeed()
{
  thrusterSpeed.speed5=depth_output+manual_speed[4];
  thrusterSpeed.speed6=depth_output+manual_speed[5];
}

void calculateThrusterSpeed()
{
  getTeleopControllerUpdate();
  
  getDepthPIDUpdate();
  getHeadingPIDUpdate();
  getForwardPIDUpdate();
  getHeadingPIDUpdate();
  getSidemovePIDUpdate();
}

/* How are we sending same set of thrusters multiple PID outputs? */

void superImposePIDoutput()
{
  calculateThrusterSpeed();
  setHorizThrustSpeed();
  setVertThrustSpeed();
  //execute the calculated thruster speed

}

void rotatePIDoutput()
{
/*Experimental*/
  
  //Vertical
  inDepthPID=true;
  getDepthPIDUpdate();
  setVertThrustSpeed();
  runThruster();

  //Horizontal
  inHeadingPID=false; inForwardPID=true; inSidemovePID=false;
  getHeadingPIDUpdate();
  getForwardPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  //thruster_pub.publish(&thrusterSpeed);
  delay(5);

  inHeadingPID=true; inForwardPID=false; inSidemovePID=false;
  getHeadingPIDUpdate();
  getForwardPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  //thruster_pub.publish(&thrusterSpeed);
  delay(5);

/*
  nh.loginfo("Adjusting sideways");
  inHeadingPID=false; inForwardPID=false; inSidemovePID=true;
  getForwardPIDUpdate();
  setHorizThrustSpeed();
  runThruster();
  //setZeroHorizThrust();  
*/

}

/************************************************************/
// MAIN LOOP
//rosserial has been tested succesfully for message sizes (topic which are pushed to Arduino) 
//to be 14 fields of float32
//If a message type exceeds 14 fields of float32, testing shows that it will not work.\
// www.ros.org/wiki/rosserial/Overview/Limitations

void prog_loop()
{

    if (millis() > time_elapsed)
    {
    superImposePIDoutput();
    //rotatePIDoutput();
    
    thruster_pub.publish(&thrusterSpeed);
    runThruster();
    time_elapsed = millis() + 50; //instead of delay(), use millis
    }
  
    nh.spinOnce(); //if nh.spinOnce is not called regularly, will result in lost sync issues with ROS.
  
}    


/************************************************************/

/****** ROS Subsriber Call Back Functions *********/

void updateControllerMode (const bbauv_msgs::controller_onoff &msg)
{
  inTopside=msg.topside;
  
  if(inTopside)
  {
  inTeleop=msg.teleop;    
  inDepthPID= msg.depth_PID;
  inForwardPID= msg.forward_PID;
  inSidemovePID= msg.sidemove_PID;
  inHeadingPID= msg.heading_PID;

  }
  else
  {
  inTeleop=false;
  inDepthPID=true;
  inForwardPID=true;
  inSidemovePID=true;
  inHeadingPID=true;
  }

}

void updateTranslationalControllerConstants(const bbauv_msgs::controller_translational_constants &msg)
{

  depthPID.SetTunings(msg.depth_kp,msg.depth_ki,msg.depth_kd);
  forwardPID.SetTunings(msg.forward_kp,msg.forward_ki,msg.forward_kd);
  sidemovePID.SetTunings(msg.sidemove_kp,msg.sidemove_ki,msg.sidemove_kd);

}

void updateRotationalControllerConstants(const bbauv_msgs::controller_rotational_constants &msg)
{
  headingPID.SetTunings(msg.heading_kp,msg.heading_ki,msg.heading_kd);
}


void updateControllerInput(const bbauv_msgs::controller_input &msg)
{
  /*setpoint and input of a PID must be of the same units */
  
  depth_input=msg.depth_input;
  depth_setpoint=msg.depth_setpoint;
  
  heading_input=msg.heading_input;
  heading_setpoint=msg.heading_setpoint;
  
  forward_input=msg.forward_input;
  forward_setpoint=msg.forward_setpoint;
  
  sidemove_input=msg.sidemove_input;
  sidemove_setpoint=msg.sidemove_setpoint;
 
}


void teleopControl(const bbauv_msgs::thruster &msg)
{
  manual_speed[0]=msg.speed1;
  manual_speed[1]=msg.speed2;
  manual_speed[2]=msg.speed3;
  manual_speed[3]=msg.speed4;
  manual_speed[4]=msg.speed5;
  manual_speed[5]=msg.speed6;
}

