//Version 4: adding Servo, update maximum speed

//------------------ Includes--------------  //External libraries
    #include <ros.h>
    #include <smcDriver_v2.h>  //From Pololu Robotics and Electronics
    #include <PID_v1.h> //@Author: Brett, Website: brettbeauregard.com
  //ROS Messages' libraries for Arduino
    #include <controller_input.h>
    #include <controller_translational_constants.h>
    #include <controller_rotational_constants.h>
    #include <controller_onoff.h>
    #include <thruster.h>
  //Arduino libraries
    #include <SoftwareSerial.h>
    #include <ArduinoHardware.h>
    #include <Servo.h>

//------------------ Global variables ---------------------
  //Timming variables - to ensure the loop run at correct frequency
  long currentTime,loopTime,time_elapsed;

  //Control parameter
  int manual_speed[6];

  //Servo Handler
  Servo myservo;

//------------------ Setup subscribers, publishers & Call back functions in ROS -----------------------------
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

//------------------ Setup PID ---------------------------------
  //PID control parameters
  bool inDepthPID, inHeadingPID, inForwardPID, inSidemovePID, inTopside, inSuperimpose, inTeleop;

  double depth_setpoint,depth_input,depth_output;
  PID depthPID(&depth_input, &depth_output, &depth_setpoint,1,0,0, DIRECT);

  double heading_setpoint,heading_input,heading_output;
  PID headingPID(&heading_input, &heading_output, &heading_setpoint,1,0,0, DIRECT);

  double forward_setpoint,forward_input,forward_output;
  PID forwardPID(&forward_input, &forward_output, &forward_setpoint,1,0,0, DIRECT);

  double sidemove_setpoint,sidemove_input,sidemove_output;
  PID sidemovePID(&sidemove_input, &sidemove_output, &sidemove_setpoint,1,0,0, DIRECT);

//------------------ Setup smcDriver --------------------------
  smcDriver mDriver(&Serial1); //Use Serial1 to handle UART communication with motor controllers

void setup()
{
  //Initialize Value for PID Control (Mode) Variables
    inTopside=true;
    inTeleop=false;
    
    inDepthPID= false;
    inHeadingPID= false;
    inForwardPID=false;
    inSidemovePID=false; 
  
  //Initialize Motor Controller
    //Set Baud rate for Serial1, which is used for UART connection
    Serial1.begin(115200);
    mDriver.init();
    //Set Thruster Ratio:
    //float ratio[6]={0.8471, 0.9715, 0.9229, 0.9708, 0.8858, 1}; 
    //mDriver.setThrusterRatio(ratio);
    
  //Initialize Manual Control
   for(int i=0;i<6;i++) 
    manual_speed[i]=0;
    
  //Initialize ROS: publishers, subscribers
    nh.initNode();
    nh.subscribe(controller_mode);
    nh.subscribe(controller_sub);
    nh.subscribe(pidconst_trans_sub);
    nh.subscribe(pidconst_rot_sub);
    nh.subscribe(teleopcontrol_sub);
    nh.advertise(thruster_pub);

  //Initialize PIDs
    depthPID.SetMode(AUTOMATIC);
    depthPID.SetSampleTime(20);
    depthPID.SetOutputLimits(-3200,3200);
    depthPID.SetControllerDirection(REVERSE);
    
    headingPID.SetMode(AUTOMATIC);
    headingPID.SetSampleTime(20);
    // too high a limit will result in too much overshoot.
    headingPID.SetOutputLimits(-800,800);
    headingPID.SetControllerDirection(DIRECT);
    
    forwardPID.SetMode(AUTOMATIC);
    forwardPID.SetSampleTime(20);
    forwardPID.SetOutputLimits(-2400,2400);
    forwardPID.SetControllerDirection(DIRECT);
    
    sidemovePID.SetMode(AUTOMATIC);
    sidemovePID.SetSampleTime(20);
    sidemovePID.SetOutputLimits(-1200,1200);
    sidemovePID.SetControllerDirection(DIRECT);

  //Initialize Servos
    myservo.attach(9);
    myservo.write(0);

  //Initialize MainLoop Timming variables
    time_elapsed=0;
    currentTime=millis();
    loopTime=currentTime;
    pinMode(13, OUTPUT);  //For debugging main loop speed
    //Note the sample time here is 50ms. ETS SONIA's control loop runs at 70ms.
}

/****************** FUNCTIONS FOR CONTROLLERS ****************/
// ------- To access Motor Controller ---------
  void runThruster()
  {
    mDriver.setMotorSpeed(1,thrusterSpeed.speed1);
    mDriver.setMotorSpeed(2,thrusterSpeed.speed2);
    mDriver.setMotorSpeed(3,thrusterSpeed.speed3);
    mDriver.setMotorSpeed(4,thrusterSpeed.speed4);
    mDriver.setMotorSpeed(5,thrusterSpeed.speed5);
    mDriver.setMotorSpeed(6,thrusterSpeed.speed6);
  }
// ------- Multiples PIDs Algorithm -----------

  //getters & setters:
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
      if (heading_input > (heading_setpoint+180))
      {
        heading_input-=360;
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
  }

/****************** MAIN LOOP ********************/
  //rosserial has been tested succesfully for message sizes (topic which are pushed to Arduino) 
  //to be 14 fields of float32
  //If a message type exceeds 14 fields of float32, testing shows that it will not work.\
  // www.ros.org/wiki/rosserial/Overview/Limitations
void loop()
{
  currentTime=millis();
  if( currentTime >= (loopTime + 40))
  {
    digitalWrite(13,HIGH);
    superImposePIDoutput();
    //rotatePIDoutput();
    
    thruster_pub.publish(&thrusterSpeed);
    runThruster();
    nh.spinOnce(); //if nh.spinOnce is not called regularly, will result in lost sync issues with ROS.
    digitalWrite(13,LOW);   
    loopTime=currentTime;
  }
}

/************ ROS Subsriber Call Back Functions **********/
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