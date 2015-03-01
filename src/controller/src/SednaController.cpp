// #define REVERSE   change the yaw controller output direction

#define LARGE_RANGE 300
#define SMALL_RANGE 200
#define SAFETY_RANGE 400
#define SEABOTIX_LIMIT 255


#include <math.h>
#include "PID.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <srmauv_msgs/thruster.h>
#include <srmauv_msgs/depth.h>
#include <srmauv_msgs/compass_data.h>
#include <srmauv_msgs/ControllerAction.h> //action
#include <srmauv_msgs/imu_data.h>
#include <srmauv_msgs/set_controller.h> //service
#include <srmauv_msgs/pid_info.h>
#include <srmauv_msgs/teleop_sedna.h>
#include <dynamic_reconfigure/server.h>
#include <controller/controllerConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <PID_controller/PID.h>
#include <NavUtils/NavUtils.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <ControllerServer/ControllerServer.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <srmauv_msgs/controller.h>
#include <srmauv_msgs/locomotion_mode.h>
#include <srmauv_msgs/thruster.h>
#include <geometry_msgs/Pose2D.h>
//this will be calibrated from the sensor
const static int loop_frequency=20;
const static int PSI30 = 206842;
const static int PSI100 = 689475;
const static int ATM = 99974;

double          thruster1_ratio = 1,
		thruster2_ratio = 1,
		thruster3_ratio= 1,
		thruster4_ratio = 1,
		thruster5_ratio =1,
		thruster6_ratio =1,
		thruster7_ratio=1,
		thruster8_ratio=1,
		thruster1_rev_ratio=1,
		thruster2_rev_ratio=1,
		thruster3_rev_ratio=1,
		thruster4_rev_ratio=1,
		thruster5_rev_ratio=1,
		thruster6_rev_ratio=1,
		thruster7_rev_ratio=1,
		thruster8_rev_ratio=1;


srmauv_msgs::thruster thruster;
srmauv_msgs::controller ctrl ;
srmauv_msgs::depth depthValue;
srmauv_msgs::thruster thrusterSpeed;
srmauv_msgs::compass_data orientationAngles;
srmauv_msgs::pid_info pidInfo;
nav_msgs::Odometry odomData;
std_msgs::Int8 current__mode;
srmauv_msgs::teleop_sedna teleop;
//teleop.depth_enable=false;
double depth_offset = 0;

struct teleop_speed{
  double sidemove;
  double forward;
  double reverse;
}teleop_velocity;


void getVisionSidemove(const std_msgs::Int16& msg);
void getOrientation(const sensor_msgs::Imu::ConstPtr& msg);
void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg);
void getPressure(const srmauv_msgs::depth& msg);
//void getTeleop(const srmauv_msgs::thruster::ConstPtr& msg);
void getTeleop(const srmauv_msgs::teleop_sedna::ConstPtr &msg);
void callback(controller::controllerConfig &config,uint32_t level);
double getHeadingPIDUpdate(); // some angle wrapping required 
float computeVelSideOffset();
float computeVelFwdOffset();
float interpolateDepth(float);
void setHorizontalThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output);
void setVerticalThrustSpeed(double depthPID_output,double pitchPID_output,double rollPID_output);
double fmap (int input, int in_min, int in_max, int out_min, int out_max);


int  limitSeabotix(int speed);


//State Machines : 
bool inTop,inTeleop,inHovermode=false,oldHovermode=false;
bool inDepthPID,inHeadingPID,inForwardPID,inSidemovePID,inPitchPID,inRollPID,
	inForwardVelPID,inSidemoveVelPID;
bool inNavigation;
bool velocityMode;
bool inVisionTracking;
bool isForward=false;
bool isSidemove=false;


ros::Publisher thrusterPub;
ros::Publisher controllerPub;
ros::Publisher pid_infoPub;
ros::Publisher orientationPub;
ros::Publisher depthPub;
ros::Publisher locomotionModePub;

ros::Subscriber orientationSub;
ros::Subscriber pressureSub;
ros::Subscriber earthSub;
ros::Subscriber teleopSub;
ros::Subscriber autonomousSub; // ~
ros::Subscriber velocitySub;
ros::Subscriber headingSub;

srmauv::sednaPID depthPID("d",1.2,0,0,20);
srmauv::sednaPID headingPID("h",1.2,0,0,20);
srmauv::sednaPID pitchPID("p",1.2,0,0,20);

srmauv::sednaPID rollPID("r",1.2,0,0,20);
srmauv::sednaPID forwardPID("f",1.2,0,0,20);
srmauv::sednaPID sidemovePID("s",1.2,0,0,20);
srmauv::sednaPID forwardVelPID("vf",0,0,0,20);
srmauv::sednaPID sidemoveVelPID("vS",0,0,0,20);

int act_forward[2];
int act_sidemove[2];
int act_heading[2];

//index 0,1 and 2,3 are forward and sidemove settings respectively
int loc_mode_forward[4] = {-LARGE_RANGE,LARGE_RANGE,-SMALL_RANGE,SMALL_RANGE};
int loc_mode_sidemove[4] = {-SMALL_RANGE,SMALL_RANGE,-LARGE_RANGE,LARGE_RANGE};
int loc_mode_heading[4] = {-SMALL_RANGE,SMALL_RANGE,-SMALL_RANGE,SMALL_RANGE};



//sets the correct actuator models 
bool locomotion_srv_handler(srmauv_msgs::locomotion_mode::Request &req, 
				srmauv_msgs::locomotion_mode::Response &res)
{
	int mode=0;
	if(req.forward && req.sidemove){
		res.success=false;
		return true;	
	}

	else if (req.forward)	{
		headingPID.setActuatorSatModel(loc_mode_heading[0],loc_mode_heading[1]); //ACTMIN and ACTMAX
		forwardPID.setActuatorSatModel(loc_mode_forward[0],loc_mode_forward[1]);
		sidemovePID.setActuatorSatModel(loc_mode_sidemove[0],loc_mode_sidemove[1]);
		ROS_INFO("We are in Surge Mode " );
		mode=1;
		res.success=true;
	}
	
	else if(req.sidemove){
		headingPID.setActuatorSatModel(loc_mode_heading[2],loc_mode_heading[3]); //ACTMIN and ACTMAX
                forwardPID.setActuatorSatModel(loc_mode_forward[2],loc_mode_forward[3]);
                sidemovePID.setActuatorSatModel(loc_mode_sidemove[2],loc_mode_sidemove[3]);
		ROS_INFO("We are in Sway Mode");
		res.success=true;
		mode=2;
	}
	else if (!req.forward && !req.sidemove){  //no mode set
		mode=0;
		//switch to the default mode
		ROS_INFO("h_min: %i ,h_max: %i ,f_min: %i ,f_max: %i ,s_min: %i, s_max: %i",act_heading[0],act_heading[1],act_forward[0],act_heading[0],act_heading[1],act_sidemove[0],act_sidemove[1]);
		res.success=true; // default mode has been set
		headingPID.setActuatorSatModel(act_heading[0],act_heading[1]);
		forwardPID.setActuatorSatModel(act_forward[0],act_forward[1]);
		sidemovePID.setActuatorSatModel(act_sidemove[0],act_sidemove[1]);
		ROS_INFO("We are in Default movement mode");
		res.success=true;
	}		
	
	else{
		res.success=false;
	}
	std_msgs::Int8 current_mode;
	current_mode.data=mode;
	locomotionModePub.publish(current_mode); //publish the mode we are in
	return true;
	
}

//the controller service call can enable/disable the various PID controllers
bool controller_srv_handler(srmauv_msgs::set_controller::Request &req,
				srmauv_msgs::set_controller::Response &res){
	inDepthPID=req.depth;
	inForwardPID=req.forward;
	inSidemovePID=req.sidemove;
	inHeadingPID=req.heading;
	inPitchPID=req.pitch;
	inRollPID=req.roll;
	inForwardVelPID=req.forward_vel;
	inSidemoveVelPID=req.sidemove_vel;
	inNavigation=req.navigation;
	
	res.complete=true;
	return true;
}




int main (int argc,char **argv){
	ros::init(argc,argv,"controller");
	double forward_output,pitch_output,roll_output,heading_output,sidemove_output,depth_output;
	double forward_vel_output,sidemove_vel_output;

	ros::NodeHandle nh;
	
	teleop.tune=true;
	teleop.depth_enable=false;
//	ctrl.heading_setpoint=20;
	thrusterPub=nh.advertise<srmauv_msgs::thruster>("/thruster_speed",1000);
	//depthPub=nh.advertise<srmauv_msgs::depth>("/depth",1000);
	controllerPub=nh.advertise<srmauv_msgs::controller>("/controller_targets",100);   // verified with tuining_ui
	locomotionModePub=nh.advertise<std_msgs::Int8>("/locomotion_mode",100,true);
	pid_infoPub=nh.advertise<srmauv_msgs::pid_info>("/pid_info",1000);   // verified with tuining_ui
	  headingSub=nh.subscribe("/imu/HeadingTrue_degree",1000,getHeading);

	//subscribers: 

	//DVL here:	velocitySub=nh.subscribe("
	orientationSub=nh.subscribe("/imu/data",1000,getOrientation);
	pressureSub=nh.subscribe("/pressure_data",1000,getPressure);
	teleopSub=nh.subscribe("/teleop_sedna",1000,getTeleop);
	//Dynamic reconfigure:
	dynamic_reconfigure::Server <controller::controllerConfig>server;
	dynamic_reconfigure::Server<controller::controllerConfig>::CallbackType f;
	f=boost::bind(&callback,_1,_2);
	server.setCallback(f);

	//initialize services
	
	ros::ServiceServer locomotion_service=nh.advertiseService("controller/locomotion_mode_srv",locomotion_srv_handler);
	ROS_INFO("Change modes using locomotion_mode_srv");
	
	ros::ServiceServer service=nh.advertiseService("controller/set_controller_srv",controller_srv_handler);
	ROS_INFO("Enable disable PID's using set_controller_srv");

	//Initialize the ActionServer:

	ControllerServer as("LocomotionServer");
	//PID Loop will run at
	ros::Rate loop_rate(loop_frequency);
	
	//publish the initial standard mode locomotion mode message
	std_msgs::Int8 std_mode;
	std_mode.data=0;
	locomotionModePub.publish(std_mode);	
	ROS_INFO("PID controllers are ready lets roll.. !");
	
	while(ros::ok())
	{
		if(inHovermode && oldHovermode!=inHovermode) // so we hover over a point
		{
			//ctrl.forward_setpoint=ctrl.forward_input;
			//ctrl.sidemove_setpoint=ctrl.sidemove_input;
			//ctrl.depth_setpoint=ctrl.depth_input;
			//ctrl.pitch_setpoint=ctrl.depth_input;
			//ctrl.heading_setpoint=ctrl.heading_input;
			
			oldHovermode=inHovermode;			
		

		}

		if(inSidemovePID)
		{
			sidemove_output=sidemovePID.computePID((double)0.0,ctrl.sidemove_input);
			pidInfo.sidemove.p=sidemovePID.getProportional();
                        pidInfo.sidemove.i=sidemovePID.getIntegral();
                        pidInfo.sidemove.d=sidemovePID.getDerivative();
			pidInfo.sidemove.total=sidemove_output;
			

		}
		else
		{
			sidemove_output=0;
			sidemovePID.clearIntegrator();
		}

		if(inForwardPID)
		                {
		                        forward_output=forwardPID.computePID((double)0.0,ctrl.forward_input);
		                        pidInfo.forward.p=forwardPID.getProportional();
		                        pidInfo.forward.i=forwardPID.getIntegral();
		                        pidInfo.forward.d=forwardPID.getDerivative();
		                        pidInfo.forward.total=forward_output;


		                }
		                else
		                {
		                        forward_output=0;
		                        forwardPID.clearIntegrator();
		                }
		if(inDepthPID)
		{
			depth_output=depthPID.computePID((double)ctrl.depth_setpoint,ctrl.depth_input)*-1;
			pidInfo.depth.p=depthPID.getProportional();
			pidInfo.depth.i=depthPID.getIntegral();
			pidInfo.depth.d=depthPID.getDerivative();
			pidInfo.depth.total=depth_output;
		//	ROS_INFO("P: %d\tI:%d\tD:%d\t total: %d\n",depthPID.getProportional(),depthPID.getIntegral(),depthPID.getDerivative(),depthPID.getTotal());
			
		}
		else{
			depth_output=0;
			depthPID.clearIntegrator();
		}
		if(inHeadingPID)
		{
				heading_output=getHeadingPIDUpdate();
				pidInfo.heading.p=headingPID.getProportional();
				pidInfo.heading.i=headingPID.getIntegral();
				pidInfo.heading.d=headingPID.getDerivative();
				pidInfo.heading.total=headingPID.getTotal();

		}
		else{
			heading_output=0;
			headingPID.clearIntegrator();
		}

		if(inPitchPID){
			pitch_output=pitchPID.computePID((double)ctrl.pitch_setpoint,ctrl.pitch_input);
			 pidInfo.pitch.p=pitchPID.getProportional();
                        pidInfo.pitch.i=pitchPID.getIntegral();
                        pidInfo.pitch.d=pitchPID.getDerivative();
                        pidInfo.pitch.total=pitch_output;



		}
		else{
			pitch_output=0;
			pitchPID.clearIntegrator();
		}

		if(inRollPID){
		                        roll_output=rollPID.computePID((double)ctrl.roll_setpoint,ctrl.roll_input);
		                         pidInfo.roll.p=rollPID.getProportional();
		                        pidInfo.roll.i=rollPID.getIntegral();
		                        pidInfo.roll.d=rollPID.getDerivative();
		                        pidInfo.roll.total=roll_output;



		                }
		                else{
		                        roll_output=0;
		                        rollPID.clearIntegrator();
		                }

		setHorizontalThrustSpeed(heading_output,forward_output,sidemove_output);
		setVerticalThrustSpeed(depth_output,pitch_output,roll_output);

		controllerPub.publish(ctrl);
		pid_infoPub.publish(pidInfo);
		thrusterPub.publish(thrusterSpeed);
		ROS_DEBUG(" F %i, SM%i, H %i, P %i, R %i, D %i, Nav %i",inForwardPID,inSidemovePID,inHeadingPID,inPitchPID, inRollPID, inDepthPID,inNavigation);
		ros::spinOnce();
		loop_rate.sleep();
}
	return 0;
}



float interpolateDepth(float adcVal){
	//do depth interpolation here
	return adcVal;
}
	
double getHeadingPIDUpdate(){
	//this piece may be a cause of some major issues related to the heading controller.. make changes as needed : Akshaya
	double wrappedHeading;
	double error=(double)ctrl.heading_setpoint-(double)ctrl.heading_input;
	wrappedHeading=headingPID.wrapAngle360(error,ctrl.heading_input);
	return headingPID.computePID(ctrl.heading_setpoint,wrappedHeading);
}






void getPressure(const srmauv_msgs::depth &msg){
	
	// the message that is coming is the raw ADC value from the pressure sensor.. lets try to add some filtering to it somewhere else probably

	double depth=(double)interpolateDepth((float)msg.depth);
	
	//ROS_INFO("%d",msg.depth);

	depthValue.pressure=msg.depth;
	depthValue.depth=msg.depth;
	ctrl.depth_input=msg.depth;
}

void getOrientation(const sensor_msgs::Imu::ConstPtr& msg){
        tf::Quaternion q;
        double roll,pitch,yaw;


        tf::quaternionMsgToTF(msg->orientation,q);
      // tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
       // tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
        tf::Matrix3x3(q).getEulerZYX(yaw,pitch,roll);
      //  tf::Matrix3x3(q).
	//ctrl.heading_input=yaw;
	ctrl.pitch_input=pitch/M_PI*180;
	ctrl.roll_input=roll/M_PI*180;
	//int x=5;
	//ROS_INFO("%f\t%f\t%f\t",yaw,pitch,roll);


}

void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg){
  ctrl.heading_input=msg->theta;
}


int limitSeabotix(int speed){
  /*
  if (speed>SEABOTIX_LIMIT)
    speed=SEABOTIX_LIMIT;
  else if(speed<-SEABOTIX_LIMIT)
    speed=-SEABOTIX_LIMIT;
  return speed;
*/

  if( speed==0)
      return 0;
  int out=(int)fmap(speed,0,255,24,255);
  return out;

}



double fmap(int input, int in_min, int in_max, int out_min, int out_max){

  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




void setHorizontalThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output)
{
  //write code for forward movement

	double speed1_output;
	double speed2_output;

  if(teleop_velocity.forward + forwardPID_output<0){
	 speed1_output=(int)(thruster1_rev_ratio*(teleop_velocity.forward+ forwardPID_output));
	 speed2_output=(int)(thruster2_rev_ratio*(teleop_velocity.forward +forwardPID_output));

}
	else{
   speed1_output=(int)(thruster1_ratio*(teleop_velocity.forward +forwardPID_output));
   speed2_output=(int)(thruster2_ratio*(teleop_velocity.forward +forwardPID_output));
}

	
  
if(speed1_output>SEABOTIX_LIMIT)
    thrusterSpeed.speed1=SEABOTIX_LIMIT;
  else if(speed1_output<-SEABOTIX_LIMIT)
    thrusterSpeed.speed1=-SEABOTIX_LIMIT;
  else
    thrusterSpeed.speed1=speed1_output;

  if(speed2_output>SEABOTIX_LIMIT)
    thrusterSpeed.speed2=SEABOTIX_LIMIT;
  else if(speed2_output<-SEABOTIX_LIMIT)
    thrusterSpeed.speed2=-SEABOTIX_LIMIT;
  else
    thrusterSpeed.speed2=speed2_output;

bool neg;
double toFmap;
if(headingPID_output<0)	
	neg=true;

if(neg){
	toFmap=-headingPID_output;
}
else{
	toFmap=headingPID_output;
}

toFmap=limitSeabotix(toFmap);

if(neg){
	headingPID_output=-toFmap;
}

else{
	headingPID_output=toFmap;
}

#ifndef REVERSE
  double speed7_output=(-headingPID_output+teleop_velocity.sidemove-sidemovePID_output);
  double speed8_output=(headingPID_output+teleop_velocity.sidemove-sidemovePID_output);
#endif

#ifdef REVERSE
  double speed7_output=(headingPID_output+teleop_velocity.sidemove-sidemovePID_output);
  double speed8_output=(-headingPID_output+teleop_velocity.sidemove-sidemovePID_output);
#endif

if(speed7_output<0)
{
speed7_output=(int)(thruster7_rev_ratio*speed7_output);
}
else
{
speed7_output=(int)(thruster7_ratio*speed7_output);
}

if(speed8_output<0)
{
speed8_output=(int)(thruster8_rev_ratio*speed8_output);
}
else
{
speed8_output=(int)(thruster8_ratio*speed8_output);    
}
	

//  speed7_output=fmap(speed7_output,)
  if(speed7_output>SEABOTIX_LIMIT)
    thrusterSpeed.speed7=SEABOTIX_LIMIT;
  else if(speed7_output<-SEABOTIX_LIMIT)
    thrusterSpeed.speed7=-SEABOTIX_LIMIT;
  else
    thrusterSpeed.speed7=speed7_output;

  if(speed8_output>SEABOTIX_LIMIT)
    thrusterSpeed.speed8=SEABOTIX_LIMIT;
  else if(speed8_output<-SEABOTIX_LIMIT)
    thrusterSpeed.speed8=-SEABOTIX_LIMIT;
  else
    thrusterSpeed.speed8=speed8_output;



}


void setVerticalThrustSpeed(double depthPID_output,double pitchPID_output,double rollPID_output)
{


  double speed3_output = thruster3_ratio*(- depthPID_output + pitchPID_output - rollPID_output);
  double speed4_output = thruster4_ratio*(- depthPID_output + pitchPID_output + rollPID_output);
  double speed5_output = thruster5_ratio*(- depthPID_output - pitchPID_output + rollPID_output);
  double speed6_output = thruster6_ratio*(- depthPID_output - pitchPID_output - rollPID_output);

  if(speed3_output < -SAFETY_RANGE) thrusterSpeed.speed3 = -SAFETY_RANGE;
      else if(speed3_output >SAFETY_RANGE) thrusterSpeed.speed3 = SAFETY_RANGE;
      else thrusterSpeed.speed3= speed3_output;

      if(speed4_output < -SAFETY_RANGE) thrusterSpeed.speed4 = -SAFETY_RANGE;
      else if(speed4_output >SAFETY_RANGE) thrusterSpeed.speed4 = SAFETY_RANGE;
      else thrusterSpeed.speed4= speed4_output;

      if(speed5_output < -SAFETY_RANGE) thrusterSpeed.speed5 = -SAFETY_RANGE;
      else if(speed5_output >SAFETY_RANGE) thrusterSpeed.speed5 = SAFETY_RANGE;
      else thrusterSpeed.speed5= speed5_output;

      if(speed6_output < -SAFETY_RANGE) thrusterSpeed.speed6 = -SAFETY_RANGE;
      else if(speed6_output >SAFETY_RANGE) thrusterSpeed.speed6 = SAFETY_RANGE;
      else thrusterSpeed.speed6= speed6_output;



}



void callback(controller::controllerConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request");

//	if(teleop.tune){
	  ROS_INFO("Tuner Enabled");

//	ctrl.depth_input=config.depth_input;

	thruster1_ratio=config.thruster1_ratio;
	thruster2_ratio=config.thruster2_ratio;
	thruster3_ratio=config.thruster3_ratio;
	thruster4_ratio=config.thruster4_ratio;
	thruster5_ratio=config.thruster5_ratio;
	thruster6_ratio=config.thruster6_ratio;
	thruster7_ratio=config.thruster7_ratio;
	thruster8_ratio=config.thruster8_ratio;

	thruster1_rev_ratio=config.thruster1_rev_ratio;
        thruster2_rev_ratio=config.thruster2_rev_ratio;
        thruster3_rev_ratio=config.thruster3_rev_ratio;
        thruster4_rev_ratio=config.thruster4_rev_ratio;
        thruster5_rev_ratio=config.thruster5_rev_ratio;
        thruster6_rev_ratio=config.thruster6_rev_ratio;
        thruster7_rev_ratio=config.thruster7_rev_ratio;
        thruster8_rev_ratio=config.thruster8_rev_ratio;


	inForwardPID=config.forward_PID;
	inSidemovePID=config.sidemove_PID;
	inDepthPID=config.depth_PID;
	inHeadingPID=config.heading_PID;
	inPitchPID=config.pitch_PID;
	inRollPID=config.roll_PID;


	//inTeleop=config.teleop;
	//inHovermode=config.hovermode;

	ctrl.depth_setpoint=config.depth_setpoint;
	ctrl.heading_setpoint=config.heading_setpoint;
	ctrl.roll_setpoint=config.roll_setpoint;
	ctrl.pitch_setpoint=config.pitch_setpoint;

	depthPID.setKp(config.depth_Kp);
	depthPID.setTd(config.depth_Td);
	depthPID.setTi(config.depth_Ti);
	depthPID.setActuatorSatModel(config.depth_min,config.depth_max);

        pitchPID.setKp(config.pitch_Kp);
        pitchPID.setTd(config.pitch_Td);
        pitchPID.setTi(config.pitch_Ti);
        pitchPID.setActuatorSatModel(config.pitch_min,config.pitch_max);

	headingPID.setKp(config.heading_Kp);
	headingPID.setTd(config.heading_Td);
	headingPID.setTi(config.heading_Ti);
        headingPID.setActuatorSatModel(config.heading_min,config.heading_max);

        rollPID.setKp(config.roll_Kp);
        rollPID.setTd(config.roll_Td);
        rollPID.setTi(config.roll_Ti);
        rollPID.setActuatorSatModel(config.roll_min,config.roll_max);
	
	sidemovePID.setKp(config.sidemove_Kp);
        sidemovePID.setTd(config.sidemove_Td);
        sidemovePID.setTi(config.sidemove_Ti);
        sidemovePID.setActuatorSatModel(config.sidemove_min,config.sidemove_max);

        forwardPID.setKp(config.forward_Kp);
        forwardPID.setTd(config.forward_Td);
        forwardPID.setTi(config.forward_Ti);
        forwardPID.setActuatorSatModel(config.forward_min,config.forward_max);

     //   forwardPID.setKp(config.forward_Kp);
     //   forwardPID.setTd(config.forward_Td);
     //   forwardPID.setTi(config.forward_Ti);
//	}

}



void getTeleop(const srmauv_msgs::teleop_sedna::ConstPtr &msg){
 teleop_velocity.sidemove=msg->sidemove_speed;
 teleop_velocity.forward=msg->forward_speed;
 teleop.tune=msg->tune;



 if(!teleop.tune){
   ctrl.depth_setpoint=msg->depth_setpoint;
   ctrl.heading_setpoint=msg->heading_setpoint;
   ctrl.pitch_setpoint=msg->pitch_setpoint;
   ctrl.roll_setpoint=msg->roll_setpoint;
   //teleop.depth_enable=msg->depth_enable;
   inDepthPID=msg->depth_enable;
   ROS_INFO("inDepth [%d]",inDepthPID);
   inHeadingPID=inPitchPID=inRollPID=msg->pid_enable;
	//inSidemovePID=true; /// this might not be a good idea.. but sidemove pid is always on 
	ctrl.sidemove_input=msg->sidemove_input;   // if not using sidemove make sidemove_input = 0
	ctrl.forward_input=msg->forward_input;
	if(msg->sidemove_input==0)
		inSidemovePID=false;
	else
		inSidemovePID=true;
	if(msg->forward_input==0)
	                inForwardPID=false;
	        else
	                inForwardPID=true;


 	}

}



