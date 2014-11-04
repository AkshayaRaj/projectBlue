#include "PID.h"
#include <ros/ros.h>
#include <srmauv_msgs/thruster.h>
#include <srmauv_msgs/controller.h>
#include <srmauv_msgs/depth.h>
#include <srmauv_msgs/compass_data.h>
#include <srmauv_msgs/ControllerAction.h> //action
#include <srmauv_msgs/imu_data.h>
#include <srmauv_msgs/set_controller.h> //service
#include <srmauv_msgs/pid_info.h>
#include <dynamic_reconfigure/server.h>
#include <controller/controllerConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <PID_controller/PID.h>
#include <NavUtils/NavUtils.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <ControllerServer/ControllerServer.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>

//this will be calibrated from the sensor
const static int loop_freq=20;
const static int PSI30 = 206842;
const static int PSI100 = 689475;
const static int ATM = 99974;

double  thruster1_level = 1,
	thruster2_level = 1,
	thruster3_level= 1,
	thruster4_level = 1,
	thruster5_level =1,
	thruster6_level =1;

srmauv_msgs::controller ctrl ;
srmauv_msgs::depth depthValue;
srmauv_msgs::thruster thrusterSpeed;
srmauv_msgs::compass_data orientationAngles;
srmauv_msgs::pid_info pidInfo;
nav_msgs::Odometry odomData;
std_msgs::Int8 current__mode;

double depth_offset = 0;



void getOrientation(const srmauv_msgs::compass_data::ConstPtr& msg);
void getPressure(const srmauv_msgs::depth::ConstPtr& msg);
void getTeleop(const srmauv_msgs::thruster::ConstPtr& msg);
void callback(controller::controllerConfig &config,uint32 level);
double getHeadingPIDUpdate(); // some angle wrapping required 
float computeVelSideOffset();
float computeVelFwdOffset();
float interpolateDepth(float);
void setHorizontalThrustSpeed(double headingPID_output,double forwardPID_output,double sidemovePID_output);
void setVerticalThrustSpeed(double depthPID_output,double pitchPID_output,double rollPID_output;
double fmap (int input, int in_min, int in_max, int out_min, int out_max);

//State Machines : 
bool inTop,inTeleop,inHover=false,oldHover=false;
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
int loc_mode_forward[4] = {-2400,2400,-400,400};
int loc_mode_sidemove[4] = {-400,400,-2400,2400};
int loc_mode_heading[4] = {-400,400,-400,400};

int  manual_speed[6]={0,0,0,0,0,0};



//sets the correct actuator models 
bool locomotion_srv_handler(srmauv_msgs::locomotion_mode::Request &req, 
				srmauv_locomotion_mode::Response &res)
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
	current_mode.data=mode;
	locomotionModePub.publish(current_mode); //publish the mode we are in
	return true;
	
}

//the controller service call can enable/disable the various PID controllers
bool controller_srv_handler(srmauv_msgs::set_controller::Request &req){
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




int main (String args[]){
	ros::init(argc,argv,"controller");
	double forward_output,pitch_output,roll_ouput,heading_output,sidemove_output,depth_output;
	double forward_vel_output,sidemove_vel_output;

	ros::NodeHandle nh;
	
	thrusterPub=nh.advertise<srmauv_msgs::thruster>("/thruster_speed",1000);
	depthPub=nh.advertise<srmauv_msgs::depth>("/depth",1000);
	controllerPub=nh.advertise<srmauv_msgs::controller("/controller_targets",100);
	locomotionModePub=nh.advertise<std_msgs::Int8>("/locomotion_mode",100,true);
	pid_infoPub=nh.advertise<srmauv_msgs::pid_info>("/pid_info",1000);

	//subscribers: 

	//DVL here:	velocitySub=nh.subscribe("
	orientationSub=nh.subscribe("/euler",1000,collectionOrientation);
	pressureSub=nh.subscribe("/pressure_data",1000,collectPressure);
	teleopSub=nh.subscribe("/teleop_controller",1000,collectTeleop);
	//Dynamic reconfigure:
	dynamic_reconfigure::Server <controller::controllerConfig>server;
	dynamic_reconfigure::Server<controller::controllerConfig>::CallbackType f;
	f=boost::bind(&callback,_1,_2);
	server.setCallback(f);

	//initialize services
	
	ros::ServiceServer locomotion_service=nh.advertiseService("locomotion_mode_srv",locomotion_srv_handler);
	ROS_INFO("Change modes using locomotion_mode_srv");
	
	ros::ServiceServer service=nh.advertiseService("set_controller_srv",controller_srv_handler);
	ROS_INFO("Enable disable PID's using set_controller_srv");

	//Initialize the ActionServer:

	ControllerServer as("LocomotionServer");
	//PID Loop will run at 20Hz
	ros::Rate loop_rate(loop_frequency);
	
	//publish the initial standard mode locomotion mode message
	std_msgs::Int8 std_mode;
	std_mode.data=0;
	locomotionModePub.publish(std_mode);	
	ROS_INFO("PID controllers are ready .. !");
	
	while(ros::ok())
	{
		if(inHovermode && oldHoverMode!=inHoverMode)
		{
			
			
		

		}
	}

}





float interpolateDepth(float adcVal){
	//do depth interpolation here
	return adcval;
}
	






void collectPressure(const std_msgs::Int16 &msg){
	
	double depth=(double)interpolateDepth(msg.data);
	
	depthValue.pressure=msg->data;
	depthValue.depth=depth;
	ctrl.depth_input=depth;
}

void collectOrientation(const srmauv_msgs::compass_data &msg){

	ctrl.heading_input=msg->yaw;
	ctrl.pitch_input=msg->pitch;
	ctrl.roll_input=msg->roll;
}



double fmap(int input, int in_min, int in_max, int out_min, int out_max){
  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



































	













