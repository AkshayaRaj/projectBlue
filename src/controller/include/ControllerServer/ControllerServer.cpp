/* ControllerServer.cpp
*  Created on: May 22nd, 2014
*  Author: Akshay Raj Dayal
*/

#include <ros/ros.h>
#include "ControllerServer.h"
#include <actionlib/server/simple_action_server.h>
#include <srmauv_msgs/ControllerAction.h>
#include <math.h>


/*Akshaya's Note ** 
#  Sway and Forward setpoints are RELATIVE to current position
   eg: give 5.00 to move 5 units in front of current location
#  Do a re-check for wrapAngle360, not sure if it would work
#  The rate of execution of the Server callback is 10 Hz, this can be changed if required .. 
#  Inputs are updated with updateInfo()

*/

//construtor 
ControllerServer::ControllerServer(std::string name):
as_(nh_,name,boost::bind(&ControllerServer::executeCb,this,_1),false),
action_name_(name) 
{
	
	_forward_input =0.0;
	_heading_input=0.0;
	_depth_input=0.0;
	_sidemove_input=0.0;
	
	MIN_FORWARD=0.05;
	MIN_SIDEMOVE=0.05;
	MIN_FORWARD_VEL=0.01;
	MIN_SIDEMOVE_VEL=0.01;
	MIN_HEADING=1;
	MIN_DEPTH=0.02;
	as_.start();
}

//execute callback:
void ControllerServer::executeCb(const srmauv_msgs::ControllerGoalConstPtr &goal){

	bool isForwardDone=false;
	bool isDepthDone=false;
	bool isSidemoveDone=false;
	bool isHeadingDone=false;
	
	bool success=true;
	double yaw_error=0;
	ros::Rate r(10);	

	//now modifying the local goal variable according to the goal setpoint message:
	//forward and sidemove setpoints are relative
	goal_.depth_setpoint=goal->depth_setpoint;
	goal_.heading_setpoint=goal->heading_setpoint; 
	goal_.forward_setpoint=goal->forward_setpoint+_forward_input;
	goal_.sidemove_setpoint=goal->sidemove_setpoint+_sidemove_input;
	goal_.forward_vel_setpoint=goal->forward_vel_setpoint;
	goal_.sidemove_vel_setpoint=goal->sidemove_vel_setpoint;


	
	ROS_INFO("Controller Action Server has just received a goal :  \n"
	    "F: %3.2f\tF_goal: %3.2f\n"
	    "S: %3.2f\tS_goal: %3.2f\n"
	    "D_goal: %3.2f\n"
	    "H_goal: %3.2f \n",
	    goal_.forward_setpoint,goal->forward_setpoint,
	    goal_.sidemove_setpoint,goal->sidemove_setpoint,
	    goal_.depth_setpoint,
	    goal_.heading_setpoint);

	ROS_INFO("Velocity goals are-> vf_g: %3.2f ,vs_g: %3.2f",goal_.forward_vel_setpoint,goal_.sidemove_vel_setpoint); 

	while( ros::ok() && success && !(isForwardDone && isDepthDone && isSidemoveDone && isHeadingDone) ){ //we can  disable isSidemoveDone when needed from here on a top level
				
		//check that preempt has not been requested 
		if(as_.isPreemptRequested() || !ros::ok())	// we need to do some "clean-up" ~ 
		{
			as_.setPreempted();
			//change the setpoints to the current input
			goal_.forward_setpoint=_forward_input;
			goal_.heading_setpoint=_heading_input;
			goal_.depth_setpoint=_depth_input;
			goal_.sidemove_setpoint=_sidemove_input;
			goal_.forward_vel_setpoint=0;
			goal_.sidemove_vel_setpoint=0;
			success=false;	
			
		}

		//calculating the yaw error using the wrap360 method: ** check configuration : Akshaya
		// I have redefined wrapAngle360 in this class -Akshaya
		yaw_error=fabs(goal_.heading_setpoint-wrapAngle360(goal_.heading_setpoint,_heading_input));
		

		if(fabs(goal_.forward_setpoint-_forward_input )<MIN_FORWARD && isFwdPos)
		{
			isForwardDone=true;
			ROS_DEBUG("Forward Setpoint Achieved");
		}
		
		if(fabs(goal_.sidemove_setpoint-_sidemove_input)<MIN_SIDEMOVE && isFwdVel)
		{
			isSidemoveDone=true;
			ROS_DEBUG("Sway Setpoint Achieved");
		}
	
		if(yaw_error<MIN_HEADING)
		{
			isHeadingDone=true;
			ROS_DEBUG("Heading Setpoint Achieved");
		}
		
		if(fabs(goal_.depth_setpoint-_depth_input)<MIN_DEPTH)
		{
			isDepthDone=true;
			ROS_DEBUG("Depth Setpoint Achieved");
		}

		//feedback is updated here: 
		feedback_.heading_error=yaw_error;
		if(isFwdPos)
			feedback_.forward_error=fabs(goal_.forward_setpoint-_forward_input) ;
		else if(isFwdVel)
			feedback_.forward_error=fabs(goal_.forward_vel_setpoint-_forward_vel_input);
		feedback_.sidemove_error=fabs(goal_.sidemove_setpoint-_sidemove_input);
		feedback_.depth_error=fabs(goal_.depth_setpoint-_depth_input);
		
		//now publishing the feedback
		as_.publishFeedback(feedback_);

		r.sleep(); //set to 10 Hz 

	} // exits if setpoints achieved ,or preempted or something went wrong , but if something went wrong then success=false 
	
	if (success)
	{
		//update the result
		result_.heading_final=_heading_input;
		result_.forward_final=_forward_input;
		result_.sidemove_final=_sidemove_input;
		result_.depth_final=_depth_input;
		
		ROS_INFO("%s : Succeeded ",action_name_.c_str());
		as_.setSucceeded(result_); 
	}
}// end of the callback method

		

	void ControllerServer::updateInfo(float forward,float sidemove,float forward_vel,float sidemove_vel,float heading,float depth) // this function will obtain inputs from the PID controller
	{
		_forward_input=forward;
		_sidemove_input=sidemove;
		_heading_input=heading;
		_depth_input=depth;
		_forward_vel_input=forward_vel;
		_sidemove_vel_input=sidemove_vel;
	}
	
	double ControllerServer::wrapAngle360(double setpoint,double heading)
	{
		double error=setpoint-heading;
		if(error>180)
			heading+=360;

		else if (error<-180)
			heading-=360;
		return fabs(heading);
	}


//****************Getters*******For Setpoints************
	float ControllerServer::getForward(){
		return goal_.forward_setpoint;
	}
	
	float ControllerServer::getSidemove(){
		return goal_.sidemove_setpoint;
	}
	
	float ControllerServer::getDepth(){
		return goal_.depth_setpoint;
	}
	
	float ControllerServer::getHeading(){
		return goal_.heading_setpoint;	
	}

	float ControllerServer::getSidemoveVel(){
		return goal_.sidemove_vel_setpoint;
	}
	
	float ControllerServer::getForwardVel(){
		return goal_.forward_vel_setpoint;
	}

void ControllerServer::setDispMode(bool isVelSide,bool isVelFwd)
{
  if(isVelSide)
  {
    isSideVel = true;
    isSidePos = false;
  }
  else
  {
    isSidePos = true;
    isSideVel = false;
  }

  if(isVelFwd)
  {
    isFwdVel = true;
    isFwdPos = false;
  }
  else
  {
    isFwdVel = false;
    isFwdPos = true;

  }
}

	ControllerServer::~ControllerServer()
	{
		_inNavigation=false;
		isFwdPos=false;
		isFwdVel=false;
		isSidePos=false;
		isSideVel=false;
	}
	
	

	













