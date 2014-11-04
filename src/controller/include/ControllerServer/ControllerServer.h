#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <srmauv_msgs/ControllerAction.h>

// Notes are included in ControllerServer.cpp

class ControllerServer{
public:
	ControllerServer(std::string name);
	void updateInfo(float forward,float sidemove,float forward_vel,float sidemove_vel,float heading,float depth);
	void executeCb(const srmauv_msgs::ControllerGoalConstPtr &goal);
	void setDispMode(bool isVelSide,bool isVelFwd);
	float getForward();
	float getSidemove();
	float getDepth();
	float getHeading();
	float getSidemoveVel();
	float getForwardVel();
	void setNavigation(bool nav);
	virtual ~ControllerServer();

private:
	double wrapAngle360(double error ,double heading);
	float _forward_input;
	float _heading_input;
	float _depth_input;
	float _sidemove_input;
	float _forward_setpoint;
	float _heading_setpoint;
	float _sidemove_setpoint;
	float _depth_setpoint;
	float _forward_vel_input;
	float _sidemove_vel_input;
	
	float MIN_FORWARD;
	float MIN_SIDEMOVE;
	float MIN_DEPTH;
	float MIN_HEADING;
	float MIN_SIDEMOVE_VEL;
	float MIN_FORWARD_VEL;
	
	bool _inNavigation;
	bool isFwdPos;
	bool isFwdVel;
	bool isSidePos;
	bool isSideVel;
	
	
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<srmauv_msgs::ControllerAction> as_;
	std::string action_name_;
	int data_count_;
	float sum_,sum_sq_;
	srmauv_msgs::ControllerGoal goal_;
	srmauv_msgs::ControllerFeedback feedback_;
	srmauv_msgs::ControllerResult result_;
};
		




#endif //CONTROLLER_SERVER_H
