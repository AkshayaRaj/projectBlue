#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <srmauv_msgs/ControllerAction.h>

int main(int argc,char **argv)
{
 ros::init(argc,argv,"pid_client");

 actionlib::SimpleActionClient<srmauv_msgs::ControllerAction> ac("LocomotionServer",true);

 ROS_INFO("waiting for server to start");

 ac.waitForServer();

 ROS_INFO("Action server started, sending goal");

 srmauv_msgs::ControllerGoal goal;
 goal.depth_setpoint=200;
 goal.heading_setpoint=50;

 ac.sendGoal(goal);
 bool finished_before_timeout=ac.waitForResult(ros::Duration(20));

 if(finished_before_timeout){
   ROS_INFO("Set-points achieved !!");
   actionlib::SimpleClientGoalState state=ac.getState();
   ROS_INFO("Action finished : %s", state.toString().c_str());
 }
 else{
   ROS_INFO("The action server timed-out");
 }
return 0;


}

