#include <srmauv_msgs/teleop_sedna.h>
#include <ros/ros.h>
#include<keyboard/Key.h>

srmauv_msgs::teleop_sedna teleop;

ros::Subscriber teleop_sub;

int main(int argc,char** argv){
  ros::init(argc,argv,"teleop_sedna");
  ros::NodeHandle nh("~");


 // teleop_sub=nh.subscribe("")

	return 0;
}

