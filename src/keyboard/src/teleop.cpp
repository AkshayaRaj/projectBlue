#include <srmauv_msgs/teleop_sedna.h>
#include <ros/ros.h>
#include<keyboard/Key.h>
#include <dynamic_reconfigure/server.h>

srmauv_msgs::teleop_sedna teleop;
keyboard::Keyboard key;

ros::Subscriber keyDown_sub;
ros::Subscriber keyUp_sub;


void keyUp(const keyboard::Keyboard& key);
void keyDown(const keyboard::Keyboard* key);


int main(int argc,char** argv){
  ros::init(argc,argv,"teleop_sedna");
  ros::NodeHandle nh;

  keyDown_sub=nh.subscribe("/keyboard/keydown",1000,keyDown);
  keyUp_sub=nh.subscribe("/keyboard/keyup",1000,keyUp);



 // teleop_sub=nh.subscribe("")


	return 0;
}

