#include <srmauv_msgs/teleop_sedna.h>
#include <ros/ros.h>
#include<keyboard/Key.h>
#include <dynamic_reconfigure/server.h>
#include <srmauv_msgs/depth.h>
#include<sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>


srmauv_msgs::teleop_sedna teleop;
srmauv_msgs::depth depth;
keyboard::Key key;

int pressure;
double yaw,pitch,roll;

ros::Subscriber keyDown_sub;
ros::Subscriber keyUp_sub;
ros::Subscriber pressureSub;
ros::Subscriber imuSub;

ros::Publisher teleopPub;

void keyUp(const keyboard::KeyConstPtr& key);
void keyDown(const keyboard::KeyConstPtr& key);
void getPressure(const std_msgs::Int16 &msg);
void getOrientation(const sensor_msgs::Imu::ConstPtr &msg);
bool in_depth=false;
bool in_yaw=false;
bool in_roll=false;
bool in_pitch=false;


int main(int argc,char** argv){
  ros::init(argc,argv,"teleop_sedna");
  ros::NodeHandle nh;
  teleop.enable=false;

  keyDown_sub=nh.subscribe("/keyboard/keydown",1000,keyDown);
  keyUp_sub=nh.subscribe("/keyboard/keyup",1000,keyUp);
  pressureSub=nh.subscribe("/pressure_data",1000,getPressure);
  imuSub=nh.subscribe("/imu/data",1000,getOrientation);

  teleopPub=nh.advertise<srmauv_msgs::teleop_sedna>("/teleop_sedna",1000);




 // teleop_sub=nh.subscribe("")
  while(ros::ok()){

    ROS_INFO("Teleop: %d\tDepth: %d\tHeading: %f\tForward: %d\tReverse: %d\tStrafe: %d",teleop.enable,teleop.depth_setpoint,teleop.heading_setpoint,teleop.forward_speed,teleop.reverse_speed,teleop.sidemove_speed);
    teleopPub.publish(teleop);

    ros::spinOnce();

  }

	return 0;
}
void getPressure(const std_msgs::Int16 &msg){
  depth.depth=msg.data;

}


void getOrientation(const sensor_msgs::Imu::ConstPtr &msg){




}

void keyDown(const keyboard::KeyConstPtr & key){
  ROS_INFO("%d pressed",key->code);
  if(key->code==key->KEY_t){
    teleop.enable=!teleop.enable;
  }
  else if(key->code==key->KEY_MINUS){
    if(teleop.enable)
    teleop.depth_setpoint--;
    else
      teleop.depth_setpoint=depth.depth;
  }

  else if(key->code==key->KEY_EQUALS){
    if(teleop.enable)
    teleop.depth_setpoint++;
    else
      teleop.depth_setpoint=depth.depth;

  }


  if(teleop.depth_setpoint<0)
    teleop.depth_setpoint=0;
  else if(teleop.depth_setpoint>500)
    teleop.depth_setpoint=500;


}

void keyUp(const keyboard::KeyConstPtr& key){
  ROS_INFO("%d released",key->code);
}
