#include <srmauv_msgs/teleop_sedna.h>
#include <ros/ros.h>
#include<keyboard/Key.h>
#include <dynamic_reconfigure/server.h>
#include <srmauv_msgs/depth.h>
#include<sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>


srmauv_msgs::teleop_sedna teleop;
srmauv_msgs::depth depth;
keyboard::Key key;

int pressure;
double yaw,pitch,roll;

ros::Subscriber keyDown_sub;
ros::Subscriber keyUp_sub;
ros::Subscriber pressureSub;
ros::Subscriber imuSub;
ros::Subscriber teleopSetter;
ros::Subscriber headingSub;
ros::Publisher teleopPub;


void keyUp(const keyboard::KeyConstPtr& key);
void keyDown(const keyboard::KeyConstPtr& key);
void getPressure(const srmauv_msgs::depth &msg);
void getOrientation(const sensor_msgs::Imu::ConstPtr &msg);
void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg);
void setTeleop(const srmauv_msgs::teleop_sedna::ConstPtr& msg);
void setCurrent();
bool in_depth=false;
bool in_yaw=false;
bool in_roll=false;
bool in_pitch=false;

//ros::NodeHandle *nh;


int main(int argc,char** argv){
  ros::init(argc,argv,"teleop_sedna");
 ros::NodeHandle nh;


  teleop.enable=false;
  teleop.tune=false;
teleop.depth_enable=false;

  keyDown_sub=nh.subscribe("/keyboard/keydown",1000,keyDown);
  keyUp_sub=nh.subscribe("/keyboard/keyup",1000,keyUp);
  pressureSub=nh.subscribe("/pressure_data",1000,getPressure);
  imuSub=nh.subscribe("/imu/data",1000,getOrientation);
  headingSub=nh.subscribe("/imu/HeadingTrue_degree",1000,getHeading);
  teleopSetter=nh.subscribe("/teleop_set",100,setTeleop);

  teleopPub=nh.advertise<srmauv_msgs::teleop_sedna>("/teleop_sedna",1000);




 // teleop_sub=nh.subscribe("")
  while(ros::ok()){

    if(!teleop.enable){
    setCurrent();
    }

  //  ROS_INFO("Teleop: %d\tDepth: %d\tHeading: %f\tForward: %d\tReverse: %d\tStrafe: %d",
   //          teleop.enable,teleop.depth_setpoint,teleop.heading_setpoint,teleop.forward_speed,teleop.reverse_speed,teleop.sidemove_speed);
    teleopPub.publish(teleop);

    ros::spinOnce();

  }

	return 0;
}
void getPressure(const srmauv_msgs::depth &msg){
  depth.depth=msg.depth;

}


void getOrientation(const sensor_msgs::Imu::ConstPtr &msg){

}

void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg){
  yaw=msg->theta;
}

void setCurrent(){
  teleop.depth_setpoint=depth.depth;
        teleop.heading_setpoint=yaw;
        teleop.forward_speed=0;
        teleop.sidemove_speed=0;
        teleop.reverse_speed=0;
}

void setTeleop(const srmauv_msgs::teleop_sedna::ConstPtr& msg){
  teleop=*msg;
}

void keyDown(const keyboard::KeyConstPtr & key){
 // ROS_INFO("%d pressed",key->code);
  if(key->code==key->KEY_t){
    teleop.enable=!teleop.enable;
    ROS_INFO("Teleop :%d",teleop.enable);
    setCurrent();

  }


  if(teleop.enable){
   if(key->code==key->KEY_u){ //update setpoints to current input
    teleop.depth_setpoint=depth.depth;
    teleop.heading_setpoint=yaw;

    ROS_INFO("Setpoints Updated ! -> Depth: %d\tHeading :%d ",teleop.depth_setpoint,(int)teleop.heading_setpoint );

  }
  else if(key->code==key->KEY_MINUS){
    if(teleop.enable)
    teleop.depth_setpoint--;
    else
      teleop.depth_setpoint=depth.depth;
    ROS_INFO("Depth SP: %d",teleop.depth_setpoint);

  }

else if(key->code==100){ //d key for depth enable disable
	teleop.depth_enable=!teleop.depth_enable;
}


  else if(key->code==key->KEY_EQUALS){
    if(teleop.enable)
    teleop.depth_setpoint++;
    else
      teleop.depth_setpoint=depth.depth;

    ROS_INFO("Depth SP: %d",teleop.depth_setpoint);

  }
  else if(key->code==key->KEY_PAGEUP){
    teleop.torpedo=true;
    ROS_INFO("Dropper fired !");
  }
  else if(key->code==key->KEY_PAGEDOWN){
    teleop.dropper=true;
  }

  else if(key->code==key->KEY_LEFTBRACKET){
    if(teleop.enable)
      teleop.heading_setpoint--;
    else
      teleop.heading_setpoint=yaw;

    ROS_INFO("Heading SP: %d",teleop.heading_setpoint);
  }
  else if(key->code==key->KEY_RIGHTBRACKET){
    if(teleop.enable)
      teleop.heading_setpoint++;
    else
      teleop.heading_setpoint=yaw;

    ROS_INFO("Heading SP: %d",teleop.heading_setpoint);
  }

  else if(key->code==96){ //tune toggle
    teleop.tune=!teleop.tune;
  }

  else if(key->code==key->KEY_UP){
    if(teleop.enable){
    int forward_vel=150;
   //nh->param("/controller/teleop_forward_velocity",forward_vel,100);
   teleop.forward_speed=forward_vel;
    }
    else{
      teleop.forward_speed=0;
    }

  }
  else if(key->code==key->KEY_DOWN){
    if(teleop.enable){
        int forward_vel=150;
    //   nh.param("/controller/teleop_reverse_velocity",forward_vel,100);
       teleop.forward_speed=-forward_vel;
        }
        else{
          teleop.forward_speed=0;
        }
  }

  else if(key->code==key->KEY_LEFT){
    if(teleop.enable){
        int side_vel=120;
    //   nh.param("/controller/teleop_sidemove_velocity",side_vel,100);
       teleop.sidemove_speed=side_vel;
        }
        else{
          teleop.sidemove_speed=0;
        }
  }

  else if(key->code==key->KEY_RIGHT){
    if(teleop.enable){
        int side_vel=120;
   //    nh.param("/controller/teleop_sidemove_velocity",side_vel,100);
       teleop.sidemove_speed=-side_vel;
        }
        else{
          teleop.sidemove_speed=0;
        }
  }

  if(teleop.depth_setpoint<=0)
    teleop.depth_setpoint=0;
  else if(teleop.depth_setpoint>800)
    teleop.depth_setpoint=800;

  }

  if(teleop.heading_setpoint<-179)
    teleop.heading_setpoint=-179;
  if(teleop.heading_setpoint>179)
    teleop.heading_setpoint=179;


}

void keyUp(const keyboard::KeyConstPtr& key){
  //ROS_INFO("%d released",key->code);

   if(key->code==key->KEY_UP){
    if(teleop.enable){

   teleop.forward_speed=0;
    }


  }
  else if(key->code==key->KEY_DOWN){
    if(teleop.enable){

       teleop.forward_speed=0;
        }

  }

  else if(key->code==key->KEY_LEFT){
    if(teleop.enable){

       teleop.sidemove_speed=0;
        }

  }

  else if(key->code==key->KEY_RIGHT){
    if(teleop.enable){

       teleop.sidemove_speed=0;
    }

  }
  else if(key->code==key->KEY_PAGEUP){
     teleop.torpedo=false;
   }
   else if(key->code==key->KEY_PAGEDOWN){
     teleop.dropper=false;
   }


}
