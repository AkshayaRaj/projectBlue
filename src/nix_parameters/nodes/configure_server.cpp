#include <ros/ros.h>

#include<dynamic_reconfigure/server.h>
#include <nix_parameters/parametersConfig.h>
#include <nix_msgs/thruster_ratio.h>
#include<nix_msgs/controller_constants.h>
#include<nix_msgs/controller_input.h>



double depth_offset;
double depth_setpoint;
double depth_setpoint_distance;


nix_msgs::thruster_ratio thruster_ratio;
nix_msgs::controller_constants controller_constants;
nix_msgs::controller_input controller_input;
//void ros::Publisher thruster_ratio_pub("thruster_ratio",&thruster_ratio);
ros::Publisher thruster_ratio_pub;
ros::Publisher controller_constants_pub;
ros::Publisher controller_input_pub;

void callback(nix_parameters::parametersConfig &config,uint32_t level){
	ROS_INFO("Reconfigure Request");
	thruster_ratio.thruster1_ratio=(float)config.thruster1_ratio;
	thruster_ratio.thruster2_ratio=(float)config.thruster2_ratio;
	thruster_ratio.thruster3_ratio=(float)config.thruster3_ratio;
	thruster_ratio.thruster4_ratio=(float)config.thruster4_ratio;
	thruster_ratio.thruster5_ratio=(float)config.thruster5_ratio;
	thruster_ratio.thruster6_ratio=(float)config.thruster6_ratio;
	controller_constants.depth_kp=(float)config.depth_kp;
	controller_constants.depth_kd=(float)config.depth_kd;
	controller_constants.depth_ki=(float)config.depth_ki;
	//controller_input.depth_input=(float)config.depth
	controller_input.depth_setpoint=(float)config.depth_setpoint;
	depth_offset=config.depth_offset;
	depth_setpoint_distance=config.depth_setpoint_distance;
	depth_setpoint=config.depth_setpoint;
	thruster_ratio_pub.publish(thruster_ratio);
	controller_input_pub.publish(controller_input);
	controller_constants_pub.publish(controller_constants);

}


int main(int argc,char **argv){
	ros::init(argc,argv,"configure_server");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);	
	 thruster_ratio_pub = nh.advertise<nix_msgs::thruster_ratio>("thruster_ratio", 1000);
	controller_constants_pub=nh.advertise<nix_msgs::controller_constants>("controller_constants",1000);
	controller_input_pub=nh.advertise<nix_msgs::controller_input>("controller_input",1000);
	dynamic_reconfigure::Server<nix_parameters::parametersConfig> server;
	dynamic_reconfigure::Server<nix_parameters::parametersConfig>::CallbackType f;

	f=boost::bind(&callback,_1,_2);
	server.setCallback(f);
	while(ros::ok()){
//	ROS_INFO("Spining node");
//	thruster_ratio_pub.publish(thruster_ratio);
	ros::spinOnce();
	}
	return 0;
}
