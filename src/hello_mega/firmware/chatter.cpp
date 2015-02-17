#include <ros.h>
#include <std_msgs/String.h>

#include <Arduinio.h>

ros::Nodehandle nh;

std_msgs::String msg;

ros::Publisher chatter("chatter",&msg);

char hello[15]="Hello Mega";

void setup()
{
	nh.initNode();
	nh.advertise(chatter);
}

void loop()
{
	msg.data=hello;
	chatter.publish(&msg);
	nh.spinOnce();
	delay(1000);
}
	

