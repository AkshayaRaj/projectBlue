/*
*NavUtils.h
*Created on May 22nd , 2014
*Author : Akshaya
*
*/

#ifndef NAV_UTILS_H 
#define NAV_UTILS_H

#include <ros/ros.h>

class NavUtils{
public:
	NavUtils();
	double quaternionToPitch(double q0,double q1,double q2,double q3);
	double quaternionToYaw(double q0,double q1,double q2,double q3);
	double quaternionToRoll(double q0,double q1,double q2,double q3);
	
	virtual ~NavUtils();
};



#endif  // navutils.
