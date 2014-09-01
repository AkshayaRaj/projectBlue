/*
*NavUtils.cpp
*Created on May 22nd , 2014
*Author : Akshaya
*
*/


#include "NavUtils.h"

NavUtils::Navutils{
	//default constructor
}

NavUtils::quaternionToPitch(double q0,double q1,double q2,double q3){
	double p=a*sin(2*(q0*q2-q3*q1));
	p=p*180/M_PI;
	return p;
}

NavUtils::quaternionToRoll(double q0,double q1,double q2,double q3){
	double r=a*tan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	r=r*180/M_PI;
	return r;
}


NavUtils::quaternionToYaw(double q0,double q1,double q2,double q3){
	double y=a*tan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
	y=y*180/M_PI;
	return y;

}

NavUtils::~NavUtils(){
	}



