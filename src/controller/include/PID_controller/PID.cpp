#include "PID.h"
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>

#define ACTMIN -1000
#define ACTMAX 1000

namespace srmauv{

	void sednaPID::setKp(double P) {
		Kp=P;
	}

	void sednaPID::setTi(double I) {
		Ti=I;
	}

	void sednaPID::setTd(double D) {
		Td=D;
	}

	
        double sednaPID::getProportional() {
		return _proportional;
	}
        double sednaPID::getDerivative() {
		return _derivative;
	}
        double sednaPID::getIntegral() {
		return _integral;
	}

	double sednaPID::getTotal() {
		return _total;
	}

	void sednaPID::setActuatorSatModel(int min=ACTMIN,int max=ACTMAX) {
		actMax=max;
		actMin=min;
	}

	sednaPID::sednaPID(std::string name,double P,double I,double D,int Ncut) {
		_name=name;
		N=Ncut;
		Kp=P;
		Ti=I;
		Td=D;
		
		_integral=0;
		_derivative=0;

	}

	double sednaPID::getActmin(){
	  return actMin;

	}

	double sednaPID::getActmax(){
	  return actMax;
	}


	double  sednaPID::actuatorConstrain(double val){
		//this fn will return  val  provided
		//that  actMin<val<actMax else it will impose a restriction on the value returned
		
		double actuatorConstrain=0;
		if (val<actMin)
			actuatorConstrain=actMin;
		else if(val>actMax)
			actuatorConstrain=actMax;
		else 	
			actuatorConstrain=val;
		return actuatorConstrain;
	}

	double sednaPID::computePID(double setpoint,double input){
	 	ros::Time nowTime=ros::Time::now();
		double output;
		double dt=nowTime.nsec-oldTime.nsec;

		//double Tt=sqrt(Ti*Td);

		//oldTime.nsec>nowTime.nsec ? dt=(nowTime.nsec+1000000000-oldTime.nsec)/1000000 :
		//			dt=nowTime.nsec/1000000;

		_proportional=Kp*(setpoint-input);

		//the following is setpoint weighting and bandwidth limitation for derivative:
//		_derivative=(Td/(Td+N*dt))*(_derivative-Kp*N*(input-inputOld));
		_derivative=0;
			
		_total=_proportional+_integral;

		//apply constrains to output: 
		output=actuatorConstrain(_total);
		 ROS_DEBUG("n: %s P: %2.f, I: %2.f, D: %2.f, dt: %2.2f, err: %6.2f",_name.c_str(),_proportional,_integral, _derivative,dt,setpoint - input);

		
		//Integral with wind-up protection: 
		if(Ti!=0){
//		  _integral+=(Kp*dt*(setpoint-input))/Ti+ (output-_total)*dt/Tt ;
		  _integral+=(Kp*(setpoint-input))/Ti;
}
		else
				_integral=0; //incase Ti is reconfgured
		if (_integral>actMax)
		  _integral=actMax;
		else if(_integral<actMin){
		  _integral=actMin;
		}

		oldTime=nowTime;
		inputOld=input;
		return output;
		}
		
		
	double sednaPID::wrapAngle360(double error, double heading){
		if(error>180)
			heading+=360;
		else if (error<-180)
			heading-=360;
		return heading;
	}

	void sednaPID::clearIntegrator(){
		_integral=0;
	}

	sednaPID::~sednaPID(){

		}







		 
		
		



	







 
				




	
		
		
		
		
	


	
		











}
