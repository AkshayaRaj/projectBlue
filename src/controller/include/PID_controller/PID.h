#ifndef PID_H_
#define PID_H_


#include <ros/ros.h>

namespace srmauv{

class sednaPID{
public:
	sednaPID(std::string,double,double,double,int);
	void setKp(double P);
	void setTi(double I);
	void setTd(double D);
	double getProportional();
	double getDerivative();
	double getIntegral();
	double getTotal();
	void setActuatorSatModel(int min,int max);
	double actuatorConstrain(double val);
	double computePID(double setpoint,double input);
	double wrapAngle360(double error,double heading);  //check inputs
	void clearIntegrator();
	double getActmin();
	double getActmax();
	virtual ~sednaPID();

private:
	double Kp;
	double Ti;
	double Td;
	double Imax;
	double N;	
	double actMax;
	double actMin;
	
	double _proportional;
	double _derivative;
	double _integral;
	double _total;
	ros::Time oldTime;
	double inputOld;
	std::string _name;
};
	

	
	
	
	
	
	

}


#endif // PID_H_
