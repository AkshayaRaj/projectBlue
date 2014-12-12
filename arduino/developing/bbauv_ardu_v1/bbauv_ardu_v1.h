// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef bbauv_ardu_v1_H_
#define bbauv_ardu_v1_H_
#include "Arduino.h"
//add your includes for the project bbauv_ardu_v1 here

#include <thruster.h>   //thruster speed
#include <manipulator.h> //For servos control
#include <openups.h>    //battery capacity
#include <hull_status.h> //Temperature, Water Sensor
#include <std_msgs/Int16.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project bbauv_ardu_v1 here
int16_t readPressure();
void readPressureFilter();
float readTempSensor(int8_t addr);
void readTemperature();
void readWater();
void runThruster();
void getThrusterSpeed(const bbauv_msgs::thruster &msg);
void getManipulator(const bbauv_msgs::manipulator &msg);
void getBatteryReading(const bbauv_msgs::openups &msg);
//Do not add code below this line
#endif /* bbauv_ardu_v1_H_ */
