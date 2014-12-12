// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef sensors_actuators_H_
#define sensors_actuators_H_
#include "Arduino.h"
//add your includes for the project bbauv_ardu_v1 here

#include <ros.h>
#include <smcDriver_v2.h> //Simple Moto Controller from Pololu Robotics and Electronics
//#include <Adafruit_ADS1015.h> //Display
//#include <Wire.h> //For I2C
//#include <Adafruit_CharacterOLED.h>

//Messages to communicate with ROS
#include <thruster.h>   //thruster speed
#include <manipulator.h> //For servos control
#include <hull_status.h> //Temperature, Water Sensor
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>

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
void pressure_init();
void humid_init();
void readPressureFilter();
void emergency_mode(int color);
float readTempSensor(int8_t addr);
void readTemperature();
void readWater();
void runThruster();
void lcd_display();
void topics_init();
void manipulators_init();
void lcd_init();
void leds_init();
void motors_init();
//void getThrusterSpeed(const bbauv_msgs::thruster &msg);
//void getManipulator(const bbauv_msgs::manipulator &msg);
//void getBatteryReading(const bbauv_msgs::openups &msg);
//Do not add code below this line
#endif /* bbauv_ardu_v1_H_ */
