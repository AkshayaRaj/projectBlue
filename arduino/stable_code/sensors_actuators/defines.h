#ifndef _DEFINES_H
#define _DEFINES_H

#define ATM 99974 //Pascals or 14.5PSI
#define PSI100 689475
#define PSI30 206842
#define LPF_CONSTANT 0.7
#define PRESSURE_TYPE_ABSOLUTE_100 0
#define PRESSURE_TYPE_GAUGE_30 1
#define WaterPin1 A0 // green
#define WaterPin2 A1 // yellow
#define WaterPin3 A2 // orange
#define TempHumAddr 0x27 // V+
#define TempAddr2 0x4A	// SDA
#define TempAddr3 0x4B	// SCL
#define ADC_16 0x48 //
#define NORMAL 0
#define DEBUG_BB 1
#define LCD_ON 1
#define LCD_OFF 0

/*------------Motor Definitions  ----------------------------*/
//Forward Depth Thrusters
#define THRUSTER_3 3
#define THRUSTER_4 4
//Rear Depth Thrusters
#define THRUSTER_5 5
#define THRUSTER_6 6

//Sidemove Thrusters
#define THRUSTER_7 1
#define THRUSTER_8 2
#define RAY_1 11
#define RAY_2 12

/*------------Manipulators Definitions-------------------*/
#define MANI_1 22 //Dropper Left
#define MANI_2 23 //Dropper Right
#define MANI_3 24 //Torpedo Left
#define MANI_4 25 //Torpedo Right
#define MANI_5 26 // Grabber Actuator
#define MANI_6 27 // Linear Actuator
#define MANI_7 28 // Rotary Actuator

#endif // _DEFINES_H
