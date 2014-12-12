// Do not remove the include below
#include "ROS_Sensor.h"
#include <ros.h>
#include <env_data.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include "defines.h"
//Constants definition

#define PRESSURE_TYPE PRESSURE_TYPE_GAUGE_30
#define DEBUG_MODE DEBUG

//MACRO definitions ------------------------------------------------------
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
//ROS Definitions -------------------------------------------------------

ros::NodeHandle  nh;
bbauv_msgs::env_data env_msg;
ros::Publisher env_pub("env_data", &env_msg);

//ADC definitions -------------------------------------------------------
Adafruit_ADS1115 ads1115;
int16_t adc0, adc1;
//Water Sensor Definitions -------------------------------------------------------

//Pressure Sensor Definitions -------------------------------------------------------
static float depth;

//Temperature Sensor Definitions -------------------------------------------------------

float temp1 = 0;
float temp2 = 0;
float temp3 = 0;

int8_t i = 0;

//Timer Interrupt driven Kernel Timers -------------------------------------------------------

int32_t main_loop, one_sec_loop;

//Function Prototypes  -------------------------------------------------------
int32_t fmap(int32_t input, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
  return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Setup -------------------------------------------------------------
void setup() {

  nh.initNode();
  nh.advertise(env_pub);
  pinMode(waterLedPin, OUTPUT);
  pinMode(WaterPin1, INPUT); // Inner
  pinMode(WaterPin2, INPUT); // Middle
  pinMode(WaterPin3, INPUT); // Outer

  Wire.begin();

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
  // deactivate internal pull-ups for twi
  // as per note from atmega8 manual pg167
  cbi(PORTC, 4);
  cbi(PORTC, 5);
#else
  // deactivate internal pull-ups for twi
  // as per note from atmega128 manual pg204
  cbi(PORTD, 0);
  cbi(PORTD, 1);
#endif
  digitalWrite(SDA,0);
  digitalWrite(SCL,0);
#if DEBUG_MODE == DEBUG
  Serial.begin(9600);
  Serial.println("Debug Mode");
#endif
}

// Loop ---------------------------------------------------------------
void loop() {
  readTemperature();
  env_pub.publish(&env_msg);
  nh.spinOnce();
}


//Sensor Tasks Callbacks ---------------------------------------------------------
void readPressure()
{
	int32_t pressure;
#if PRESSURE_TYPE == PRESSURE_TYPE_GAUGE_30
	adc0 = ads1115.readADC_SingleEnded(0);
	pressure = fmap(adc0, 5340,26698,ATM,PSI30);
	depth = (float) pressure*100/(1000*9.81); //In centimetres
#elif PRESSURE_TYPE == PRESSURE_TYPE_ABSOLUTE_100
	adc1 = ads1115.readADC_SingleEnded(1);
	pressure = fmap(adc1, 3277,29491,0, PSI100);
	depth2 = (float) pressure2*100/(1000*9.81);
#endif

#if DEBUG_MODE == DEBUG
	Serial.println(pressure);
#endif
	env_msg.Depth = depth;
}

float readTempSensor(int8_t addr)
{
	int16_t reading = 0;
	int8_t store = 0;
	float temp = 0 ;
	Wire.beginTransmission(addr);
	Wire.requestFrom(addr,2);
	if(2 <= Wire.available())    // if two bytes were received
	{
	    reading = Wire.read();  // receive high byte (overwrites previous reading)
	    reading = reading << 4;    // shift high byte to be high 8 bits
	    store |= Wire.read() >> 4; // receive low byte as lower 8 bits
	    reading |= store;
	    temp = (float) reading*0.0625;
#if DEBUG_MODE == DEBUG
	    //reading >>= 4;
	    Serial.print(reading);
	    Serial.print(" ");
	    Serial.print(temp,4);   // print the reading
	    Serial.print(" ");
#endif
	    return reading;
	  } else return 0;
}

void readTemperature()
{
	temp1 = readTempSensor(TempAddr1);
	temp2 = readTempSensor(TempAddr2);
	temp3 = readTempSensor(TempAddr3);

	env_msg.Temp0 = temp1;
	env_msg.Temp1 = temp2;
	env_msg.Temp2 = temp3;
#if DEBUG_MODE == DEBUG
	    Serial.println();
#endif
}

void readWater()
{
	  env_msg.WaterDetA = 1-digitalRead(WaterPin1);
	  env_msg.WaterDetB = 1-digitalRead(WaterPin2);
	  env_msg.WaterDetC = 1-digitalRead(WaterPin3);
	//turn-on LED when water is detected
	if (env_msg.WaterDetA==1 || env_msg.WaterDetB==1 || env_msg.WaterDetC==1)
	    digitalWrite(waterLedPin,HIGH);
}

