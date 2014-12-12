#include <SoftwareSerial.h>
#include "smcDriver.h"

#define rxPin 36 // Orange wire <-- receive from the 1st SMC Tx pin
#define txPin 37 // Red wire --> transmit to all SMCs Rx pin

smcDriver mDriver= smcDriver(rxPin,txPin);

void setup()
{
  mDriver.init();
}

int speed=0;
int dspeed=500;

void loop()
{
  mDriver.setMotorSpeed(1,speed);
  mDriver.setMotorSpeed(2,-speed);
  mDriver.setMotorSpeed(3, 1500);
  
  speed+=dspeed;
  delay(2000);
  if (speed>=2500 || speed <=-2500)
    dspeed=-dspeed;
}
