#include <Servo.h>
#include <Thrusters.h>

//Selecting Pins for ESC
Thrusters thruster(12,11);

void setup(){
Serial.begin(9600);
}

void loop()
{
 //Init Motor
 thruster.mov(0);
 delay(5000);
 
 //Move motor full throttle
 thruster.mov(3200);
 
 //Hold max speed
 delay(5000);
 
 //Move to full reverse
 thruster.mov(-3200);
 //Hold max reverse
 delay(5000);
 
 //Stop Thrusters
 thruster.mov(0);
 delay(5000);
}

