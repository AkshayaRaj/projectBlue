#include <smcDriver_v2.h>
//Avoid using Software Serial
//using UART1 to daisy chain with 6 SMCs

long currentTime,loopTime;
smcDriver mDriver(&Serial1);
int mSpeed;
void setup()
{
 Serial1.begin(115200);     
 mDriver.init();
 currentTime=millis();
 loopTime=currentTime;
 pinMode(13,OUTPUT);
 mSpeed=0;
}

void loop()
{
  currentTime=millis();
  if( currentTime >= (loopTime + 40))
    {
      mSpeed=(mSpeed+100)%3200;
      digitalWrite(13,LOW);
      
      mDriver.setMotorSpeed(1,mSpeed);
      mDriver.setMotorSpeed(2,mSpeed);
      mDriver.setMotorSpeed(3,mSpeed);
      mDriver.setMotorSpeed(4,mSpeed);
      mDriver.setMotorSpeed(5,mSpeed);
      mDriver.setMotorSpeed(6,mSpeed);
      
      loopTime=currentTime; 
      digitalWrite(13,HIGH);  
    }
}