
#include <Servo.h>

Servo myservo;
int buttonState =0;

void setup()
{ 
    pinMode(7,OUTPUT);
    Serial.begin(9600);
    myservo.attach(9);
}

void loop()
{
  /*while(
  !Serial.available())
  {
    Serial.println("waiting for input");  
    delay(1000);
  }*/
  //buttonState = Serial.parseInt();
  buttonState=digitalRead(7);
  if(buttonState)
     myservo.write(0);
  else
     myservo.write(180);
  delay(15);
  /*
  myservo.write(0);
  delay(1000);
  myservo.write(180);
  delay(1000);
  */
}
