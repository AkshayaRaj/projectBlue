#include <LEDS.h>

#define REDPIN 9 // 9
#define GREENPIN 8 //8
#define BLUEPIN 10 //10

#define red 1
#define orange 2
#define yellow 3
#define green 4
#define blue 5
#define indigo 6
#define violet 7
#define white 8
#define off 9


LEDS led(REDPIN,GREENPIN,BLUEPIN);

int r,g,b; 
void setup()
{
  Serial.begin(9600);
}


void loop()
{ 
  
Serial.println("red");
 led.colour(red);
 delay(5000); 
 Serial.println("orange");
 led.colour(orange);
 delay(5000);  
  Serial.println("yellow");
 led.colour(yellow);
 delay(5000); 
  Serial.println("green");
 led.colour(green);
 delay(5000); 
  Serial.println("blue");
 led.colour(blue);
 delay(5000); 
  Serial.println("indigo");
 led.colour(indigo);
 delay(5000); 
  Serial.println("violet");
 led.colour(violet);
 delay(5000); 
  Serial.println("white");
 led.colour(white);
 delay(5000); 

  Serial.println("off");
 led.colour(off);
 delay(5000); 
 
}
