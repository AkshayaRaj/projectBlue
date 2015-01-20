#include <ros.h>
#include <ArduinoHardware.h>
#include <keyboard/Key.h>
#include <srmauv_msgs/depth.h>
#include "defines.h"
#include <std_msgs/Bool.h>
#include <Adafruit_NeoPixel.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, NEOPIXEL, NEO_GRB + NEO_KHZ800);

static uint32_t currentTime,loopTime, slow_loop;

ros::NodeHandle nh;




//Function Prototypes **************
void emergencyCallback(const std_msgs::Bool &msg);


// Publishers *************



//Subscribers *************
ros::Subscriber<std_msgs::Bool> emergency_sub("emergency",emergencyCallback);





void setup(){
  pinMode(TORPEDO,OUTPUT);
  pinMode(DROPPER,OUTPUT);
  digitalWrite(TORPEDO,LOW);
  digitalWrite(DROPPER,LOW);
  
  
  nh.initNode();
  initLights();
  
  initTopics();
}

void loop(){
       // currentTime=millis();
    //     if(currentTime>=(slow_loop+333)){
        nh.spinOnce();
        

 //       slow_loop=currentTime;
 // }


}

void initLights(){
   strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  //  colorWipeFull(strip.Color(0, 255, 0), 50); // Green
  blink(0,255,0,2);
  
  
    
}

void initTopics(){
        nh.subscribe(emergency_sub);

}

void blink(int r,int g,int b,int times){
  for(int i=0;i<times;i++){
  colorWipe(strip.Color(r, g, b), 50);
  delay(400);
  colorWipe(strip.Color(0, 0, 0), 50);
  }
  
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
     // delay(wait);
  }
}

void colorWipeFull(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}




void emergencyCallback(const std_msgs::Bool &msg){
        if(msg.data==true){
               while(true){
                   blink(255,0,0,2);
                }
                
     
}


