#include <ros.h>
#include <ArduinoHardware.h>
#include <keyboard/Key.h>
#include <srmauv_msgs/thruster.h>
#include <Servo.h>
#include <srmauv_msgs/depth.h>
#include "defines.h"
#include <std_msgs/Bool.h>




static uint32_t currentTime,loopTime, fast_loop,time_elapsed, medium_loop, slow_loop;
int pressure;

ros::NodeHandle(nh);


Servo s3;
Servo s4;
Servo s5;
Servo s6;


srmauv_msgs::depth depth;
srmauv_msgs::thruster thruster;
std_msgs::Bool emergency;

void collectThruster(const srmauv_msgs::thruster &msg);

ros::Publisher depth_pub("/pressure_data",&depth);
ros::Publisher emergency_pub("/emergency",&emergency);
ros::Subscriber<srmauv_msgs::thruster>thruster_sub("/thruster_speed",collectThruster);



void setup(){
  delay(500);
  s3.attach(TH3);  
  s4.attach(TH4);
  s5.attach(TH5);
  s6.attach(TH6);
  initThrusters();
  pinMode(LCD,OUTPUT);
  digitalWrite(LCD,LOW);
  nh.initNode();
  enableThrusters();
 
  initPressure();
  initTopics();
  time_elapsed=0;
  currentTime=millis();
  loopTime=currentTime;

  
  
  
}


void loop(){
  
  currentTime=millis();
  
  if(currentTime>=(fast_loop+10))
  {
    readPressure();
    
    fast_loop=currentTime;
  }
  
  currentTime=millis();
  
  if(currentTime>=(medium_loop+50)){
    runThrusters();
    depth.depth=pressure;
    depth_pub.publish(&depth);
    
     nh.spinOnce();
    //assign pressure and publish it
    
    medium_loop=currentTime;
  }
  currentTime=millis();
  
  if(currentTime>=(slow_loop+333)){
   readWater();
   
   slow_loop=currentTime; 
  }
 
  
}



void initThrusters(){
 
  //active low seabotix relays:
  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
  pinMode(RELAY3,OUTPUT);
  
  digitalWrite(RELAY1,LOW);
  digitalWrite(RELAY2,LOW);
  digitalWrite(RELAY3,LOW);
  
  s3.write(1500);
    delay(50);
  s4.write(1500);
    delay(50);
  s5.write(1500);
    delay(50);
  s6.write(1500);
  delay(50);
  pinMode(TH1_F,OUTPUT);
  pinMode(TH2_F,OUTPUT);
  pinMode(TH7_F,OUTPUT);
  pinMode(TH8_F,OUTPUT);
  
  pinMode(TH1_R,OUTPUT);
  pinMode(TH2_R,OUTPUT);
  pinMode(TH7_R,OUTPUT);
  pinMode(TH8_R,OUTPUT);
  
  digitalWrite(TH1_F,1);
  digitalWrite(TH1_R,0);
  digitalWrite(TH2_F,1);
  digitalWrite(TH2_R,0);
  digitalWrite(TH7_F,1);
  digitalWrite(TH7_R,0);
  digitalWrite(TH8_F,1);
  digitalWrite(TH8_R,0);
  
  //delay(1000);
  
}




void runThrusters(){
  s3.write(1500+thruster.speed3);
 // delay(10);
  s4.write(1500+thruster.speed4);
 // delay(10);
  s5.write(1500+thruster.speed5);
 // delay(10);
  s6.write(1500+thruster.speed6);
//delay(10);
  
  if(thruster.speed1<0){
    digitalWrite(TH1_F,LOW);  
    digitalWrite(TH1_R,HIGH);
  }
  else if(thruster.speed1>0){
    digitalWrite(TH1_F,HIGH);  
    digitalWrite(TH1_R,LOW);
  }
  
  if(thruster.speed2<0){
    digitalWrite(TH2_F,LOW);  
    digitalWrite(TH2_R,HIGH);
  }
  else if(thruster.speed2>0){
    digitalWrite(TH2_F,HIGH);  
    digitalWrite(TH2_R,LOW);
  }
  
  if(thruster.speed7<0){
    digitalWrite(TH7_F,LOW);  
    digitalWrite(TH7_R,HIGH);
  }
  else if(thruster.speed7>0){
    digitalWrite(TH7_F,HIGH);  
    digitalWrite(TH7_R,LOW);
  }
  if(thruster.speed8>0){
    digitalWrite(TH8_F,LOW);  
    digitalWrite(TH8_R,HIGH);
  }
  else if(thruster.speed8<0){
    digitalWrite(TH8_F,HIGH);  
    digitalWrite(TH8_R,LOW);
  }  
    
    
    
    analogWrite(TH1,abs(thruster.speed1));
    analogWrite(TH2,abs(thruster.speed2));
    analogWrite(TH7,abs(thruster.speed7));
    analogWrite(TH8,abs(thruster.speed8));
    
  
  
  
}
void initPressure(){
  pressure=analogRead(PRESSURE_1);
  delay(1000);
}

void initTopics(){
  nh.advertise(depth_pub);
  nh.advertise(emergency_pub);
  nh.subscribe(thruster_sub);
    
}

void readPressure(){
  int new_pressure=analogRead(PRESSURE_2);
  pressure=pressure+ LPF_CONSTANT*(float)(new_pressure-pressure);
  depth.depth=pressure;
}

void readWater(){
 if(analogRead(LEAK_SENSOR)<WATER_LEAK_THRESH) {
   emergencyMode();
 }
}
void enableThrusters(){
  
}

void collectThruster(const srmauv_msgs::thruster &msg){
  thruster=msg;
}
  
 void emergencyMode(){
   emergency.data=true;
   while(true){
    emergency_pub.publish(&emergency);
   
    analogWrite(TH1,0);
    analogWrite(TH2,0);
    analogWrite(TH7,0);
    analogWrite(TH8,0);
    
    s3.write(SURFACE);
    s4.write(SURFACE);
    s5.write(SURFACE);
    s6.write(SURFACE);
    
   }
 
   
   
   
   
 }
  


