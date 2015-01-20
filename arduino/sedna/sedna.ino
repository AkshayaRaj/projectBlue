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


Servo s3,s4,s5,s6;

srmauv_msgs::depth depth;
srmauv_msgs::thruster thruster;
std_msgs::Bool emergency;

void collectThruster(const srmauv_msgs::thruster &msg);

ros::Publisher depth_pub("/depth",&depth);
ros::Publisher emergency_pub("/emergency",&emergency);
ros::Subscriber<srmauv_msgs::thruster>thruster_sub("/thrusterSpeed",collectThruster);



void setup(){
  s3.attach(TH3);  
  s4.attach(TH4);
  s5.attach(TH5);
  s6.attach(TH6);
  pinMode(LCD,OUTPUT);
  digitalWrite(LCD,LOW);
  enableThrusters();
  initThrusters();
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
    
    //assign pressure and publish it
    nh.spinOnce();
    medium_loop=currentTime;
  }
  currentTime=millis();
  
  if(currentTime>=(slow_loop+333)){
   readWater();
   
   slow_loop=currentTime; 
  }
  
  
}


void initThrusters(){
 
  s3.write(1500);
  s4.write(1500);
  s5.write(1500);
  s6.write(1500);
  pinMode(TH1_REV,OUTPUT);
  pinMode(TH2_REV,OUTPUT);
  pinMode(TH7_REV,OUTPUT);
  pinMode(TH8_REV,OUTPUT);
  
  delay(1000);
  
}




void runThrusters(){
  s3.write(1500+thruster.speed3);
  s4.write(1500+thruster.speed4);
  s5.write(1500+thruster.speed5);
  s6.write(1500+thruster.speed6);
  
  if(thruster.speed1<0)
    digitalWrite(TH1_REV,HIGH);
  else
    digitalWrite(TH1_REV,LOW);
    
    if(thruster.speed2<0)
    digitalWrite(TH2_REV,HIGH);
  else
    digitalWrite(TH2_REV,LOW);
  
  if(thruster.speed7<0)
    digitalWrite(TH7_REV,HIGH);
  else
    digitalWrite(TH7_REV,LOW);
    
    if(thruster.speed7<0)
    digitalWrite(TH7_REV,HIGH);
  else
    digitalWrite(TH7_REV,LOW);
    
    analogWrite(TH1,abs(thruster.speed1));
    analogWrite(TH2,abs(thruster.speed2));
    analogWrite(TH3,abs(thruster.speed3));
    analogWrite(TH4,abs(thruster.speed4));
    
  
  
  
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
  int new_pressure=analogRead(PRESSURE_1);
  pressure=pressure+ LPF_CONSTANT*(float)(new_pressure-pressure);
}

void readWater(){
 if(analogRead(LEAK_SENSOR)>WATER_LEAK_THRESH) {
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
  


