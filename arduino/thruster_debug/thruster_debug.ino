#define LCD 44
#include<Servo.h>
#define TH1 4
#define TH2 3
#define TH3 8
#define TH4 9
#define TH5 7
#define TH6 6
#define TH7 2
#define TH8 5

#define REVERSE

#define TH1_F 30
#define TH1_R 33

#define TH2_F 26
#define TH2_R 29

#define TH7_F 22
#define TH7_R 25

#define TH8_F 34
#define TH8_R 37


Servo s3,s5,s6,s4;


void setup(){
  pinMode(LCD,OUTPUT);
  digitalWrite(LCD,LOW);
 s3.attach(TH3); 
//s4.attach(TH4);
//s5.attach(TH5);
//s6.attach(TH6); 

  s3.write(1500);
//  s4.write(1500);
//  s5.write(1500);
//  s6.write(1500);
pinMode(38,OUTPUT);
pinMode(39,OUTPUT);
pinMode(41,OUTPUT);

digitalWrite(38,LOW);
digitalWrite(39,LOW);
digitalWrite(41,LOW);


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
  digitalWrite(TH8_F,0);
  digitalWrite(TH8_R,1);
  
 
  /*
  digitalWrite(TH1_F,0);
  digitalWrite(TH1_R,1);
  digitalWrite(TH2_F,0);
  digitalWrite(TH2_R,1);
  digitalWrite(TH7_F,0);
  digitalWrite(TH7_R,1);
  digitalWrite(TH8_F,0);
  digitalWrite(TH8_R,1);
  */
  delay(1000);
  Serial.begin(9600);
  
  
}

void loop(){
  int val=1500;
 if(Serial.available()>0){
    val=Serial.parseInt();
   Serial.println(val);
    s3.writeMicroseconds(val);
     analogWrite(TH1,abs(val));
    analogWrite(TH2,abs(val));
    analogWrite(TH7,abs(val));
    analogWrite(TH8,abs(val));
 //   s4.writeMicroseconds(val);
 //   s5.writeMicroseconds(val);
 //   s6.writeMicroseconds(val);
    
   
 } 


//  s4.write(1600);
//s5.write(1600);
//s6.write(1600);
delay(500);
  
}
