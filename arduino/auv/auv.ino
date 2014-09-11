
#include <ros.h>
#include <PID_v1.h>
//#include <SoftwareSerial.h>
#include <ArduinoHardware.h>
#include <keyboard/Key.h>
#include <nix_msgs/controller_input.h>
#include <nix_msgs/controller_constants.h>
#include <nix_msgs/thruster_ratio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define RED 0
#define BLUE 36

#define T1_PWM 2
#define T2_PWM 3
#define T3_PWM 4
#define T4_PWM 5
#define T5_PWM 6
#define T6_PWM 7
#define T1_F 32
#define T1_R 26
#define T2_F 33
#define T2_R 27
#define T3_F 22
#define T3_R 28
#define T4_F 23
#define T4_R 29
#define T5_F 24
#define T5_R 30
#define T6_F 25
#define T6_R 31 
#define MOTOR_ON 34
#define FORWARD_RATIO 1
#define PRESSURE_SENSOR 0


long currentTime,loopTime,elapsedTime;

ros::NodeHandle nh;

//messages to publish:
std_msgs::Bool inTeleopMode;
std_msgs::Int32 depth;
nix_msgs::thruster_ratio ratio;
nix_msgs::controller_input feedback;


int timer=0;

//prototypes
void updateControllerInput(const nix_msgs::controller_input &msg);
void updateControllerConstants(const nix_msgs::controller_constants &msg);
void collectTeleop(const keyboard::Key &msg);
void setLED();
void keyUp(const keyboard::Key &msg);
void superImposePID();
void runThrusters();
void getDepthPIDUpdate();
void motorOn(const std_msgs::Bool &msg);
void collectThruster(const nix_msgs::thruster_ratio &msg);

void forward(int speed);
void reverse(int speed);
void yaw_left(int speed);
void yaw_right(int speed);
void heave(int speed);
int  cap_speed(int speed);
void pid_heave(int speed);


ros::Subscriber<nix_msgs::controller_input>pid_input_sub("controller_input",updateControllerInput);
ros::Subscriber<nix_msgs::controller_constants>pid_constants("controller_constants",updateControllerConstants);
ros::Subscriber<keyboard::Key>keydown_sub("/keyboard/keydown",collectTeleop);
ros::Subscriber<keyboard::Key>keyup_sub("/keyboard/keyup",keyUp);
ros::Subscriber<std_msgs::Bool>motor_on_sub("motor_on",motorOn);
ros::Subscriber<nix_msgs::thruster_ratio>thruster_ratio_sub("thruster_ratio",collectThruster);




ros::Publisher inTeleop_pub("inTeleop",&inTeleopMode);
ros::Publisher depth_pub("depth",&depth);
ros::Publisher feedback_pub("feedback",&feedback);




int forward_speed;
int reverse_speed;
int yaw_left_speed;
int yaw_right_speed;
int strafe_left_speed;
int strafe_right_speed;
int heave_speed;

bool inTeleop;
bool inPID;
//bool forward,reverse,yaw_left,yaw_right,strafe_left,strafe_right;

double depth_setpoint,depth_input,depth_output;
double heading_setpoint,heading_input,heading_output;

float depth_kp=1,depth_ki=0,depth_kd=0;
PID depthPID(&depth_input,&depth_output,&depth_setpoint,1,0,0,DIRECT);

void setup(){
	inTeleop=true;
	inPID=false;
	pinMode(BLUE,OUTPUT);
	pinMode(RED,OUTPUT);
	pinMode( T1_F,OUTPUT);
	pinMode( T1_R,OUTPUT);
	pinMode( T2_F,OUTPUT);
	pinMode( T2_R,OUTPUT);
	pinMode( T3_F,OUTPUT);
	pinMode( T3_R,OUTPUT);
	pinMode( T4_F,OUTPUT);
	pinMode( T4_R,OUTPUT);
	pinMode( T5_F,OUTPUT);
	pinMode( T5_R,OUTPUT);
	pinMode( T6_F,OUTPUT);
	pinMode( T6_R,OUTPUT);
        pinMode( MOTOR_ON,OUTPUT);
	digitalWrite(BLUE,HIGH);
        digitalWrite(MOTOR_ON,LOW);
	

	forward_speed=230;
	reverse_speed=150;
	yaw_left_speed=230;
	yaw_right_speed=230;
	heave_speed=255;
        depth_setpoint=162;

	//ROS Initialization
	nh.initNode();
	nh.subscribe(pid_input_sub);
	nh.subscribe(pid_constants);
	nh.subscribe(keydown_sub);
	nh.subscribe(keyup_sub);
        nh.subscribe(motor_on_sub);
	nh.advertise(inTeleop_pub);
        nh.advertise(depth_pub);

	//PID initialization
	depthPID.SetMode(AUTOMATIC);
	depthPID.SetSampleTime(20);
	depthPID.SetOutputLimits(-255,255); //this can be changed as needed
	//depthPID.SetControllerDirection(REVERSE);

	//Initialize timings
	elapsedTime=0;
	currentTime=millis();
	loopTime=currentTime;
	

}

void loop(){
	currentTime=millis();
        depth_input=analogRead(PRESSURE_SENSOR);
        depth_input=130;
        depth.data=depth_input;
	if(inPID){
	if(currentTime>=loopTime+40){
	
			getDepthPIDUpdate();
			runThrusters();
		}
	}
        depth_pub.publish(&depth);
        //inTeleop_pub.publish(&inTeleopMode);

	nh.spinOnce();
}

void runThrusters(){
	//later 
      pid_heave(depth_output);
  
}

void collectThruster(const nix_msgs::thruster_ratio &msg){
   ratio=msg; 
}

void collectTeleop(const keyboard::Key &msg){
	if(msg.code==msg.KEY_t){
		inTeleop=!inTeleop;
		inPID=!inPID;
          if(inTeleop)
            nh.loginfo("[In Teleop]");
          else
            nh.loginfo("[In PID_Mode]");
          
        
	}
	
	if(inTeleop==inPID){
		inTeleop=true;
		inPID=false;
	}

	if(inTeleop){
                if(msg.code==msg.KEY_UP){
                        forward(forward_speed);
                        nh.loginfo("Forward");
                }
                else if (msg.code==msg.KEY_DOWN){
                        reverse(reverse_speed);
                        nh.loginfo("Reverse");
                }
                else if (msg.code==msg.KEY_LEFT){
                        yaw_left(yaw_left_speed);
                        nh.loginfo("Yaw_left");
                }
                else if (msg.code==msg.KEY_RIGHT){
                        yaw_right(yaw_right_speed);
                        nh.loginfo("Yaw_right");
                }
                else if (msg.code==msg.KEY_SPACE){
                        heave(heave_speed);
                        nh.loginfo("Heave");
                }
       }

	setLED();	
		


	

}

void getDepthPIDUpdate(){
	depthPID.SetMode(inPID);
	if(inPID){
	depthPID.Compute();
        timer++;
        if(timer>1000){
        char buf[40];
        sprintf(buf,"Depth-> SP:%d IN:%d OUT:%d",(int)depth_setpoint,(int)depth_input,(int)depth_output);
        nh.logwarn(buf);
        timer=0;
        }
	}
	else{
	depth_output=0;
	}

       // feedback.depth_input=(float)depth;
       // feedback.depth_output=(float)depth_output;
       // feedback.depth_setpoint=(float)depth_setpoint;
}
	

void keyUp(const keyboard::Key &msg){
	if(inTeleop){
		if(msg.code==msg.KEY_UP)
			forward(0);
		else if(msg.code==msg.KEY_LEFT)
			yaw_left(0);
		else if(msg.code==msg.KEY_RIGHT)
			yaw_right(0);
		else if (msg.code==msg.KEY_DOWN)
			reverse(0);
		else if (msg.code==msg.KEY_SPACE)
			heave(0);
               else if (msg.code==msg.KEY_d){
                   char buf[10];
                   sprintf(buf,"Depth : %2.6f\n",depth_input);
                  // string messge=buf;
                   nh.loginfo(buf);
               }
	}
}

void forward(int speed){
	digitalWrite(T1_F,LOW);
	digitalWrite(T1_R,HIGH);
	digitalWrite(T2_F,LOW);
	digitalWrite(T2_R,HIGH);
	analogWrite(T1_PWM,(int)speed*ratio.thruster1_ratio);
	analogWrite(T2_PWM,(int)speed*ratio.thruster2_ratio);
}

void reverse(int speed){
	digitalWrite(T1_F,HIGH);
        digitalWrite(T1_R,LOW);
        digitalWrite(T2_F,HIGH);
        digitalWrite(T2_R,LOW);
        analogWrite(T1_PWM,(int)speed*ratio.thruster1_ratio);
        analogWrite(T2_PWM,(int)speed*ratio.thruster2_ratio);

}

void yaw_left(int speed){
	digitalWrite(T3_F,HIGH);
	digitalWrite(T3_R,LOW);
	digitalWrite(T4_F,HIGH);
	digitalWrite(T4_R,LOW);
	analogWrite(T3_PWM,(int)speed*ratio.thruster3_ratio);
	analogWrite(T4_PWM,(int)speed*ratio.thruster4_ratio);

}
void yaw_right(int speed){
	digitalWrite(T3_F,LOW);
	digitalWrite(T3_R,HIGH);
	digitalWrite(T4_F,LOW);
	digitalWrite(T4_R,HIGH);
	analogWrite(T3_PWM,(int)speed*ratio.thruster3_ratio);
	analogWrite(T4_PWM,(int)speed*ratio.thruster4_ratio);
}

void heave(int speed){
	digitalWrite(T5_F,LOW);
	digitalWrite(T5_R,HIGH);
	digitalWrite(T6_F,LOW);
	digitalWrite(T6_R,HIGH);
	analogWrite(T5_PWM,(int)speed*ratio.thruster5_ratio);
	analogWrite(T6_PWM,(int)speed*ratio.thruster6_ratio);
}

void pid_heave(int speed){
	if(speed>0){
		digitalWrite(T5_F,HIGH);
		digitalWrite(T5_R,LOW);
		digitalWrite(T6_F,HIGH);
		digitalWrite(T6_R,LOW);
		analogWrite(T5_PWM,(int)speed*ratio.thruster5_ratio);
		analogWrite(T6_PWM,(int)speed*ratio.thruster6_ratio);
		}
	else if(speed<0){
                speed=speed*(-1);
                digitalWrite(T5_F,LOW);
                digitalWrite(T5_R,HIGH);
                digitalWrite(T6_F,LOW);
                digitalWrite(T6_R,HIGH);
                analogWrite(T5_PWM,(int)speed*ratio.thruster5_ratio);
                analogWrite(T6_PWM,(int)speed*ratio.thruster6_ratio);
                }

}

void setLED(){
	if(inTeleop)
		digitalWrite(BLUE,HIGH);
	else
		digitalWrite(BLUE,LOW);
	/*if(inPID)
		digitalWrite(RED,HIGH);
	else
		digitalWrite(RED,LOW);
*/
}

void updateControllerInput(const nix_msgs::controller_input &msg){
	depth_input=msg.depth_input;
	depth_setpoint=msg.depth_setpoint;
        nh.loginfo("updated depth setpoint");
        nh.spinOnce();
	
}
void updateControllerConstants(const nix_msgs::controller_constants &msg){
	depthPID.SetTunings(msg.depth_kp,msg.depth_ki,msg.depth_kd);
        nh.loginfo("Updated PID constants");
                nh.spinOnce();
}

void motorOn(const std_msgs::Bool &msg){
    if(msg.data==true)
      digitalWrite(MOTOR_ON,HIGH);
    else
      digitalWrite(MOTOR_ON,LOW);
}


