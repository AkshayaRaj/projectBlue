#include <ros.h>
#include <PID_v1.h>
//#include <SoftwareSerial.h>
#include <ArduinoHardware.h>
#define RED 1
#define BLUE 2

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

#define FORWARD_RATIO 1



long currentTime,loopTime,elapsedTime;

ros::NodeHandle nh;

//messages to publish:
std_msgs::Bool inTeleop;


//prototypes
void updateControllerInput(nix_msgs::controller_input &msg);
void updateControllerConstants(nix_msgs::controller_constants &msg);
void collectTeleop(myKeyboard::key &msg);
void setLED();
void keyUp(myKeyboard::key &msg);
void superImposePID();
void runThrusters();
void getDepthPIDUpdate();

void forward(int speed);
void reverse(int speed);
void yaw_left(int speed);
void yaw_right(int speed);
void heave(int speed);
int  cap_speed(int speed);
void pid_heave(int speed);

ros::Subscriber<nix_msgs::controller_input>pid_input_sub("controller_input",&updateControllerInput);
ros::Subscriber<nix_msgs::controller_constants>pid_constants("controller_constants",&updateControllerConstants);
ros::Subscriber<myKeyboard::key>keydown_sub("/keyboard/keydown",&collectTeleop);
ros::Subscriber<muKeyboard::key>keyup_sub("/keyboard/keyup",&keyUp);


ros::Publisher inTeleop_pub("inTeleop",&inTeleop);



int forward_speed;
int reverse_speed;
int yaw_left_speed;
int yaw_right_speed;
int strafe_left_speed;
int strafe_right_speed;
int heave_speed;

bool inTeleopMode;
bool inPIDMode;
bool forward,reverse,yaw_left,yaw_right,strafe_left,strafe_right;

double depth_setpoint,depth_input,depth_output;
double heading_setpoint,heading_input,heading_output;

PID depth_PID(&depth_input,&depth_output,&depth_setpoint,1,0,0,DIRECT);

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
	digitalWrite(BLUE,HIGH);
	

	forward_speed=100;
	reverse_speed=100;
	yaw_left_speed=80;
	yaw_right_speed=80;
	heave_speed=230;

	//ROS Initialization
	nh.initNode();
	nh.subscribe(pid_input_sub);
	nh.subscribe(pid_constants);
	nh.subscribe(keydown_sub);
	nh.subscribe(keyup_sub);
	nh.advertise(inTeleop_pub);

	//PID initialization
	depthPID.SetMode(AUTOMATIC);
	depthPID.SetSampleTime(20);
	depthPID.SetOutputLimits(-255,255); //this can be changed as needed
	depthPID.SetControllerDirection(REVERSE);

	//Initialize timings
	elapsedTime=0;
	currentTime=millis();
	loopTime=currentTime();
	

}

void loop(){
	currentTime=millis();
	if(inPID){
	if(currentTime>=loopTime+40){
	
			getDepthPIDUpdate();
			runThrusters();
		}
	}

	nh.spinOnce();
}

void runThrusters(){
	
	


}

void collectTeleop(myKeyboard::key &msg){
	if(msg.code==msg.KEY_t){
		inTeleop=!inTeleop;
		inPID=!inPID;
	}
	
	if(inTeleop==inPID){
		inTeleop=true;
		inPID=false;
	}

	if(inTeleop){
                if(msg.key==msg.KEY_UP)
                        forward(forward_speed);
                else if (msg.key==msg.KEY_DOWN)
                        reverse(reverse_speed);
                else if (msg.key==msg.KEY_LEFT)
                        yaw_left(yaw_left_speed);
                else if (msg.key==msg.KEY_RIGHT)
                        yaw_right(yaw_right_speed);
                else if (msg.key==KEY_SPACE)
                        heave(heave_speed);
       }

	setLED();	
		


	

}

void getDepthPIDUpdate(){
	depthPID.SetMode(inPID);
	if(inPID){
	depthPID.compute();
	}
	else{
	depth_output=0;
	}
}
	

void keyUp(myKeyboard::key &msg){
	if(inTeleop){
		if(msg.key==msg.KEY_UP)
			forward(0);
		else if(msg.key==KEY_LEFT)
			left(0);
		else if(msg.key==KEY_RIGHT)
			right(0);
		else if (msg.key==KEY_DOWN)
			reverse(0);
		else if (msg.key==KEY_SPACE)
			heave(0);
	}
}

void forward(int speed){
	digitalWrite(T1_F,HIGH);
	digitalWrite(T1_R,LOW);
	digitalWrite(T2_F,HIGH);
	digitalWrite(T2_R,LOW);
	analogWrite(T1_PWM,speed);
	analogWrite(T2_PWM,speed);
}

void reverse(){
	digitalWrite(T1_F,LOW);
        digitalWrite(T1_R,HIGH);
        digitalWrite(T2_F,LOW);
        digitalWrite(T2_R,HIGH);
        analogWrite(T1_PWM,speed);
        analogWrite(T2_PWM,speed);

}

void yaw_left(int speed)
	digitalWrite(T1_F,

)
void yaw_right(int speed){
}

void heave(int speed){
	digitalWrite(T5_F,HIGH);
	digitalWrite(T5_R,LOW);
	digitalWrite(T6_F,HIGH);
	digitalWrite(T6_R,LOW);
	analogwrite(T5_PWM,speed);
	analogWrite(T6_PWM,speed);
}

void pid_heave(int speed){
	if(speed>0){
		digitalWrite(T5_F,HIGH)
		digitalWrite(T5_R,LOW);
		digitalWrite(T6_F,HIGH);
		digitalWrite(T6_R,LOW);
		analogWrite(T5_PWM,depth_output);
		analogWrite(T6_PWM,depth_output);
		}
	 if(speed<0){
                digitalWrite(T5_F,LOW)
                digitalWrite(T5_R,HIGH);
                digitalWrite(T6_F,LOW);
                digitalWrite(T6_R,HIGH);
                analogWrite(T5_PWM,depth_output);
                analogWrite(T6_PWM,depth_output);
                }

}

void setLED(){
	if(inTeleop)
		digitalWrite(BLUE,HIGH);
	else
		digitalWrite(BLUE,LOW);
	if(inPID)
		digitalWrite(RED,HIGH)
	else
		digitalWrite(RED,LOW);
}
