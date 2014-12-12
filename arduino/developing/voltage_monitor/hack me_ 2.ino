//Like the linux Kernel, do not **** with this code unless you know what you are doin'

//#include <std_msgs/String.h>
#include <Wire.h> 
#include "DHT.h"
//#include <ros.h>

// CONSTANT DEFINITIONS


// Global Variable declarations

String sinitializer = "";
#define DELAYFACTOR 1
float vread1;	//voltage reading 1
float vread2;	//voltage reading 2
float vread3;	//voltage reading 2
char buffer[6]; // for use with dtostrf
char buffer2[6]; // for use with dtostrf
float FACTOR1 = 5.7579;
float FACTOR2 = 5.6926;
#define DHTPIN 10
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature;	//in deg C
float humidity;	//in percent

// ROS STUFF

//ros::NodeHandle nh;
//std_msgs::String str_msg;
//ros::Publisher jellyduino("jellyduino", &str_msg);

// function declarations

void boot();

void lcd_clear_row(int);
void flicker_leds(int, int, int);
void initialize_leds(int, int);
void lcd_clear_row(int);
void set_state_leds(int, int);



// LED Class

class Leds
{
	bool state[11];
	bool init[11];
	public :
	void initled(int);
	bool get_state(int);
	void set_state(int, bool);
	void change_state(int);
};

void Leds::initled (int x){
	if(x <= 1){
			x--;
		}
	state[x] = false;
	init[x] = true;
	pinMode(x, OUTPUT);
	digitalWrite(x, false);
}

bool Leds::get_state (int x){
	if(x <= 1){
			x--;
		}
	return state[x];
}

void Leds::set_state (int x, bool dstate){
	if(init[x] == true){
		if(x <= 1){
			x--;
		}
		digitalWrite(x, dstate);
		state[x] = dstate;
	}
}

void Leds::change_state(int x){
	if(init[x] == true){
		if(x <= 1){
			x--;
		}
		state[x] = !state[x];
		digitalWrite(x, state[x]);
	}
}

// Global Objects Declarations

Leds leds;

void set_state_leds(int start, int end){

	//sets start to end in desired state
	for(int i = start; i <= end; i++){
		leds.set_state(i, true);
	}
	for(int i = end+1; i <= 10; i++){
		leds.set_state(i, false);
	}

}
void set_state_leds2(int start, int end, int value){

	//sets start to end in desired state
	if(start != value){
		for(int i = start; i <= value; i++){
			leds.set_state(i, true);
		}
		for(int i = value+1; i <= end; i++){
			leds.set_state(i, false);
		}
	}else{
		leds.set_state(value, true);
		for(int i = value+1; i <= end; i++){
			leds.set_state(i, false);
		}
	}

}


void led_percentage_handler(int pct1){

	int lightup = pct1/ 10;
	set_state_leds(2, lightup);

}

void led_percentage_handler(float reading1, float reading2){

	if (reading1 < 22.5 && reading2 < 22.5){
		flicker_leds(2,9,10);
	}else if (reading1 < 22.5){
		flicker_leds(2,5,10);
	}else if (reading2 < 22.5){
		flicker_leds(6,9,10);
	}

	int lightup1 = max(floor(reading1 - 22 ) + 2, 2);
	//Serial.println(lightup1);
	set_state_leds2(2, 5, lightup1);
	int lightup2 = max(floor(reading2 - 22 ) + 6, 6);
	//Serial.println(lightup2);
	set_state_leds2(6, 9, lightup2);

}

void initialize_leds(int start, int end){

	for(int i = start; i <= end; i++){
		leds.initled(i);
	}

	flicker_leds(start, end, 10);

	test_leds();

}


void initialize_serial(){

	Serial.begin(9600);
	Serial.println(sinitializer);
	Serial.println("BBAUV NOOB BOARD");

}

void read_dht(){
	humidity = dht.readHumidity();
	temperature = dht.readTemperature();
	if (isnan(temperature) || isnan(humidity)) {
		//error handling here
	} else {
		Serial.println(sinitializer + "Humidity: " + int(humidity) + " %%\t" + "Temperature: " + int(temperature) + " *C"); 
	  }
}

void read_voltage(){
	vread1 = analogRead(0) / 1024.0 * 5.0 * FACTOR1;
	vread2 = analogRead(1) / 1024.0 * 5.0 * FACTOR2;
	led_percentage_handler(vread1, vread2);
	dtostrf(vread1, 3, 2, buffer);
	dtostrf(vread2, 3, 2, buffer2);
	Serial.println(sinitializer + "Voltage 1: " + buffer + " Voltage 2: " + buffer2);
}



void setup()   /*----( SETUP: RUNS ONCE )----*/
{
	// LED init

	initialize_leds(1,10);

	// Serial Init

	initialize_serial();

	// ROS 

	//nh.initNode();
	//nh.advertise(jellyduino);

	// General Stuff




}



void loop(){

	read_voltage();
	read_dht();
	//nh.spinOnce();
	//delay(1000);

}






// Test Suites

void flicker_leds(int start, int end, int iter){
	for(int j = 0; j <= iter; j++){
		for(int i = start; i <= end; i++){
			leds.change_state(i);
		}
		delay(50 * DELAYFACTOR);
	}
}

void test_leds(){

	for (int i = 1 ; i <= 100; i++){
		led_percentage_handler(i);
		delay(5 * DELAYFACTOR);
	}

}