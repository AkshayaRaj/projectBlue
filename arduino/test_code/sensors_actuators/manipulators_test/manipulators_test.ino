/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 22;
int led1 = 24;
int led2 = 26;
int led3 = 28;
int led4 = 30;
int led5 = 32;
int led6 = 34;
int led7 = 36;
int delay_val = 500;
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  pinMode(led2, OUTPUT);   
  pinMode(led3, OUTPUT);   
  pinMode(led4, OUTPUT);   
  pinMode(led5, OUTPUT);   
  pinMode(led6, OUTPUT);   
  pinMode(led7, OUTPUT);   
  pinMode(led1, OUTPUT);   
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("H");
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);               // wait for a second
  digitalWrite(led1, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led5, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led6, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  digitalWrite(led7, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_val);   
  Serial.println("L");
  digitalWrite(led, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);               // wait for a second
  digitalWrite(led1, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led2, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led3, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led4, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led5, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led6, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
  digitalWrite(led7, LOW);   // turn the LED on (LOW is the voltage level)
  delay(delay_val);   
}
