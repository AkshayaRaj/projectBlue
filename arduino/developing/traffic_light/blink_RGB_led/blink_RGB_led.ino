
int red_led = 0;
int green_led = 1;
int blue_led = 4;

void setup() {                
  // initialize the digital pin as an output.
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);  
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, LOW);
  digitalWrite(blue_led, LOW);
  delay(8000);
  
  digitalWrite(red_led, LOW);
  digitalWrite(green_led, HIGH);
  digitalWrite(blue_led, LOW);
  delay(8000);
  
  digitalWrite(red_led, LOW);
  digitalWrite(green_led, LOW);
  digitalWrite(blue_led, HIGH);
  delay(8000);
}
