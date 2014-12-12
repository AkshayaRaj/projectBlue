 
int red_led   = 5;
int green_led = 6;
int blue_led  = 7;
int acc_ref_pin = A0;
int accXpin = A1;
int accYpin = A2;
int accZpin = A3;
unsigned long threshold = 17000;
int freeze_time = 2000;
//unsigned long stable_acc = 0;
int cyc_mode = 1; // start with cycle mode
int curLED = 0; // 0 for red, 1 for yellow, 2 for green

unsigned long accX = 0;
unsigned long accY = 0;
unsigned long accZ = 0;
unsigned long accX2 = 0;
unsigned long accY2 = 0;
unsigned long accZ2 = 0;
unsigned long cur_acc = 0;
unsigned long acc_ref = 0;

unsigned long loopTime = 0;
unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned long timeout = 60000;

void read_acc(){
  acc_ref = analogRead(acc_ref_pin)/2;
  accX = analogRead(accXpin);
  accY = analogRead(accYpin);
  accZ = analogRead(accZpin);
  accX2 = (accX - acc_ref)*(accX - acc_ref);
  accY2 = (accY - acc_ref)*(accY - acc_ref);
  accZ2 = (accZ - acc_ref)*(accZ - acc_ref);
}

void setup() {
  Serial.begin(9600);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);

  digitalWrite(red_led, LOW);
  digitalWrite(green_led, LOW);
  digitalWrite(blue_led, LOW); 

  read_acc();
  //stable_acc = accX2 + accY2 + accZ2; 

  currentTime = millis();
  loopTime = currentTime;
}

void cycLED(){
  if (curLED == 2){
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, LOW);
    digitalWrite(blue_led, LOW);
    curLED = 0;
  }
  else if (curLED == 0){
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, LOW);
    curLED = 1;
  }
  else{
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, LOW);
    curLED = 2;
  }
}

void loop() {
  read_acc();
  cur_acc = accX2 + accY2 + accZ2;
  //Serial.println(cur_acc);

  currentTime = millis();
  if (cyc_mode){
    if (currentTime - loopTime >= 5000){
      loopTime = currentTime;
      cycLED();
    }

    if (cur_acc > threshold){
      cyc_mode = 0;
      startTime = currentTime;
      delay(freeze_time);
      //digitalWrite(red_led, LOW);
      //digitalWrite(green_led, LOW);
      //digitalWrite(blue_led, HIGH);
    }
  }
  else{
    if (cur_acc > threshold){
      cycLED();
      startTime = currentTime;
      delay(freeze_time);
    }

    if (currentTime - startTime > timeout)
      cyc_mode = 1;
  }
}

