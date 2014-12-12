#include <Wire.h>
int address = 0b1001010; //0x4A

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
  Wire.beginTransmission(address);
  Wire.write(byte(0x00));      // Continuous, 128SPS, gain of 1
  Wire.endTransmission();      // stop transmitting
  Serial.begin(9600);          // start serial communication at 9600bps
}

int reading = 0;

void loop()
{
  // step 4: request reading from sensor
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if(2 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.println(reading);   // print the reading
  }

  delay(100);                  // wait a bit since people have to read the output :)
}
