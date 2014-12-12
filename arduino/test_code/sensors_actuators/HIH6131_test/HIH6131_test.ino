#include <Wire.h>

/**
Honeywell HumidIconTM
Digital
Humidity/Temperature
Sensors: HIH6130/6131
and HIH6120/6121 Series


Takes about 36.65 ms between measurement request and actual data to be read. Experimental test requires about 35ms delay minimal for fresh data to be read.
If sampled below that rate, firmware will output Data Stale.
*/
int address = 0x27; //Sensor address at 0x27

int reading = 0;
int humidity = 0;
int temp = 0;

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
  Serial.println("start sensing");
}


void loop()
{
    Wire.beginTransmission(address);
  Wire.write(byte(0x00));      
  Wire.endTransmission();      
  delay(36); 
  // step 4: request reading from sensor
  Wire.requestFrom(address, 4);    // request 4 bytes from slave device

  // step 5: receive reading from sensor
  if(4 <= Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    if(reading > 64) Serial.println("Data Stale");
    else
    {
      reading = reading << 8;    // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits
      humidity = float(reading)*100/16382;
      Serial.print(humidity);   // print the reading
      Serial.print(" ");
      
      reading = Wire.read();
      reading = reading << 8;
      reading |= Wire.read(); // receive low byte as lower 8 bits
      reading = reading >> 2;
      temp = float(reading)*165/16382 - 40 ;
      Serial.println(temp);
    }
  }

  delay(1000);                  // wait a bit since people have to read the output :)
}
