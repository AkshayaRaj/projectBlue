#include <TempAD7414.h>
#include <Wire.h>

TempAD7414 temp(96);

void setup()
{
  Serial.begin(9600);
  Serial.println("in Void setup");
  temp.initTempAD7414();
}

void loop()
{
  double tp = temp.getTemp();
  Serial.println(tp); 
  delay(1000);
}
