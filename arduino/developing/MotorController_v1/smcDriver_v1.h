#ifndef _SMC_DRIVER_
#define _SMC_DRIVER_

#include <SoftwareSerial.h>

using namespace std;

class smcDriver{
private:
  SoftwareSerial smcSerial;
  unsigned char CRC7_POLY;
  unsigned char CRCTable[256];
  float _thrusterRatio[6];
public:
  smcDriver(int _rxPin, int _txPin);
  void init();
  void sendCommand(unsigned char message[], unsigned char length);
  void exitSafeStart(uint8_t id);
  void setMotorSpeed(uint8_t id, int speed);
  void setThrusterRatio(float ratio[]);
};

#endif
