#include <SoftwareSerial.h>

using namespace std;

class smcDriver{
private:
  SoftwareSerial smcSerial;
  unsigned char CRC7_POLY;
  unsigned char CRCTable[256];
public:
  smcDriver(int _rxPin, int _txPin);
  void init();
  
  unsigned char getCRCForByte(unsigned char val);
  void buildCRCTable();
  unsigned char getCRC(unsigned char message[], unsigned char length);
  void sendCommand(unsigned char message[], unsigned char length);
  void exitSafeStart(uint8_t id);
  void setMotorSpeed(uint8_t id, int speed);
};
