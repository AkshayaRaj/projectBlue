#include <SoftwareSerial.h>

#define rxPin 9 // black wire <-- receive from the 1st SMC Tx pin
#define txPin 8 // orange wire --> transmit to all SMCs Rx pin

SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);

//CRC calculation code
const unsigned char CRC7_POLY = 0x91;
unsigned char CRCTable[256];
 
unsigned char getCRCForByte(unsigned char val)
{
  unsigned char j;
 
  for (j = 0; j < 8; j++)
  {
    if (val & 1)
      val ^= CRC7_POLY;
    val >>= 1;
  }
 
  return val;
}
 
void buildCRCTable()
{
  int i;
 
  // fill an array with CRC values of all 256 possible bytes
  for (i = 0; i < 256; i++)
  {
    CRCTable[i] = getCRCForByte(i);
  }
}
 
unsigned char getCRC(unsigned char message[], unsigned char length)
{
  unsigned char i, crc = 0;
 
  for (i = 0; i < length; i++)
    crc = CRCTable[crc ^ message[i]];
  return crc;
}

//motorcontroller handling code!
//----------------------------------------------
void sendCommand(unsigned char message[], unsigned char length)
{
  unsigned char i=0;
  
  for(i=0;i<length;i++)
    smcSerial.write(message[i]);
}

// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart(uint8_t id)
{
  unsigned char message[4] = {0xAA,0x00,0x03,0x00};
  
  switch(id)
  {
    case 1: message[1]=0x01;
            break;
    case 2: message[1]=0x02;
            break;
    case 3: message[1]=0x03;
            break;
    case 4: message[1]=0x04;
            break;
    case 5: message[1]=0x05;
            break;
    case 6: message[1]=0x06;
            break;
  }
  message[3] = getCRC(message, 3);
  
  sendCommand(message,4);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(uint8_t id, int speed)
{
  unsigned char speedmsg[6]={0xAA,0x00,0x06,0x00,0x00,0x00};
  //by default: motor reverse (0x06)
  
  switch(id)
  {
    case 1: speedmsg[1]=0x01;
            break;
    case 2: speedmsg[1]=0x02;
            break;
    case 3: speedmsg[1]=0x03;
            break;
    case 4: speedmsg[1]=0x04;
            break;
    case 5: speedmsg[1]=0x05;
            break;
    case 6: speedmsg[1]=0x06;
            break;
  }
  
  if (speed < 0)
  {
    speed = -speed;  // make speed positive
  }
  else
  {
    speedmsg[2]=0x05; //change from reverse to forward
  }
  
  speedmsg[3]=speed&0x1F;
  speedmsg[4]=speed>>5;
  speedmsg[5]=getCRC(speedmsg,5);
  
  sendCommand(speedmsg,6);
}
 
void setup()
{
  // initialize software serial object with baud rate of 19.2 kbps
  smcSerial.begin(19200);
  buildCRCTable();
  delay(5);
  
  exitSafeStart(1);
  exitSafeStart(2);
  exitSafeStart(3);
  exitSafeStart(4);
  exitSafeStart(5);
  exitSafeStart(6);

}
 
void loop()
{
  setMotorSpeed(1,-1500);
  setMotorSpeed(2,-1500);
  setMotorSpeed(3,-1500);
  setMotorSpeed(4,-1500);
  setMotorSpeed(5,-1500);
  setMotorSpeed(6,-1500);

}

