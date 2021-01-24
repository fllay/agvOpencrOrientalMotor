#include "ModbusBLVmotor.h"

int  OffsetSpeed = 0;
float  GainSpeed = 30;
int MaxRPM = 4000;

// Constructors ////////////////////////////////////////////////////////////////

ModbusBLVmotor::ModbusBLVmotor() //constructor
{
  //Pin map
  
}



// Public Methods //////////////////////////////////////////////////////////////
void ModbusBLVmotor::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  //pinMode(xx,OUTPUT);
  //pinMode(yy,INPUT);
  Serial2.begin(115200,SERIAL_8N1); 
  Serial4.begin(115200,SERIAL_8N1); 
   
}
// Set speed for motor 1, speed is a number betwenn -400 and 400


void ModbusBLVmotor::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  uint16_t speedRPM; //map to round per minute speed of the motor, 0 and 80-4000 rpm set at REG_RotationSpeed
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max speed of modbus round/minute
    speed = 400;

  if (speed !=0)   
     speedRPM = OffsetSpeed+(uint16_t)(speed*GainSpeed);// map 1-400pwm to 80-4000rpm
  else
     speedRPM = 0;
     
  if (speedRPM > MaxRPM)  // Max speed of modbus round/minute
    speedRPM = MaxRPM;

  //set rotation speed to register 
  sendModbusPacketM1(M1ADR, SINGLEWRITE, REG_RotationSpeed, (uint16_t)speedRPM);
  //Serial.print(">> M1, M2 (rpm)= "); Serial.print(speedRPM);
   
  if (reverse)
  {
    sendModbusPacketM1(M1ADR, SINGLEWRITE, REG_DriveMotor, CMD_Backward);
  }
  else
  {
    sendModbusPacketM1(M1ADR, SINGLEWRITE, REG_DriveMotor, CMD_Forward);
  }
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void ModbusBLVmotor::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  uint16_t speedRPM; //map to round per minute speed of the motor, 0 and 80-4000 rpm set at REG_RotationSpeed
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max speed of modbus round/minute
    speed = 400;

  if (speed !=0)   
     speedRPM = OffsetSpeed+(uint16_t)(speed*GainSpeed); // map 1-400pwm to 80-4000rpm
  else
     speedRPM = 0; 
     
  if (speedRPM > MaxRPM)  // Max speed of modbus round/minute
    speedRPM = MaxRPM;

  //set rotation speed to register 
  sendModbusPacketM2(M2ADR, SINGLEWRITE, REG_RotationSpeed, (uint16_t)speedRPM);
  //Serial.print(", "); Serial.println(speedRPM);
   
  if (reverse)
  {
    sendModbusPacketM2(M2ADR, SINGLEWRITE, REG_DriveMotor, CMD_Backward);
  }
  else
  {
    sendModbusPacketM2(M2ADR, SINGLEWRITE, REG_DriveMotor, CMD_Forward);
  }
}

// Set speed for motor 1 and 2
void ModbusBLVmotor::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Brake motor 1, brake is a number between 0 and 400 -- 0 = immediately or with decerelate time from 2-150 x100ms
void ModbusBLVmotor::setM1Brake(int brake) //with decerelate option
{
  uint16_t decTime=0;
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 400)  // Max brake
    brake = 400;
    
   //map 1-400 to 2-150 x100ms
   decTime = 150-(uint16_t )(brake*150/400);
   sendModbusPacket(M2ADR, SINGLEWRITE, REG_DecerelationTime, decTime);  
   Serial.print("Decerelation speed of M1 is set to "); Serial.println(decTime);

   if (brake ==0) //stop immedialtely
    sendModbusPacket(M1ADR, SINGLEWRITE, REG_DriveMotor, CMD_StopNow);
   else
    sendModbusPacket(M1ADR, SINGLEWRITE, REG_DriveMotor, CMD_StopDec);
   
}

// Brake motor 2, brake is a number between 0 and 400
void ModbusBLVmotor::setM2Brake(int brake)
{
    uint16_t decTime=0;
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 400)  // Max brake
    brake = 400;
    
   //map 1-400 to 2-150 x100ms
   decTime = 150-(uint16_t )(brake*150/400);
   sendModbusPacket(M2ADR, SINGLEWRITE, REG_DecerelationTime, decTime);  
   Serial.print("Decerelation speed of M2 is set to "); Serial.println(decTime);

   if (brake == 0) //stop immedialtely
    sendModbusPacket(M2ADR, SINGLEWRITE, REG_DriveMotor, CMD_StopNow);
   else
    sendModbusPacket(M2ADR, SINGLEWRITE, REG_DriveMotor, CMD_StopDec);
}

// Brake motor 1 and 2, brake is a number between 0 and 400
void ModbusBLVmotor::setBrakes(int m1Brake, int m2Brake)
{
  setM1Brake(m1Brake);
  setM2Brake(m2Brake);
}

// Read error status for motor 1 
unsigned char ModbusBLVmotor::getM1Alarm()
{
   sendModbusPacket(M1ADR, REGISTERREAD, REG_PresentAlarm, 0x01); //read from single address
   //wait and receive packet from Motor here
}

// Read error status for motor 2 
unsigned char ModbusBLVmotor::getM2Alarm()
{
  sendModbusPacket(M2ADR, REGISTERREAD, REG_PresentAlarm, 0x01);   
  //wait and receive packet from Motor here
}

// Read error status for motor 1 
unsigned char ModbusBLVmotor::getM1Warning()
{
   sendModbusPacket(M1ADR, REGISTERREAD, REG_PresentWarning, 0x01); //read from single address
   //wait and receive packet from Motor here
   
}

// Read error status for motor 2 
unsigned char ModbusBLVmotor::getM2Warning()
{
  sendModbusPacket(M2ADR, REGISTERREAD, REG_PresentWarning, 0x01); 
  //wait and receive packet from Motor here  
}




////////////// modbus packetizing M1////////////////////////////

bool ModbusBLVmotor::sendModbusPacketM1(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data)
{

  int i=0;
    
  uoPacket[0] = dst; //start packet 
  uoPacket[1] = type;  //0x06
  uoPacket[2] = adr>>8 ; //upper byte
  uoPacket[3] = adr;     //lower byte
  uoPacket[4] = data>>8; 
  uoPacket[5] = data; 
  
  uint16_t crc = crc16_big(uoPacket, 6);
  
  //Serial.print("\n>>crc16 outgoing packet = "); Serial.println(crc,HEX);
  
  uoPacket[6] = crc; //crc low byte
  uoPacket[7] = crc>>8; //crc high byte 
 /*
   Serial.print("Outgoing packet is ");
   for (i=0; i<8; i++)
   {
   Serial.print("0x"); Serial.print(uoPacket[i], HEX);   Serial.print(",");
   }
   Serial.print(" Length = "); Serial.println(i);
  */
  delay(20);
  //sending out the packet to 
  //if((millis()- ModbusPrevMillis) > 100) { //In pre-set it commences regulation with the calculated PID
  //  ModbusPrevMillis = millis();
  if (Serial2.write(&uoPacket[0], 8))
    return(true); //full length of this packet 
  else
    return(false); 
  //}
}



////////////// modbus packetizing M2////////////////////////////

bool ModbusBLVmotor::sendModbusPacketM2(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data)
{

  int i=0;
    
  uoPacket[0] = dst; //start packet 
  uoPacket[1] = type;  //0x06
  uoPacket[2] = adr>>8 ; //upper byte
  uoPacket[3] = adr;     //lower byte
  uoPacket[4] = data>>8; 
  uoPacket[5] = data; 
  
  uint16_t crc = crc16_big(uoPacket, 6);
  
  //Serial.print("\n>>crc16 outgoing packet = "); Serial.println(crc,HEX);
  
  uoPacket[6] = crc; //crc low byte
  uoPacket[7] = crc>>8; //crc high byte 
 /*
   Serial.print("Outgoing packet is ");
   for (i=0; i<8; i++)
   {
   Serial.print("0x"); Serial.print(uoPacket[i], HEX);   Serial.print(",");
   }
   Serial.print(" Length = "); Serial.println(i);
  */
  delay(20);
  //sending out the packet to 
  //if((millis()- ModbusPrevMillis) > 100) { //In pre-set it commences regulation with the calculated PID
  //  ModbusPrevMillis = millis();
  if (Serial4.write(&uoPacket[0], 8))
    return(true); //full length of this packet 
  else
    return(false); 
  //}
}



////////////// modbus packetizing////////////////////////////

bool ModbusBLVmotor::sendModbusPacket(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data)
{

  int i=0;
    
  uoPacket[0] = dst; //start packet 
  uoPacket[1] = type;  //0x06
  uoPacket[2] = adr>>8 ; //upper byte
  uoPacket[3] = adr;     //lower byte
  uoPacket[4] = data>>8; 
  uoPacket[5] = data; 
  
  uint16_t crc = crc16_big(uoPacket, 6);
  
  //Serial.print("\n>>crc16 outgoing packet = "); Serial.println(crc,HEX);
  
  uoPacket[6] = crc; //crc low byte
  uoPacket[7] = crc>>8; //crc high byte 
 /*
   Serial.print("Outgoing packet is ");
   for (i=0; i<8; i++)
   {
   Serial.print("0x"); Serial.print(uoPacket[i], HEX);   Serial.print(",");
   }
   Serial.print(" Length = "); Serial.println(i);
  */
  delay(20);
  //sending out the packet to 
  //if((millis()- ModbusPrevMillis) > 100) { //In pre-set it commences regulation with the calculated PID
  //  ModbusPrevMillis = millis();
  if (Serial2.write(&uoPacket[0], 8))
    return(true); //full length of this packet 
  else
    return(false); 
  //}
}

bool ModbusBLVmotor::sendModbusPacketBurst(uint8_t dst, uint8_t type, uint16_t startadr, uint8_t* data, uint8_t numdata)
{

  uint8_t numreg = numdata/2; //16bit register

  //packetizing
  uoPacket[0] = dst; //start packet 
  uoPacket[1] = type;  //0x10
  uoPacket[2] = startadr>>8 ; //upper byte
  uoPacket[3] = startadr;     //lower byte
  uoPacket[4] = numreg>>8;
  uoPacket[5] = numreg;
  uoPacket[6] = numdata; 
  
  int offset = 7;
  int i;
  for (i=0; i<numdata; i++)
  {
    uoPacket[offset+i] = data[i];    
  }
  
  uint16_t crc = crc16_big(uoPacket, numdata+offset);
  
  Serial.print(">>crc16 outgoing packet = "); Serial.println(crc,HEX);
  
  uoPacket[offset+numdata] = crc>>8; //crc high byte
  uoPacket[offset+numdata+1] = crc; //crc low byte 
 
   Serial.print("\nOutgoing packet is ");
   for (i=0; i< offset+numdata+2; i++)
   {
     Serial.print("0x"); Serial.print(uoPacket[i], HEX);   Serial.print(",");
   }
   Serial.print(" Length = "); Serial.println(i);
  
  //sending out the packet to FCC
  if (Serial2.write(uoPacket, offset+numdata+2))
    return(true); //full length of this packet 
  else
    return(false);
}

// read the incoming binary --------------------------------------
bool ModbusBLVmotor::getModbusPacket() 
{
 //uint8_t incomingUi = (uint8_t)(Serial3.read());
 //while(Serial3.available())
 {
  byte incomingUi =  Serial2.read();
  
  if ( incomingUi != 0xFF) //data is coming
  {
 
    if ((uiPacketIdx + 1) < maxPacketLength)
    {
        uiPacket[uiPacketIdx] = (uint8_t)incomingUi;
        
        Serial1.print("\nHEX");
        Serial1.println(uiPacket[uiPacketIdx],HEX);
        
        uiPacketIdx++;
     }
    bEndPacket = false;
    bReceivePacket = true;
  }
  else 
  {
    if (bReceivePacket) bEndPacket = true;
    bReceivePacket = false;
    
      
  }  
  
  return(bEndPacket);
 }
  
}

uint16_t ModbusBLVmotor::crc16_big(uint8_t *data_p, uint8_t length) //big indian used in AGV
{

  uint8_t i = 0, j = 0;
  uint16_t crc = 0xffff;

  for (i = 0; i<length; i++, data_p++)
  {
    crc = ((uint16_t)*data_p) ^ crc; //why invert?
    for (j = 0; j<8; j++)
    {
      if (crc & 0x0001) //if lsb bit is "1"
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc>>1;
    }
  }

  return (crc);

}

//////////////////////////////////////////////////////////////////////////

// Return motor 1 current value in milliamps.
unsigned int ModbusBLVmotor::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  return 0; //analogRead(_CS1) * 34;
}

// Return motor 2 current value in milliamps.
unsigned int ModbusBLVmotor::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  return 0; //analogRead(_CS2) * 34;
}

// Return error status for motor 1 
unsigned char ModbusBLVmotor::getM1Fault()
{
  return 0; //!digitalRead(_EN1DIAG1);
}

// Return error status for motor 2 
unsigned char ModbusBLVmotor::getM2Fault()
{
  return 0; //!digitalRead(_EN2DIAG2);
}
