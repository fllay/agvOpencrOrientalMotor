#include <stdint.h>
#include <Arduino.h>

#define M1ADR 0x01       //address of RS485 set to M1 driver
#define M2ADR 0x02       //address of RS485 set to M2 driver

#define  REGISTERREAD 0x03  //command to write to a register
#define  SINGLEWRITE  0x06  //command to write to a register
#define  BURSTWRITE   0x10  //command to write to multiple registers, define start address and length of reg/data

//register locations, define only the low byte since the high bytes are always neglected----------------
#define REG_RotationSpeed            0x0481 //16-bit low byte
#define REG_AccerelationTime         0x0601
#define REG_DecerelationTime         0x0681
#define REG_TorqueLimit              0x0701

#define REG_PresentAlarm             0x0081  
#define REG_PresentWarning           0x0097
#define REG_CommunicationError       0x0701

#define REG_DriveMotor               0x007D  //to start/stop a motor with commands below
//define at 0x007D [7..0]  x,x,stop,rev,fwd,m2,m1,m0 -------------------
#define CMD_Forward                0x0008 //forward rotation (may set acc time before use)
#define CMD_Backward               0x0030 //backward rotation
#define CMD_StopDec                0x0020 //stop with decerelate time
#define CMD_StopNow                0x0000 //stop immediately

#define maxPacketLength  128

class ModbusBLVmotor
{
  public:  
    // CONSTRUCTORS
    ModbusBLVmotor(); // Default pin selection.
       
    // PUBLIC METHODS
    void init();  
    void setM1Speed(int speed); // Set speed for M1. +/- forward/reverse
    void setM2Speed(int speed); // Set speed for M2. +/- forward/reverse
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    
    void setM1Brake(int brake); // Brake M1. 
    void setM2Brake(int brake); // Brake M2.
    void setBrakes(int m1Brake, int m2Brake); // Brake both M1 and M2.

    void setM1AccTime(unsigned int ms100); // Set accerelation time (x100ms) for M1
    void setM2AccTime(unsigned int ms100); // Set accerelation time (x100ms) for M2
    void setAccTime(unsigned int m1ms100, unsigned int m2ms100); // Set Acc time for both M1 and M2.

    void setM1DecTime(unsigned int ms100); // Set decerelation time (x100ms) for M1
    void setM2DecTime(unsigned int ms100); // Set decerelation time (x100ms) for M2
    void setDecTime(unsigned int m1ms100, unsigned int m2ms100); // Set dec time for both M1 and M2.

    unsigned char getM1Alarm(); // Get current reading for M1. 
    unsigned char getM2Alarm(); // Get current reading for M2.
    unsigned char getM1Warning(); // Get fault reading from M1.
    unsigned char getM2Warning(); // Get fault reading from M2.

    bool  sendModbusPacket(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data);
    bool  sendModbusPacketM1(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data);
    bool  sendModbusPacketM2(uint8_t dst, uint8_t type, uint16_t adr, uint16_t data);
    bool  sendModbusPacketBurst(uint8_t dst, uint8_t type, uint16_t startadr, uint8_t* data, uint8_t numdata);
    bool  getModbusPacket();
    uint16_t  crc16_big(uint8_t *data_p, uint8_t length); //big indian used in AGV

   //for pwm motor. they must be removed or mapped to blv motor-------- 
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1. 
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.

    
  private:
  
  String sPacket;
  uint8_t uiPacket [maxPacketLength]; //incoming packet buffer
  uint8_t uiPacketIdx=0;
  uint8_t uoPacket [maxPacketLength]; //outgoing packet buffer
  uint8_t uoPacketIdx=0;
  
  uint8_t CrcDatain[maxPacketLength]; //data manipulation for crc16 input
  
  boolean bCompletePacket = false;
  boolean bReceivePacket = false;
  boolean bEndPacket = false;
  boolean bFirstTime = true;  
  
  char cBuffer[40]; //bufferring for chars
  
  unsigned long ModbusPrevMillis = 0;
     
    
};


