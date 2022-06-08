// #include <variant.h>
#include <Arduino.h>

#define PROFILE_SELECTOR (0)

// SERIAL_WL for wireless serial
// Yellow 3.3V    White RX    Red TX    Black GROUND
#define SERIAL_WL (Serial3)
#define INIT_TIME (4) // unit : second

typedef float CAL_TYPE;

class IMU
{
public:
  void INIT();
  void INIT_MEAN();
  void READ();
  void GetData();
  void Print();

  CAL_TYPE init_TKx=0;
  CAL_TYPE init_TKy=0;
  CAL_TYPE init_TKz=0;
  CAL_TYPE init_LTx=0;
  CAL_TYPE init_LTy=0;
  CAL_TYPE init_LTz=0;
  CAL_TYPE init_RTx=0;
  CAL_TYPE init_RTy=0;
  CAL_TYPE init_RTz=0;
  CAL_TYPE init_LSx=0;
  CAL_TYPE init_LSy=0;
  CAL_TYPE init_LSz=0;
  CAL_TYPE init_RSx=0;
  CAL_TYPE init_RSy=0;
  CAL_TYPE init_RSz=0;
  CAL_TYPE init_LFx=0;
  CAL_TYPE init_LFy=0;
  CAL_TYPE init_LFz=0;
  CAL_TYPE init_RFx=0;
  CAL_TYPE init_RFy=0;
  CAL_TYPE init_RFz=0;

  CAL_TYPE TKx=0;
  CAL_TYPE LTx=0;
  CAL_TYPE RTx=0;
  CAL_TYPE LSx=0;
  CAL_TYPE RSx=0;
  CAL_TYPE LFx=0;
  CAL_TYPE RFx=0;
  CAL_TYPE TKy=0;
  CAL_TYPE LTy=0;
  CAL_TYPE RTy=0;
  CAL_TYPE LSy=0;
  CAL_TYPE RSy=0;
  CAL_TYPE LFy=0;
  CAL_TYPE RFy=0;
  CAL_TYPE TKz=0;
  CAL_TYPE LTz=0;
  CAL_TYPE RTz=0;
  CAL_TYPE LSz=0;
  CAL_TYPE RSz=0;
  CAL_TYPE LFz=0;
  CAL_TYPE RFz=0;

  // CAL_TYPE TKAVx=0;
  // CAL_TYPE LTAVx=0;
  // CAL_TYPE RTAVx=0;
  // CAL_TYPE LSAVx=0;
  // CAL_TYPE RSAVx=0;
  // CAL_TYPE LFAVx=0;
  // CAL_TYPE RFAVx=0;
  // CAL_TYPE TKAVy=0;
  // CAL_TYPE LTAVy=0;
  // CAL_TYPE RTAVy=0;
  // CAL_TYPE LSAVy=0;
  // CAL_TYPE RSAVy=0;
  // CAL_TYPE LFAVy=0;
  // CAL_TYPE RFAVy=0;
  // CAL_TYPE TKAVz=0;
  // CAL_TYPE LTAVz=0;
  // CAL_TYPE RTAVz=0;
  // CAL_TYPE LSAVz=0;
  // CAL_TYPE RSAVz=0;
  // CAL_TYPE LFAVz=0;
  // CAL_TYPE RFAVz=0;
  // Trunk
  CAL_TYPE TKAVx=0;
  CAL_TYPE TKAVy=0;
  CAL_TYPE TKAVz=0;
  CAL_TYPE TKEulerx=0;
  CAL_TYPE TKEulery=0;
  CAL_TYPE TKEulerz=0;
  CAL_TYPE TKLAx=0;
  CAL_TYPE TKLAy=0;
  CAL_TYPE TKLAz=0;
  CAL_TYPE TKqA=0;
  CAL_TYPE TKqB=0;
  CAL_TYPE TKqC=0;
  CAL_TYPE TKqD=0;
  // Right thigh
  CAL_TYPE RTAVx=0;
  CAL_TYPE RTAVy=0;
  CAL_TYPE RTAVz=0;
  CAL_TYPE RTEulerx=0;
  CAL_TYPE RTEulery=0;
  CAL_TYPE RTEulerz=0;
  CAL_TYPE RTLAx=0;
  CAL_TYPE RTLAy=0;
  CAL_TYPE RTLAz=0;
  CAL_TYPE RTqA=0;
  CAL_TYPE RTqB=0;
  CAL_TYPE RTqC=0;
  CAL_TYPE RTqD=0;
  // Left thigh
  CAL_TYPE LTAVx=0;
  CAL_TYPE LTAVy=0;
  CAL_TYPE LTAVz=0;
  CAL_TYPE LTEulerx=0;
  CAL_TYPE LTEulery=0;
  CAL_TYPE LTEulerz=0;
  CAL_TYPE LTLAx=0;
  CAL_TYPE LTLAy=0;
  CAL_TYPE LTLAz=0;
  CAL_TYPE LTqA=0;
  CAL_TYPE LTqB=0;
  CAL_TYPE LTqC=0;
  CAL_TYPE LTqD=0;
    // Right shank
  CAL_TYPE RSAVx=0;
  CAL_TYPE RSAVy=0;
  CAL_TYPE RSAVz=0;
  CAL_TYPE RSEulerx=0;
  CAL_TYPE RSEulery=0;
  CAL_TYPE RSEulerz=0;
  CAL_TYPE RSLAx=0;
  CAL_TYPE RSLAy=0;
  CAL_TYPE RSLAz=0;
  CAL_TYPE RSqA=0;
  CAL_TYPE RSqB=0;
  CAL_TYPE RSqC=0;
  CAL_TYPE RSqD=0;
   // Left shank
  CAL_TYPE LSAVx=0;
  CAL_TYPE LSAVy=0;
  CAL_TYPE LSAVz=0;
  CAL_TYPE LSEulerx=0;
  CAL_TYPE LSEulery=0;
  CAL_TYPE LSEulerz=0;
  CAL_TYPE LSLAx=0;
  CAL_TYPE LSLAy=0;
  CAL_TYPE LSLAz=0;
  CAL_TYPE LSqA=0;
  CAL_TYPE LSqB=0;
  CAL_TYPE LSqC=0;
  CAL_TYPE LSqD=0;

   // Right Heel
  CAL_TYPE RHAVx=0;
  CAL_TYPE RHAVy=0;
  CAL_TYPE RHAVz=0;
  CAL_TYPE RHEulerx=0;
  CAL_TYPE RHEulery=0;
  CAL_TYPE RHEulerz=0;
  CAL_TYPE RHLAx=0;
  CAL_TYPE RHLAy=0;
  CAL_TYPE RHLAz=0;
  CAL_TYPE RHqA=0;
  CAL_TYPE RHqB=0;
  CAL_TYPE RHqC=0;
  CAL_TYPE RHqD=0;
   // Left Heel
  CAL_TYPE LHAVx=0;
  CAL_TYPE LHAVy=0;
  CAL_TYPE LHAVz=0;
  CAL_TYPE LHEulerx=0;
  CAL_TYPE LHEulery=0;
  CAL_TYPE LHEulerz=0;
  CAL_TYPE LHLAx=0;
  CAL_TYPE LHLAy=0;
  CAL_TYPE LHLAz=0;
  CAL_TYPE LHqA=0;
  CAL_TYPE LHqB=0;
  CAL_TYPE LHqC=0;
  CAL_TYPE LHqD=0;

  
  CAL_TYPE LKx=0; 
  CAL_TYPE RKx=0;
  CAL_TYPE RLKx=0;
  CAL_TYPE LKx_filtered_last=0;
  CAL_TYPE LKx_filtered=0;
  CAL_TYPE RKx_filtered_last=0;
  CAL_TYPE RKx_filtered=0;
  CAL_TYPE RLKx_filtered=0;
  CAL_TYPE DOTC[2];
  CAL_TYPE SquatTorque;
  
  CAL_TYPE y_delay[200]={0};
  CAL_TYPE RLKx_delay[200]={0};
   
  int delayindex=0;
  int delaypoint=50;
  int currentpoint=0;
  int doi=0;
  double Gain_E=1;
  double Gain_F=1;
  double test=0;
  double test1=0;
  double test2=0;
  int IMUInitializationFlag = 0;
  int IMUDataProcessingFlag = 0;
  int serial1Flag = 0;
  int serial2Flag = 0;
  float tempX = 0;
  float tempY = 0;
  float tempZ = 0;
  uint8_t tempA;

  double y_raw=0;
  double y_filtered=0;
  double y_filtered_last=0;

  void DelayOutputTorqueCommand();
  void processData();
  void SquatTorqueCommand();
  
private:

  int count1 = 0;
  uint8_t st = 0;
  uint8_t Datain[399];
  int read_count = 0;
  uint8_t ch;
  void Packet_Decode(uint8_t c);

};
