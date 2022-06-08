#include "WL_IMU.h"
union u_tag {
  byte b[4];
  float fval;
} u;
float IMUdata[98] = {0};
int iiii=0;
void IMU::Packet_Decode(uint8_t c)
{
  switch (st)
  {
    case 0: //Read 1st Byte
      if (c == 0x3a)
      {
        st = 1;
        Datain[read_count] = c;
        read_count += 1;
        //      Serial.print(read_count);
//          S/erial.print("3a");
      }
      break;
    case 1: //Read 2nd Byte
      if (c == 0x88) // original value: 0xc4, need another byte 0x01?
      {
        st = 2;
        Datain[read_count] = c;
        read_count += 1;
//        Serial.print(/"88");
      }
      else
      {
        st = 0;
        read_count = 0;
      }
      break;
    case 2: //Read 3nd Byte
      if (c == 0x01) // original value: 0xc4, need another byte 0x01?
      {
        st = 3;
        Datain[read_count] = c;
        read_count += 1;
//        Serial.print/("01");
      }
      else
      {
        st = 0;
        read_count = 0;
      }
      break;
    case 3:
      Datain[read_count] = c;
      tempA = c;
      read_count += 1;
      IMUDataProcessingFlag = 1;
      if (read_count >= 399) //original value: 49*4+6+1=203, new value = 98*4+6+1=399
      {
        st = 0;
        processData();
        read_count = 0;
      }
      break;
    default:
      st = 0;
      break;
  }
}

void IMU::INIT()
{
  SERIAL_WL.begin(460800);
  IMUInitializationFlag = 1;
}

void IMU::READ()
{

  // if (Serial.available()){
  //   serial1Flag = 1;
  // }
  if (SERIAL_WL.available())
  {
//      Serial.pri/nt("00");
    ch = SERIAL_WL.read();
    serial2Flag = 1;
    Packet_Decode(ch);
  }
}

void IMU::GetData()
{
  for (int i = 3; i < 394; i = i + 4) // original value: i<198, new value: 98*4+2(head)=398
  {
    u.b[0] = Datain[i];
    u.b[1] = Datain[i + 1];
    u.b[2] = Datain[i + 2];
    u.b[3] = Datain[i + 3];
    IMUdata[(i - 3) / 4] = u.fval;
  }
}

void IMU::processData()
{
  GetData();
  // Trunk
  TKAVx = IMUdata[1];
  TKAVy = IMUdata[2];
  TKAVz = IMUdata[3];
  TKEulerx = IMUdata[4];
  TKEulery = IMUdata[5];
  TKEulerz = IMUdata[6];
  TKLAx = IMUdata[7];
  TKLAy = IMUdata[8];
  TKLAz = IMUdata[9];
  // Right Thigh
  RTAVx = IMUdata[15];
  RTAVy = IMUdata[16];
  RTAVz = IMUdata[17];
  RTEulerx = IMUdata[18];
  RTEulery = IMUdata[19];
  RTEulerz = IMUdata[20];
  RTLAx = IMUdata[21];
  RTLAy = IMUdata[22];
  RTLAz = IMUdata[23];
  // Left Thigh
  LTAVx = IMUdata[29];
  LTAVy = IMUdata[30];
  LTAVz = IMUdata[31];
  LTEulerx = IMUdata[32];
  LTEulery = IMUdata[33];
  LTEulerz = IMUdata[34];
  LTLAx = IMUdata[35];
  LTLAy = IMUdata[36];
  LTLAz = IMUdata[37];
  // Right Shank
  RSAVx = IMUdata[43];
  RSAVy = IMUdata[44];
  RSAVz = IMUdata[45];
  RSEulerx = IMUdata[46];
  RSEulery = IMUdata[47];
  RSEulerz = IMUdata[48];
  RSLAx = IMUdata[49];
  RSLAy = IMUdata[50];
  RSLAz = IMUdata[51];
  // Left Shank
  LSAVx = IMUdata[57];
  LSAVy = IMUdata[58];
  LSAVz = IMUdata[59];
  LSEulerx = IMUdata[60];
  LSEulery = IMUdata[61];
  LSEulerz = IMUdata[62];
  LSLAx = IMUdata[63];
  LSLAy = IMUdata[64];
  LSLAz = IMUdata[65];
   // Right Heel
  RHAVx = IMUdata[71];
  RHAVy = IMUdata[72];
  RHAVz = IMUdata[73];
  RHEulerx = IMUdata[74];
  RHEulery = IMUdata[75];
  RHEulerz = IMUdata[76];
  RHLAx = IMUdata[77];
  RHLAy = IMUdata[78];
  RHLAz = IMUdata[79];

 // Left Heel
  LHAVx = IMUdata[85];
  LHAVy = IMUdata[86];
  LHAVz = IMUdata[87];
  LHEulerx = IMUdata[88];
  LHEulery = IMUdata[89];
  LHEulerz = IMUdata[90];
  LHLAx = IMUdata[91];
  LHLAy = IMUdata[92];
  LSLAz = IMUdata[93];
  
  Serial.print(TKAVx);
  Serial.print(",");
  Serial.print(TKAVy);
  Serial.print(",");
  Serial.print(TKAVz);
  Serial.print(",");
  Serial.print(TKLAx);
  Serial.print(",");
  Serial.print(TKLAy);
  Serial.print(",");
  Serial.print(TKLAz);
  Serial.print(",");
  Serial.print(TKEulerz);
  Serial.print(",");
  Serial.print(TKEulery);
  Serial.print(",");
  Serial.print(TKEulerx);
  Serial.print(",");
  Serial.print(RTAVx);
  Serial.print(",");
  Serial.print(RTAVy);
  Serial.print(",");
  Serial.print(RTAVz);
  Serial.print(",");
  Serial.print(RTLAx);
  Serial.print(",");
  Serial.print(RTLAy);
  Serial.print(",");
  Serial.print(RTLAz);
  Serial.print(",");
  Serial.print(RTEulerz);
  Serial.print(",");
  Serial.print(RTEulery);
  Serial.print(",");
  Serial.print(RTEulerx);
  Serial.print(",");
  Serial.print(LTAVx);
  Serial.print(",");
  Serial.print(LTAVy);
  Serial.print(",");
  Serial.print(LTAVz);
  Serial.print(",");
  Serial.print(LTLAx);
  Serial.print(",");
  Serial.print(LTLAy);
  Serial.print(",");
  Serial.print(LTLAz);
  Serial.print(",");
  Serial.print(LTEulerz);
  Serial.print(",");
  Serial.print(LTEulery);
  Serial.print(",");
  Serial.print(LTEulerx);
  Serial.print(",");
  Serial.print(RSAVx);
  Serial.print(",");
  Serial.print(RSAVy);
  Serial.print(",");
  Serial.print(RSAVz);
  Serial.print(",");
  Serial.print(RSLAx);
  Serial.print(",");
  Serial.print(RSLAy);
  Serial.print(",");
  Serial.print(RSLAz);
  Serial.print(",");
  Serial.print(RSEulerz);
  Serial.print(",");
  Serial.print(RSEulery);
  Serial.print(",");
  Serial.print(RSEulerx);
  Serial.print(",");
  Serial.print(LSAVx);
  Serial.print(",");
  Serial.print(LSAVy);
  Serial.print(",");
  Serial.print(LSAVz);
  Serial.print(",");
  Serial.print(LSLAx);
  Serial.print(",");
  Serial.print(LSLAy);
  Serial.print(",");
  Serial.print(LSLAz);
  Serial.print(",");
  Serial.print(LSEulerz);
  Serial.print(",");
  Serial.print(LSEulery);
  Serial.print(",");
  Serial.print(LSEulerx);
  Serial.print(",");
  Serial.print(RHAVx);
  Serial.print(",");
  Serial.print(RHAVy);
  Serial.print(",");
  Serial.print(RHAVz);
  Serial.print(",");
  Serial.print(RHLAx);
  Serial.print(",");
  Serial.print(RHLAy);
  Serial.print(",");
  Serial.print(RHLAz);
  Serial.print(",");
  Serial.print(RHEulerz);
  Serial.print(",");
  Serial.print(RHEulery);
  Serial.print(",");
  Serial.print(RHEulerx);
  Serial.print(",");
  Serial.print(LHAVx);
  Serial.print(",");
  Serial.print(LHAVy);
  Serial.print(",");
  Serial.print(LHAVz);
  Serial.print(",");
  Serial.print(LHLAx);
  Serial.print(",");
  Serial.print(LHLAy);
  Serial.print(",");
  Serial.print(LHLAz);
  Serial.print(",");
  Serial.print(LHEulerz);
  Serial.print(",");
  Serial.print(LHEulery);
  Serial.print(",");
  Serial.println(LHEulerx);
}
