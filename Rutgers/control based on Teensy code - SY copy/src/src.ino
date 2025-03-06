//for right knee exo, positive current assist extension, and torque sensor measures positive torque
#include "WL_IMU.h"
#include <Arduino.h>
IMU imu;                                //Create IMU object see WL_IMU.h
double Fsample = 500;                   // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;               // [Hz] Bluetooth sending data frequency
double Fsample_insole = 200;            // [Hz] insole reading data frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000/Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000/Fsample_ble);       // used to control the Bluetooth communication frequency
unsigned long Tinterval_insole_microsecond = (unsigned long)(1000000/Fsample_insole); // used to control the insole reading data frequency
unsigned long current_time = 0;         
unsigned long previous_time = 0;        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;    // used to control the Bluetooth communication frequency

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);
  imu.INIT();
  delay(1000);
  IMUCurrentControlSetup();  //initialize current control and IMU control. this is use for direct current control without torque PID loop (torque reference/torque constant= current reference -> current PID loop)

}

void loop() {
  IMUCurrentControl();         //if you want to try torque control, please uncommment IMUCurrentControl(); and use this function.
}

void IMUCurrentControlSetup()
{
  imu.INIT();    //Initialize IMU;
  delay(1000);
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
}

//void testIMU(){
//  imu.READ();
//  // Serial.print(imu.IMUInitializationFlag);
//  // Serial.print(" ");
//  // Serial.print(imu.serial1Flag);
//  // Serial.print(" ");
//  // Serial.print(imu.serial2Flag);
//  // Serial.print(" ");
//  // Serial.println(imu.IMUDataProcessingFlag);
//  // if (imu.TKx > 0){
//  //   Serial.println('TKx > 0');
//  // }
//  // Serial.println(imu.tempA); 
//  // Serial.print("  ");
//  // Serial.print(imu.tempY); 
//  // Serial.print("  ");
//  // Serial.print(imu.tempZ); 
//  // Serial.print("  ");
//  Serial.print(imu.TKEulerx); 
//  Serial.print("  ");
//  Serial.print(imu.TKEulery); 
//  Serial.print("  ");
//  Serial.println(imu.TKEulerz); 
//  // Serial.println("  ");
//  // Serial.print(imu.TKAVx); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKAVy); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKAVz); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKLAx); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKLAy); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKLAz); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKqA); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKqB); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKqC); 
//  // Serial.print("  ");
//  // Serial.print(imu.TKqD); 
//  // Serial.println("  ");
//}

void IMUCurrentControl()
{
  ////******IMU+Current Control Example Torque Constant 0.6 Nm/A**********////////
  imu.READ();   //Check if IMU data available and read it. the sample rate is 200 hz

  current_time = micros();    //query current time (microsencond)

  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    previous_time_ble = current_time;
//  Serial.print(imu.TKEulerx); 
//  Serial.print("  ");
//  Serial.print(imu.TKEulery); 
//  Serial.print("  ");
//  Serial.println(imu.TKEulerz); 
  }

}
