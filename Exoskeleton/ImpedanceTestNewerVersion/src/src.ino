//for right knee exo, positive current assist extension, and torque sensor measures positive torque
#include "Motor_Control_Pediatric_V2.h"
#include <SPI.h>
#include <FlexCAN.h>
#include "ads1292r.h"
#include "Torque_Control.h"
#include "WL_IMU.h"
#include <Arduino.h>
#include "math.h"

CAN_message_t msgR;
struct CAN_filter_t defaultMask;
uint32_t ID_offset = 0x140;
uint32_t Motor_ID3 = 1;
uint32_t Motor_ID1 = 3;      // Motor Can Bus ID, left leg, loadcell: port 1
uint32_t Motor_ID2 = 2;      // Motor Can Bus ID, right leg, loadcell: port 2
int CAN_ID = 0;         // CAN port from Teensy
double Gear_ratio = 6; //The actuator gear ratio, will enfluence actuator angle and angular velocity
int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;
Motor_Control_Pediatric_V2 m3(Motor_ID3, CAN_ID, Gear_ratio);  //Create motor object see Motor_Control_Pediatric_V2.h
Motor_Control_Pediatric_V2 m1(Motor_ID1, CAN_ID, Gear_ratio);  //Create motor object see Motor_Control_Pediatric_V2.h
Motor_Control_Pediatric_V2 m2(Motor_ID2, CAN_ID, Gear_ratio);  //Create motor object see Motor_Control_Pediatric_V2.h
ads1292r torque_sensor1;                //Create torque sensor object see ads1292r.h
Torque_Control Tor_control;             //Creare torque control object see Torque_control.h
IMU imu;                                //Create IMU object see WL_IMU.h



int assist_mode = 1;





double weight = 52;     // [kg] weight of the subject
double Pgain = 7.5;                 //P gain of torque control
double Igain = 0.7;                 //I gain of torque control
double Dgain = 0;                   //D gain of torque control
double MaxPIDout = 10;              //Max torque control PID output (Unit Current A, inner loop is current controller)

//double theta_k_l;
//double theta_k_r;
//double k_st = 0.06; 
//double k_sw = 0.2; 
//double theta_st_l_0 = 2.33; 
//double theta_st_r_0 = 1.33; 
//double a = 0.099; 
//double b = 2.619;
double theta_k_l;
double theta_k_r;
double k_st = 0.06; 
double k_sw = 0.05; 
double theta_st_l_0 = 5; 
double theta_st_r_0 = 5; 
double a = 0.099; 
double b = 1.619;

double Fsample = 500;                   // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
unsigned long current_time = 0;
unsigned long previous_time = 0;        // used to control the controller sample rate.
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);             // used to control the teensy controller frequency
//double Tsample = 1 / Fsample;           // sample period (second)

double Cur_command_L = 0;
double Cur_command_R = 0;
int current_limitation = 3;  //(unit Amp)//double Tor_command_L = 0;
double torque_command_L = 0;
double torque_command_R = 0;

int Left_Knee_Torque = 0;
int Right_Knee_Torque = 0;
double Left_Knee_Torque_Commend;
double Right_Knee_Torque_Commend;
double Angle_desired_L;
double Angle_desired_R;
double Angle_L_pre = 0;
double Angle_R_pre = 0;
double AngleV_L = 0;
double AngleV_R = 0;
double AngleV_L_pre = 0;
double AngleV_R_pre = 0;
double freq = 0;
float count = 0;

double errSum = 0;
double lastErr = 0;

unsigned long temp = 0;

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(115200);       //used for communication with computer.
  Serial4.begin(115200);      //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  torque_sensor1.Torque_sensor_initial();     //initial the torque sensor see ads1292r.cpp.
  torque_sensor1.Torque_sensor_gain(0.0003446 * (-1) * 2, 0.0003446 * (-1) * 2.35); //set the calibration gain for torque sensor. Torque= gain* ADCvalue+offset.see ads1292r.cpp.
  torque_sensor1.Torque_sensor_offset_calibration();      //Auto offset the torque sensor to zero. see ads1292r.cpp.
  delay(500);
  imu.Gain_E = 10;            //Extension gain for delay output feedback control
  imu.Gain_F = 10;            //Flexion gain for delay output feedback control
  imu.delaypoint = 10;        //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  m3.init_motor();
  m1.init_motor();           // Strat the CAN bus communication & Motor Control
  Serial.println("M1 Success");
  m2.init_motor();           // Strat the CAN bus communication & Motor Control
  Serial.println("M2 Success");
  CurrentControlSetup();  //initialize current control and IMU control. this is use for direct current control without torque PID loop (torque reference/torque constant= current reference -> current PID loop)
  
}

void loop() {
  receive_CAN_data();
  CurrentControl();         //if you want to try torque control, please uncommment IMUCurrentControl(); and use this function.
}

void CurrentControlSetup()
{
  imu.INIT();    //Initialize IMU;
  delay(500);
  current_time = micros();
  previous_time = current_time;
  m3.read_multi_turns_angle();//read angle and angle velocity
  receive_CAN_data();
  m1.read_multi_turns_angle();//read angle and angle velocity
  receive_CAN_data();
  m2.read_multi_turns_angle();//read angle and angle velocity
  receive_CAN_data();
  Angle_L_pre = m1.motorAngle;
  Angle_R_pre = m2.motorAngle;
  m3.read_motor_status_2();//read angle and angle velocity
  receive_CAN_data();
  m1.read_motor_status_2();//read angle and angle velocity
  receive_CAN_data();
  m2.read_motor_status_2();//read angle and angle velocity
  receive_CAN_data();
  AngleV_L_pre = m1.speed_value;
  AngleV_R_pre = m2.speed_value;
}

void CurrentControl()
{
  ////******IMU+Current Control Example Torque Constant 0.6 Nm/A**********////////
//  imu.READ();   //Check if IMU data available and read it. the sample rate is 100 hz
//  torque_sensor1.Torque_sensor_read(); //Check if torque sensor1 is available // Vahid

  current_time = micros();    //query current time (microsencond)

  //********* use to control the teensy controller frequency **********//

  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    if (Stop_button) //stop
    {
      Cur_command_L = 0;
      Cur_command_R = 0;
    }
    else
    {
      
      Compute_Cur_Commands();// Vahid

    }
    Cur_limitation();
    m1.send_current_command(Cur_command_L);
//    receive_CAN_data();
    m2.send_current_command(Cur_command_R);
//    receive_CAN_data();
    //    //Cur_command_L = 2;//positive=flex
    //    //Cur_command_R = -2;//positive=extend

    previous_time = current_time; //reset previous control loop time
    //
    //  }
  }
}

  void Compute_Cur_Commands()
  {
    if (assist_mode == 0) //display joint angles
    {
      /////*********Print motor Position - Use below code to read Position 2021-08-17 by Howard*********/////
      Cur_command_L = 0;
      Cur_command_R = 0;
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      AngleV_L = m1.speed_value;
      AngleV_R = m2.speed_value;
      Serial.print(m1.motorAngle); // angle unit: degree
      Serial.print(" ,");
      Serial.print(m2.motorAngle); // angle unit: degree
      Serial.print(",");
      Serial.print(AngleV_L); // angle unit: degree
      Serial.print(",");
      Serial.println(AngleV_R); // angle unit: degree
      Cur_command_L = 0;
      Cur_command_R = 0;
//      Serial.print(m1.motorAngle); // angle unit: degree
//      Serial.print("  ");
//      Serial.println(m2.motorAngle); // angle unit: degree
    }
    else if (assist_mode == 1) //Aaron young: walking impedance
    {
      
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      AngleV_L = m1.speed_value;
      AngleV_R = m2.speed_value;
      Cur_command_L = -0.04*( m1.motorAngle + 18.50) - 0.0001 * m1.speed_value;
      Cur_command_R =  0.04*(-m2.motorAngle + 28.50) + 0.0001 * m2.speed_value;
      Serial.print(m1.motorAngle); // angle unit: degree
      Serial.print(",");
      Serial.print(Cur_command_L); // angle unit: degree
      Serial.print(",");
      Serial.print(m2.motorAngle); // angle unit: degree
      Serial.print(",");
      Serial.println(Cur_command_R); // angle unit: degree
      
    }
    
    else if (assist_mode == 2) //sine wave position // Impedance tracking
    {
      
      //      Cur_command_L =  imu.Gain_E * sin(2 * PI * current_time / 1000000)/2.2;    //(unit Amp);
      //      Cur_command_R = -imu.Gain_F * sin(2 * PI * current_time / 1000000)/2.2;    //(unit Amp);
//      Cur_command_L =  1.5 * sin(2 * PI * current_time / 1000000) / 2.2;  //(unit Amp);
//      Cur_command_R = -1.5 * sin(2 * PI * current_time / 1000000) / 2.2;  //(unit Amp);
//      milli_time_current_double = millis();
//      milli_time_current_double = milli_time_current_double / 1000.0;
//      torque_command_L = Cur_command_L * 2.2;
//      torque_command_R = Cur_command_R * 2.2;
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      Angle_desired_L = 45*sin(count/90*PI)+29; // from -16 to 74
      Angle_desired_R = 45*sin(count/90*PI)-17; // from -62 to 28 // both 90 degrees
      
//      Angle_desired_L = 0.0017*count -16; // from -16 to 74
//      Angle_desired_R = 0.0017*count -62; // from -62 to 28 // both 90 degrees
      
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
//      Serial.print(Angle_desired_L);
//      Serial.print(" ");
//      Serial.println(Angle_desired_R);
      
      Cur_command_L = -0.12*( m1.motorAngle - Angle_desired_L ) - 0.02*(m1.speed_value - 45*cos(count/90*PI)/90*PI);
      Cur_command_R = -0.12*( m2.motorAngle - Angle_desired_R ) - 0.02*(m2.speed_value - 45*cos(count/90*PI)/90*PI);
      
      Serial.print(Angle_desired_L);
      Serial.print(" ");
      Serial.print(m1.motorAngle);
      Serial.print(" ");
      Serial.print(Cur_command_L);
      Serial.print(" ");
      Serial.print(Angle_desired_R);
      Serial.print(" ");
      Serial.print(m2.motorAngle);
      Serial.print(" ");
      Serial.println(Cur_command_R);
//      Cur_command_L = 0;
//      Cur_command_R = 0;
      count =count+1;
    }
    
    else if (assist_mode == 3) // PID
    {
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      Angle_desired_L = 45*sin(count/360*PI)+29; // from -16 to 74
      Angle_desired_R = 45*sin(count/360*PI)-17; // from -62 to 28 // both 90 degrees
     
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      
     double timeChange = (double)(current_time - previous_time);
     timeChange = timeChange / 1000000;
     /*Compute all the working error variables*/
     double error = Angle_desired_L - m1.motorAngle;
     errSum += (error * timeChange);
     double dErr = (error - lastErr) / timeChange;
    
     /*Compute PID Output*/
     Cur_command_L = 0.06 * error + 0.05 * errSum + 0.01 * dErr;
    
     /*Remember some variables for next time*/
     lastErr = error;

      
      Serial.print(Angle_desired_L);
      Serial.print(" ");
      Serial.print(m1.motorAngle);
      Serial.print(" ");
      Serial.print(Cur_command_L);
      Serial.print(" ");
      Serial.print(Angle_desired_R);
      Serial.print(" ");
      Serial.print(m2.motorAngle);
      Serial.print(" ");
      Serial.println(Cur_command_R);
//      Cur_command_L = 0;
      Cur_command_R = 0;
      count =count+1;
    }
    else if (assist_mode == 4) // follow human
    {
      
      Cur_command_L = -0.1*( m1.motorAngle - Angle_L_pre ) - 0.02*(m1.speed_value - AngleV_L_pre);
      Cur_command_R = -0.1*( m2.motorAngle - Angle_R_pre ) - 0.02*(m2.speed_value - AngleV_R_pre);
      
      Serial.print(Angle_L_pre);
      Serial.print(" ");
      Serial.print(m1.motorAngle);
      Serial.print(" ");
      Serial.print(Cur_command_L);
      Serial.print(" ");
      Serial.print(Angle_R_pre);
      Serial.print(" ");
      Serial.print(m2.motorAngle);
      Serial.print(" ");
      Serial.println(Cur_command_R);
  

      Cur_command_L = Left_Knee_Torque_Commend / 2;
      Cur_command_R = Right_Knee_Torque_Commend / 2;
      torque_command_L = Left_Knee_Torque_Commend;
      torque_command_R = Right_Knee_Torque_Commend;
    }
    else if (assist_mode == 5) // stiffness control
    {
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
//      theta_k_l = m1.motorAngle+18.67;
//      theta_k_r = -m2.motorAngle+28.5;
      theta_k_l = m1.motorAngle+9;
      theta_k_r = -m2.motorAngle+19;
      
      torque_command_L = k_st * ( 1 - (1 / (1+ exp(-a * (( theta_k_r - theta_k_l ) -b) )))) * (theta_k_r - theta_st_l_0) + k_sw * (1 / (1+ exp(-a * ((theta_k_r - theta_k_l) -b) ))) * (theta_k_r - 57 );
      torque_command_R = -k_st * ( 1 - (1 / (1+ exp(-a * (( theta_k_l - theta_k_r ) -b) )))) * (theta_k_l - theta_st_r_0) + k_sw * (1 / (1+ exp(-a * ((theta_k_l - theta_k_r) -b) ))) * (theta_k_l - 57 );
      

      Cur_command_L = torque_command_L*3 ;
      Cur_command_R = torque_command_R*3 ;
      
      Serial.print(theta_k_l);
      Serial.print(" ");
      Serial.print(theta_k_r);
      Serial.print(" ");
      Serial.print(torque_command_L);
      Serial.print(" ");
      Serial.print(torque_command_R);
      Serial.print(" ");
      Serial.print(m1.iq_A);
      Serial.print(" ");
      Serial.println(m2.iq_A);
  

//      Cur_command_L = Left_Knee_Torque_Commend / 2;
//      Cur_command_R = Right_Knee_Torque_Commend / 2;
//      torque_command_L = Left_Knee_Torque_Commend;
//      torque_command_R = Right_Knee_Torque_Commend;
    }


    else if (assist_mode == 5) // impedance control for squatting
    {
      m3.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m1.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m2.read_multi_turns_angle();//read angle and angle velocity
      receive_CAN_data();
      m3.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m1.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
      m2.read_motor_status_2();//read angle and angle velocity
      receive_CAN_data();
//      theta_k_l = m1.motorAngle+18.67;
//      theta_k_r = -m2.motorAngle+28.5;
      theta_k_l = m1.motorAngle+9;
      theta_k_r = -m2.motorAngle+19;
      
      torque_command_L = k_st * ( 1 - (1 / (1+ exp(-a * (( theta_k_r - theta_k_l ) -b) )))) * (theta_k_r - theta_st_l_0) + k_sw * (1 / (1+ exp(-a * ((theta_k_r - theta_k_l) -b) ))) * (theta_k_r - 57 );
      torque_command_R = -k_st * ( 1 - (1 / (1+ exp(-a * (( theta_k_l - theta_k_r ) -b) )))) * (theta_k_l - theta_st_r_0) + k_sw * (1 / (1+ exp(-a * ((theta_k_l - theta_k_r) -b) ))) * (theta_k_l - 57 );
      

      Cur_command_L = torque_command_L*3 ;
      Cur_command_R = torque_command_R*3 ;
      
      Serial.print(theta_k_l);
      Serial.print(" ");
      Serial.print(theta_k_r);
      Serial.print(" ");
      Serial.print(torque_command_L);
      Serial.print(" ");
      Serial.print(torque_command_R);
      Serial.print(" ");
      Serial.print(m1.iq_A);
      Serial.print(" ");
      Serial.println(m2.iq_A);
  

//      Cur_command_L = Left_Knee_Torque_Commend / 2;
//      Cur_command_R = Right_Knee_Torque_Commend / 2;
//      torque_command_L = Left_Knee_Torque_Commend;
//      torque_command_R = Right_Knee_Torque_Commend;
    }
    
    else if (assist_mode == 9)
    {
      mode = "Stop";
      Cur_command_L = 0;
      Cur_command_R = 0;
    }
  }

  void Cur_limitation()
  {
    //************* Current limitation *************//
    Cur_command_L = min( current_limitation, Cur_command_L);
    Cur_command_L = max(-current_limitation, Cur_command_L);
    Cur_command_R = min( current_limitation, Cur_command_R);
    Cur_command_R = max(-current_limitation, Cur_command_R);
  }


  void initial_CAN()
  {
    //initial CAN Bus
    Can0.begin(1000000, defaultMask, 1, 1);
    delay(3000);
    pinMode(28, OUTPUT);
    digitalWrite(28, LOW);
    Serial.println("Can bus setup done...");
  }
  
  void receive_CAN_data()
  {
    while (Can0.available() > 0)
    {
      Can0.read(msgR);
      if (msgR.id == (ID_offset + Motor_ID1))
      {
        m1.DataExplanation(msgR);
      }
      else if (msgR.id == (ID_offset + Motor_ID2))
      {
        m2.DataExplanation(msgR);
      }
    }
  }
