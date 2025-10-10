///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// #include <soc/soc.h>
// #include <soc/rtc_cntl_reg.h>

String SSID = "Drony";
String Password = "11223344";

IPAddress App_IP(192, 168, 4, 2);

WiFiUDP UDP;
unsigned int UDP_Port = 12345;
int CH_1, CH_2, CH_3, CH_4;
float Pr, I, D, A, RT, PT;

#define frequency 8000
#define resolution 8
#define pwmPin1 18
#define pwmPin2 25
#define pwmPin3 33
#define pwmPin4 32
#define m1_channel 0
#define m2_channel 1
#define m3_channel 2
#define m4_channel 3

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 50;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the yaw P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the yaw I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the yaw D-controller (default = 0.0).
int pid_max_yaw = 50;                     //Maximum output of the PID-controller (+/-).

uint8_t gyro_address = 0x68;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t MotorInput1, MotorInput2, MotorInput3, MotorInput4;
int16_t throttle, cal_int;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

uint32_t loop_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;

uint8_t error;
float battery_voltage;
unsigned long error_timer = 0;   // to store millis timing
int error_counter = 0;           // counts number of flashes
int error_led = 0;               // LED state flag

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Serial.begin(115200);                                                     // disable this while flying
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);
  ledcAttachChannel(pwmPin1, frequency, resolution, 0);
  ledcAttachChannel(pwmPin2, frequency, resolution, 1);
  ledcAttachChannel(pwmPin3, frequency, resolution, 2);
  ledcAttachChannel(pwmPin4, frequency, resolution, 3);

  WiFi.softAP(SSID, Password, 11);
  UDP.begin(UDP_Port);

  pinMode(17,OUTPUT);
  //Check if the MPU-6050 is responding.
  Wire.begin();                                                //Start the I2C as master

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                    // Stay here if MPU-6050 did not respond
    error = 1;                                                  //Set the error status to 1.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  
  battery_voltage = (float)analogRead(34)/4095*3.3*2.157;
  while(battery_voltage<=3.5){
    error = 2;                                                  //Set the error status to 1.
    error_signal();                                             //Show the error via the red LED.
    delay(4); 
  }
  gyro_setup();                                                 //Initialize the gyro and set the correct registers.
  delay(5000);
  calibrate_gyro();                                             //Calibrate the gyro offset.
  loop_timer = micros();                                        //Set the timer for the first loop.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(34)/4095*3.3*2.157*0.08 );
  if (battery_voltage < 3.5)error = 1;
  error_signal();  
  gyro_signalen();                                                                 //Read the gyro and accelerometer data.
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle.
  angle_yaw += (float)gyro_yaw * 0.0000611;                                        //Calculate the traveled yaw angle.
  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;

  //Transfer roll/pitch due to yaw
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  if (abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;
 
  if (UDP.parsePacket()) {
    char packetBuffer[64];
    int len = UDP.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      if (sscanf(packetBuffer, "%d,%d,%d,%d,%f,%f,%f,%f,%f,%f", 
                 &CH_1, &CH_2, &CH_3, &CH_4, &Pr, &I, &D, &A, &RT, &PT) != 10) {
        Serial.println("Parsing failed!");
        return;
      }
    }
  }

  pid_roll_setpoint = 250*(CH_1-127)/127;
  pid_pitch_setpoint = -250*(CH_2-127)/127;
  throttle = CH_3;
  pid_yaw_setpoint = 250*(CH_4-127)/127;
  pid_yaw_setpoint = 0;

  calculate_pid();


  if (throttle > 240) throttle = 240;
  MotorInput1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
  MotorInput2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
  MotorInput3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
  MotorInput4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

  if (MotorInput1 < 10) MotorInput1 = 10;
  if (MotorInput2 < 10) MotorInput2 = 10;
  if (MotorInput3 < 10) MotorInput3 = 10;
  if (MotorInput4 < 10) MotorInput4 = 10;

  if (MotorInput1 > 250) MotorInput1 = 240;
  if (MotorInput2 > 250) MotorInput2 = 240;
  if (MotorInput3 > 250) MotorInput3 = 240;
  if (MotorInput4 > 250) MotorInput4 = 240;

  if (throttle < 10) {
    MotorInput1 = 0;
    MotorInput2 = 0;
    MotorInput3 = 0;
    MotorInput4 = 0;
    reset_pid();
  }

  ledcWrite(pwmPin1, MotorInput1);
  ledcWrite(pwmPin2, MotorInput2);
  ledcWrite(pwmPin3, MotorInput3); 
  ledcWrite(pwmPin4, MotorInput4);

  if (micros() - loop_timer > 4050) {
    Serial.println("Loop overrun!");
  }
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}