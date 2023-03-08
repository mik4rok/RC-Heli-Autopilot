#include <math.h>
#include <Servo.h>
Servo AilServo;
Servo EleServo;

// Define PID constants ROLL
#define KRP 0.5
#define KRI 0.01
#define KRD 0.001

// Define PID constants PITCH
#define KPP 0.5
#define KPI 0.01
#define KPD 0.001

// Define PID constants YAW
#define KYP 10.0
#define KYI 0.1
#define KYD 0.01


#define RollRate 1;
#define PitchRate 0.05;
#define YawRate 0.05;

// Define control limits
#define MAX_PWM 255
#define MIN_PWM 0

float roll_integral = 0;
float prev_roll_error = 0;  // initialize to 0 as integral grows and reduced with time
float pitch_integral = 0;
float prev_pitch_error = 0;
float yaw_integral = 0;
float prev_yaw_error = 0;



void servo_setup() {
  AilServo.attach(8);
  EleServo.attach(9);
}


void sbus_to_pid(float &orientation_z, float &orientation_y, int &Thr, int &Ail, int &Ele, int &Rud, int &ch5, int &failsafe, float &rate_pitch, float &rate_roll, float &rate_yaw) {


    // calculations to output deg/s from radio angle input

    float Error_Roll = Ail - orientation_z;
    float Error_Pitch = Ele - orientation_y;
    rate_roll = Error_Roll * RollRate;
    rate_pitch = Error_Pitch * PitchRate;
    rate_yaw = Rud * YawRate;


    // Serial.print(rate_pitch);
    // Serial.print(" ");
    // Serial.print(rate_yaw);
    // Serial.print(" ");
    // Serial.println(rate_roll);
}


// Update PID controller
void pid_update(float &gyro_x, float &gyro_y, float &gyro_z, float target_roll, float target_pitch, float target_yaw, float &roll_output, float &pitch_output, float &yaw_output, int dt) {

  float roll_error = target_roll - gyro_x;    //might need to be changed based on gyro orientation
  float pitch_error = target_pitch - gyro_y;  // might need to be changed based on gyro orientation
  float yaw_error = target_yaw - gyro_z;      // might need to be changed based on gyro orientation



  roll_output = KRP * roll_error + KRI * roll_integral + KRD * (roll_error - prev_roll_error);
  pitch_output = KPP * pitch_error + KPI * pitch_integral + KPD * (pitch_error - prev_pitch_error);
  yaw_output = KYP * yaw_error + KYI * yaw_integral + KYD * (yaw_error - prev_yaw_error);

  // Update integral terms
  roll_integral += roll_error * dt;
  pitch_integral += pitch_error * dt;
  yaw_integral += yaw_error * dt;

  // Update previous error terms
  prev_roll_error = roll_error;
  prev_pitch_error = pitch_error;
  prev_yaw_error = yaw_error;

  //Serial.println("Yaw: " + String(gyro_z) + " Yaw Error: " + String(yaw_error) + " Yaw Output: " + String(yaw_output));  //debugging pid code
}

// Main function
void PIDmain(float roll_output, float pitch_output, float yaw_output) {
  // Calculate PWM values
  int roll_pwm = map(constrain(roll_output, -90, 90), -90, 90, 1200, 1800);
  int pitch_pwm = map(constrain(pitch_output, -90, 90), -90, 90, 1200, 1800);
  int yaw_pwm = map(constrain(yaw_output, -100, 100), -100, 100, 1000, 0);



  if (failsafe == 0 && ch5 > 800) {
    analogWrite(0, yaw_pwm);
    analogWrite(1, Thr);
    AilServo.writeMicroseconds(roll_pwm);   // sets the servo position to its minimum
    EleServo.writeMicroseconds(pitch_pwm);  // sets the servo position to its minimum

  } else {
    analogWrite(0, 0);
    analogWrite(1, 0);
    AilServo.writeMicroseconds(1500);   // sets the servo position to its minimum
    EleServo.writeMicroseconds(1500);  // sets the servo position to its minimum
  }



  //  Serial.println("roll PWM 1000-2000: " + String(roll_pwm) + " Yaw PID: " + String(roll_output));
  //Serial.println("Yaw PWM 0-1024: " + String(yaw_pwm) + " Yaw PID: " + String(yaw_output));
}
