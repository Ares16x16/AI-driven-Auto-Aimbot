#include <Arduino.h>

// AB encoder pins
#define R_A_encoder_pin 21
#define R_B_encoder_pin 19
#define L_A_encoder_pin 26
#define L_B_encoder_pin 25

// Motor PWM pins
// Left motor
#define L_R_PWM_pin 27
#define L_L_PWM_pin 33
// Right motor
#define R_R_PWM_pin 17
#define R_L_PWM_pin 23

// Servo pins
#define Servo_pan_pin  18
#define Servo_tilt_pin 22

// laser pin
#define Laser_pin 16

// Servo properties
// as servo library maps 0-180 degree to Us (ours should be 540 - 2400?)
// setting minUs and maxUs define "0 degree" position and "180 degree" postion
// maxUs is set to only 1000 because the design of turret does not allow the tilt motor to go to 180 degree
// can set 2 sets of minUs & maxUs for tilt servo and pan servo
#define minUs 544
#define maxUs 1800

// PID constants (adjusted, but can be calibrated further)
#define left_kp 1.5
#define left_ki 0.3
#define left_kd 0.1

#define right_kp 2.9
#define right_ki 0.3
#define right_kd 0.1


// Utility Functions for all programs
void delay_local(long milis)
{
  long timeout = millis() + milis;
  while (millis() < timeout)
  {
  }
}