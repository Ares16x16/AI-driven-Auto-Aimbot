#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <QuickPID.h>

class Motor
{
protected:
  // motor pins
  int R_PWM_pin;
  int L_PWM_pin;
  // encoder pins
  int A_pin;
  int B_pin;
  // prev variables
  unsigned long time_prev;

public:
// PID values, made public to easily access
  float input;
  float output;
  float target;
  QuickPID PID;
  
// member functions 
  Motor(int R_pin_PWM, int L_pin_PWM, int pin_A, int pin_B);
  void setMotor(float pwm);
  float getSpeed(float d_pos_b);
};

#endif