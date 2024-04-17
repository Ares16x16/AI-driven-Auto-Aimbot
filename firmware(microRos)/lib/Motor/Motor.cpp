#include <Arduino.h>
#include <QuickPID.h>

#include <Motor.h>

Motor::Motor(int R_pin_PWM, int L_pin_PWM, int pin_A, int pin_B)
{
  Motor::R_PWM_pin = R_pin_PWM;
  Motor::L_PWM_pin = L_pin_PWM;
  pinMode(R_PWM_pin, OUTPUT);
  pinMode(L_PWM_pin, OUTPUT);
  // INPUT_PULLUP copied, idk what it really does too
  pinMode(pin_A, INPUT_PULLUP);
  pinMode(pin_B, INPUT_PULLUP);
  QuickPID PID (&input, &output, &target);
  Motor::PID = PID;
}

void Motor::setMotor(float pwm)
{
  if (target> 0)  // rotate R
  {
    analogWrite(R_PWM_pin, constrain(fabs(pwm), 30, 60));
    analogWrite(L_PWM_pin, 0);
  }
  else if (target < 0)  // rotate L
  {
    analogWrite(L_PWM_pin, constrain(fabs(pwm), 30, 60));
    analogWrite(R_PWM_pin, 0);
  }
  else  // stop rotate
  {
    analogWrite(L_PWM_pin, 0);
    analogWrite(R_PWM_pin, 0);
  }
}

float Motor::getSpeed(float d_pos_b){
  float speed = d_pos_b / (millis() - time_prev);
  float rps = speed / 274.0;
  return (2 * 3.141592653589793 * 5 * rps);
}