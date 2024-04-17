/*TO DO LIST:
  1. Implement controller control
  2. Migrate to ROS
  3. Test with 2 motors*/

#include <Arduino.h>
#include "settings.h"
#include "AB_Encoder.cpp"
#include "Motor_PWM_driver.cpp"

// Interrupt variables, rcords position of encoder relative to the initial position when boot up
volatile int pos_b = 0;

// Utility function to read pos_b safely to avoid Interrupt while accessing
int read_pos_b()
{
  noInterrupts(); // disable Interrupt when reading the pos
  int pos_speed1 = pos_b;
  interrupts();
  return pos_speed1;
}

// Interrupt function
void readEncoderB()
{
  int b = digitalRead(FL_B_encoder_pin);
  int increment = 0;
  if (b > 0)
    increment = 1;
  else
    increment = -1;
  pos_b = pos_b + increment;
}

void setup()
{
  Serial.begin(115200);
  // set up pins, see settings.h
  pinMode(FL_A_encoder_pin, INPUT);
  pinMode(FL_B_encoder_pin, INPUT);
  pinMode(FR_A_encoder_pin, INPUT);
  pinMode(FR_B_encoder_pin, INPUT);
  pinMode(FL_Motor_A_pin, OUTPUT);
  pinMode(FL_Motor_B_pin, OUTPUT);
  pinMode(FR_Motor_A_pin, OUTPUT);
  pinMode(FR_Motor_B_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(FL_A_encoder_pin), readEncoderB, RISING);

  Serial.println("set up complete");
}

// create objects
AB_Encoder FL_AB(FL_A_encoder_pin, FL_B_encoder_pin);
AB_Encoder FR_AB(FR_A_encoder_pin, FR_B_encoder_pin);
Motor FL_Motor(FL_A_encoder_pin, FL_B_encoder_pin);
Motor FR_Motor(FR_A_encoder_pin, FR_B_encoder_pin);

void loop()
{
  // Left forward PWM 100-255
  // Right backward PWM 100-255
  for (int i = 100; i < 255; i++)
  {
    int PID_target_L = i;
    int PID_target_R = -i;
    Serial.print("Left target speed: ");
    Serial.println(PID_target_L);
    Serial.print("Right target speed: ");
    Serial.println(PID_target_R);
    Serial.print("Left Speed: ");
    Serial.println(FL_AB.getspeed_count_pulses(read_pos_b()));
    Serial.print("Left Speed: ");
    Serial.println(FR_AB.getspeed_count_pulses(read_pos_b()));
    FL_Motor.setMotor(FL_AB.PID(PID_target_L, read_pos_b()));
    FR_Motor.setMotor(FR_AB.PID(PID_target_R, read_pos_b()));
    // delay too low will cause the PWM module stop function (lights go dim and chips gets really hot)
    delay_local(100);
  }
  // Left backward PWM 100-255
  // Right forward PWM 100-255
  for (int i = 100; i < 255; i++)
  {
    int PID_target_L = -i;
    int PID_target_R = i;
    Serial.print("Left target speed: ");
    Serial.println(PID_target_L);
    Serial.print("Right target speed: ");
    Serial.println(PID_target_R);
    Serial.print("Left Speed: ");
    Serial.println(FL_AB.getspeed_count_pulses(read_pos_b()));
    Serial.print("Left Speed: ");
    Serial.println(FR_AB.getspeed_count_pulses(read_pos_b()));
    FL_Motor.setMotor(FL_AB.PID(PID_target_L, read_pos_b()));
    FR_Motor.setMotor(FR_AB.PID(PID_target_R, read_pos_b()));
    // delay too low will cause the PWM module stop function (lights go dim and chips gets really hot)
    delay_local(100);
  }

  // TODO: ADD SUPPORT FOR CONTROLLER
}