#include <Arduino.h>

// AB encoder pins
#define FL_A_encoder_pin 32
#define FL_B_encoder_pin 33
#define FR_A_encoder_pin 26
#define FR_B_encoder_pin 27
// Motor PWM pins
#define FL_Motor_A_pin 14
#define FL_Motor_B_pin 13
#define FR_Motor_A_pin 36 // haven't decide
#define FR_Motor_B_pin 37 // haven't decide

// Utility Functions for all programs
void delay_local(long milis)
{
    long timeout = millis() + milis;
    while (millis() < timeout)
    {
    }
}