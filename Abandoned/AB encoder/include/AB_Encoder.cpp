#include <Arduino.h>

class AB_Encoder
{
protected:
    // encoder global variables
    int pinA;
    int pinB;

    // PID constants, need adjustments
    float kp = 5;
    float ki = 1;
    float kd = 1;

    // for filtering
    float speed1_filt = 0;
    float speed1_prev = 0;

    // PID and speed measurement 1 variables
    int pos_prev_speed1 = 0;
    long time_prev_speed1 = 0;
    float e_prev = 0;
    float e_integral = 0;

public:
    // constructor
    AB_Encoder(int A_encoder_pin = 0, int B_encoder_pin = 0)
    {
        if (A_encoder_pin == 0 || B_encoder_pin == 0)
        {
            Serial.println("AB_Encoder build failed, pin not specified");
        }
        pinA = A_encoder_pin;
        pinB = B_encoder_pin;
    }

    // speed measurement method 1
    float getspeed_count_pulses(int pos_b)
    {
        return speed1_filt;
    }

    /*  PID calculation
        retunrs PWM, the absoulte values of the result of PID calculation
        input target speed, and loop this function until it's at target speed
        usage: [encoder object].PID (target speed, read_pos_b())
    */
    int PID(int target, int pos_b)
    {
        // calculate speed using method 1
        long time_current = micros();
        float time_delta = ((float)(time_current - time_prev_speed1)) / (1.0e6); // time elapsed (second)
        float speed1 = (pos_b - pos_prev_speed1) / time_delta;
        pos_prev_speed1 = pos_b;
        time_prev_speed1 = time_current;
        // convert to rpm
        speed1 = speed1 / 585.0 * 60.0;
        // low-pass filter, might need adjustments, see https://www.youtube.com/watch?v=HJ-C4Incgpw
        /*
        float LowPassFilter::operator(float input){
  unsigned long timestamp = _micros();
  float dt = (timestamp - timestamp_prev)*1e-6f;
  // quick fix for strange cases (micros overflow)
  if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;

  // calculate the filtering 
  float alpha = Tf/(Tf + dt);
  float y = alpha*y_prev + (1.0f - alpha)*x;

  // save the variables
  y_prev = y;
  timestamp_prev = timestamp;
  return y;*/
        speed1_filt = 0.854 * speed1_filt + 0.0728 * speed1 + 0.0728 * speed1_prev;
        speed1_prev = speed1;

        // PID calculation
        float e = target - speed1_filt;            // error
        float e_deriv = (e - e_prev) / time_delta; // derivative (D)
        e_integral = e_integral + e * time_delta;  // integral   (I)
        int u = kp * e + kd * e_deriv + ki * e_integral;

        e_prev = e;
        return u;
    }
};
