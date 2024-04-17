#include <Arduino.h>
// #include "settings.h"
// #include "AB_Encoder.cpp"

// example code from https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl
// will change later

// PWM val >= 100 to move the Motor or else it wont move due to torque problem
// set to 0 so that it actually stop
class Motor
{
protected:
    AB_Encoder encoder;
    int INA_pin;
    int INB_pin;

public:
    // constructor
    Motor(int pin_INA, int pin_INB)
    {
        INA_pin = pin_INA;
        INB_pin = pin_INB;
        analogWrite(INA_pin, 0);
        analogWrite(INB_pin, 0);
    }
    // drive motor
    // usage: setMotor(200);
    void setMotor(int pwm)
    {
        if (pwm > 0) // rotate one way
        {
            analogWrite(INA_pin, constrain(fabs(pwm), 100, 255));
            analogWrite(INB_pin, 0);
        }
        else if (pwm < 0) // rotate the other way
        {
            analogWrite(INA_pin, 0);
            analogWrite(INB_pin, constrain(fabs(pwm), 100, 255));
        }
        else // stop rotate
        {
            analogWrite(INA_pin, 0);
            analogWrite(INB_pin, 0);
        }
    }
};