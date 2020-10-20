#ifndef _JOINT_H_
#define _JOINT_H_

#include <Arduino.h>
// #include <Servo.h>
#include <ESP32Servo.h>
#include <Interpolations.h>
#include <Utils.h>

class Joint{
    private:
        int min_us, max_us;
        float current_angle, max_angular_speed, offset, min_limit, max_limit;
        uint8_t servo_pin;
        Servo servo;
    public:
        Joint(uint8_t);
        Joint(uint8_t, float);
        Joint(uint8_t, float, int, int);
        Joint(uint8_t, float, int, int, float, float, float);
        void step();
        float get_current_angle(bool);
        float get_current_angle();        
        void write(float, bool);
        void write(float);
        float micros_to_angle(float);
        float angle_to_micros(float);
};

#endif