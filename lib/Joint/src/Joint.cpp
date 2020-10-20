#include "Joint.h"

Joint::Joint(uint8_t servo_pin, float offset_in_degrees, int min_us, int max_us, float max_angular_speed_deg_sec, float min_limit_deg, float max_limit_deg){
    this->servo_pin = servo_pin;
    this->min_us = min_us;
    this->max_us = max_us;
    this->servo.attach(this->servo_pin, min_us, max_us);
    this->offset = deg2rad(offset_in_degrees) + PI/2;
    this->max_angular_speed = max_angular_speed_deg_sec;
    this->min_limit = deg2rad(min_limit_deg);
    this->max_limit = deg2rad(max_limit_deg);          
    // this->write(0);
}

Joint::Joint(uint8_t servo_pin) : Joint(servo_pin, 0.0, DEFAULT_uS_LOW, DEFAULT_uS_HIGH, 5.0, 0.0, 180.0){}
Joint::Joint(uint8_t servo_pin, float offset_in_degrees) : Joint(servo_pin, offset_in_degrees, DEFAULT_uS_LOW, DEFAULT_uS_HIGH, 5.0, -90.0, 90.0){}
Joint::Joint(uint8_t servo_pin, float offset_in_degrees, int min_us, int max_us) : Joint(servo_pin, offset_in_degrees, min_us, max_us, 5.0, -90.0, 90.0){}

void Joint::write(float value, bool is_rad){
    int angle_in_micros;

    if(!is_rad)
        value = deg2rad(value);
    value = constrain(value, this->min_limit, this->max_limit);
    this->current_angle = value;
    angle_in_micros = (int)round(this->angle_to_micros(value+this->offset));    
    this->servo.writeMicroseconds(angle_in_micros);
}

float Joint::micros_to_angle(float value){
    return Utils::mapf(value, this->min_us, this->max_us, 0, PI);
}
float Joint::angle_to_micros(float value){
    return Utils::mapf(value, 0, PI, this->min_us, this->max_us);
}

void Joint::write(float value){this->write(value, false);}

float Joint::get_current_angle(bool in_radians){
    if(in_radians)
        return this->current_angle;
    else
        return rad2deg(this->current_angle);
}
float Joint::get_current_angle(){return this->get_current_angle(true);}