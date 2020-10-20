#include "JointGoal.h"

JointGoal::JointGoal(Joint *j, float goal_angle_in_rads, float ang_speed_rads_s, float (*ifunc)(float, float, float)){
    this->j = j;
    this->goal_angle = goal_angle_in_rads;
    this->ifunc = ifunc;    
    this->ang_speed = ang_speed_rads_s;
    this->first_step = true;
    this->reached_goal = false;
    this->min_duration = 1000;
}

JointGoal::JointGoal(Joint *j, float goal_angle_in_rads, float ang_speed_rads_s) : JointGoal(j, goal_angle, ang_speed_rads_s, Interpolations::linear){}

void JointGoal::step(){
    float t,to_servo;    

    if(!this->reached_goal){
        if(this->first_step){
            this->first_step = false;
            this->start = micros();
            this->start_angle = this->j->get_current_angle();
            this->duration = (unsigned long)((Utils::absf(this->goal_angle - this->start_angle)/this->ang_speed) * 1000000.0);
            // Setting minimum duration of 15ms
            // if(this->duration < this->min_duration){
            //     Serial.print("Duration less than minimum: ");
            //     Serial.print(this->duration);
            //     Serial.print(". Delta angle: "); Serial.println(Utils::absf(this->goal_angle - this->start_angle));
            //     this->duration = this->min_duration;            
            // } 
        }        
        t = constrain((((float)(micros() - this->start))/this->duration), 0.0, 1.0);
        if(t >= 1.0){
            this->reached_goal = true;
            t = 1.0;
        }
        to_servo = this->ifunc(t, this->start_angle, this->goal_angle);        
        this->j->write(to_servo, true);
    }
}

bool JointGoal::reached(){return this->reached_goal;}