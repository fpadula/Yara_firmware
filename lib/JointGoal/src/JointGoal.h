#ifndef _JOINTGOAL_H_
#define _JOINTGOAL_H_

#include <Interpolations.h>
#include <Joint.h>
#include <Utils.h>

class JointGoal{
    private:        
        float (*ifunc)(float, float, float), goal_angle, start_angle, ang_speed;
        unsigned long start, duration, min_duration;
        bool first_step, reached_goal; 
        Joint *j;        
    public:
        JointGoal(Joint *, float, float);
        JointGoal(Joint *, float, float, float (*)(float, float, float));
        void step();
        bool reached();        
};

#endif