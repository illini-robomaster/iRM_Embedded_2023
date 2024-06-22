#pragma once
#include <math.h>
typedef struct {
    float position;
    float velocity;
} kinematics_state;

class TrapezoidProfile 
{
    private:
        // Kinematics Constraints
        float acceleration;
        float cruise_velocity;

        // kinematics states
        kinematics_state current_state;
        kinematics_state goal_state;

        // estimated critical timestamps
        float end_acc_time;
        float end_cruise_time;
        float end_decc_time; 

        float last_update_time_ms;

        int direction;

    public:
        TrapezoidProfile(float acc, float cruise_vel, float init_pos, int init_time_ms) : acceleration(acc), cruise_velocity(abs(cruise_vel)), last_update_time_ms(init_time_ms){}
        
        /**
        * @brief update the internal target and return a new target that satisfy the acceleration and velocity limit.
        * @return kinematics_state which contains position and velocity of the current setpoint at time t.
        */
        kinematics_state calculate(float new_target, int current_time_ms);

        /**
         * @brief make sure the direction of the output state matches the 
        */
        kinematics_state direct(kinematics_state in);

};