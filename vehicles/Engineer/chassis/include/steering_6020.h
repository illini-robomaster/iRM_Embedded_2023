#include "motor.h"
#include "power_limit.h"
#include "controller.h"


namespace control{

typedef struct{
    MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
    float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
    float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
    float transmission_ratio; /* transmission ratio of motor                       */
    float* omega_pid_param;   /* pid parameter used to control speed of motor      */
    float max_iout;
    float max_out;

}steering6020_t;

class Steering6020{
public:
    Steering6020(steering6020_t data);
    ~Steering6020();

    void Update();
    void CalcOutput();

    void SetTarget();
    void SetMaxSpeed();
    void SetMaxAcceleration();

    void GetTarget();
    void Holding();



private:
    control::MotorCANBase* motor;
    float target_angle_;
    float current_angle_;


    bool hold_;


    float max_speed;
    float max_acceleration;
    float max_iout;
    float max_out;
    float transmission_ratio; /* transmission ratio of motor                       */

   

};

}