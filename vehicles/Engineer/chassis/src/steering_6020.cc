#include "steering_6020.h"

namespace control{

    static void steer6020_callback(const uint8_t data[], void* args) {
        ServoMotor* servo = reinterpret_cast<ServoMotor*>(args);
        servo->UpdateData(data);
    }

    Steering6020::Steering6020(steering6020_t data){
        motor = data.motor;

        max_speed = data.max_speed;
        max_acceleration = data.max_acceleration;
        max_iout = data.max_iout;
        max_out = data.max_out;
        transmission_ratio = data.transmission_ratio;
        target_angle_ = 0;
        current_angle_ = motor->GetTheta();
    }

    Steering6020::~Steering6020(){
        motor
    }
    
   




}