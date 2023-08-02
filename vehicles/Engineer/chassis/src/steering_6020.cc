#include "steering_6020.h"

namespace control{

    static void steering6020_callback(const uint8_t data[], void* args) {
        Steering6020* motor = reinterpret_cast<Steering6020*>(args);
        motor->UpdateData(data);
    }

    Steering6020::Steering6020(steering6020_t data){
        motor_ = data.motor;

        max_speed_= data.max_speed;
        max_acceleration_ = data.max_acceleration;
        max_iout_ = data.max_iout;
        max_out_ = data.max_out;
        transmission_ratio_ = data.transmission_ratio;
        target_angle_ = 0;
        current_angle_ = motor_->GetTheta();
        //TODO: CHEKC THESE TWO VALUES
        proximity_in_ = 0.1;
        proximity_out_ = 0.1;

        hold_ = true;
        target_angle_ = 0;
        hold_detector_ = new BoolEdgeDetector(false);

        omega_pid_.Reinit(data.omega_pid_param, data.max_iout, data.max_out);
        
        data.motor->can_->RegisterRxCallback(data.motor->rx_id_, steering6020_callback, this);

    }

    Steering6020::~Steering6020(){
        motor_ = nullptr;
    }

    void Steering6020::UpdateData(const uint8_t data[]){
        motor_->UpdateData(data);

        current_angle_ = motor_->GetTheta();
    }

    void Steering6020::CalcOutput(){
        float diff = 0;
        float out = 0;
        
        diff = motor_->GetThetaDelta(target_angle_);
        out = omega_pid_.ComputeOutput(diff);

        // print("Motor output : %4d",command);
        motor_->SetOutput(out);
    }

    servo_status_t Steering6020::SetTarget(float target, bool override){
        if(motor_->GetThetaDelta(target_angle_) < 0.1 || override) {
            target_angle_ = target;
            return TURNING_CLOCKWISE;
        }
        return INPUT_REJECT;
    }

    void Steering6020::SetMaxSpeed(float max_speed){max_speed_ = max_speed;}

    void Steering6020::SetMaxAcceleration(float max_acceleration){max_acceleration_ = max_acceleration;};

    float Steering6020::GetTarget(){return target_angle_;}

    float Steering6020::GetTheta(){return current_angle_;}


    bool Steering6020::inPosition(){return abs(target_angle_- motor_->GetTheta())<0.1;}

    void Steering6020::PrintData(){
        print("Current: %04f, Target: %04f", current_angle_,target_angle_);
    }




    
   




}