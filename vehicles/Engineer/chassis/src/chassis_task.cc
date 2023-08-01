
#include "chassis_task.h"


//Constants 
// static unsigned int flag_summary = 0;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;

/*Args*/
static control::MotorCANBase* motor1 = nullptr;
static control::MotorCANBase* motor2 = nullptr;
static control::MotorCANBase* motor3 = nullptr;
static control::MotorCANBase* motor4 = nullptr;
static control::MotorCANBase* motor5 = nullptr;
static control::MotorCANBase* motor6 = nullptr;
static control::MotorCANBase* motor7 = nullptr;
static control::MotorCANBase* motor8 = nullptr;

static control::ServoMotor* steering_motor1 = nullptr;
static control::ServoMotor* steering_motor2 = nullptr;
static control::ServoMotor* steering_motor3 = nullptr;
static control::ServoMotor* steering_motor4 = nullptr;

static control::engineer_steering_chassis_t* chassis_data;
static control::EngineerSteeringChassis* chassis;

static volatile bool Dead = false;




void chassisTask(void* arg){
    UNUSED(arg);


    control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
    control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

    
    chassis->SteerSetMaxSpeed(ALIGN_SPEED);
    

    chassis->SteerCalcOutput();
    control::MotorCANBase::TransmitOutput(steer_motors, 4);

    chassis->SteerSetMaxSpeed(RUN_SPEED);
    // chassis->SteerThetaReset();
    chassis->SetWheelSpeed(0,0,0,0);

    while (true) {
        // float relative_angle = receive->relative_angle;
        float relative_angle = 0;
        float sin_yaw, cos_yaw, vx_set, vy_set;
        float vx, vy, wz; 
        // chassis->PrintData();


        // vx_set = -receive->vy;
        // vy_set = receive->vx;
        vx_set = dbus->ch3;
        vy_set = dbus->ch2;



        chassis->SteerSetMaxSpeed(RUN_SPEED);
        sin_yaw = sin(relative_angle);
        cos_yaw = cos(relative_angle);
        vx = cos_yaw * vx_set + sin_yaw * vy_set;
        vy = -sin_yaw * vx_set + cos_yaw * vy_set;
        wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * relative_angle);       /* TODO : ASK IF GIMBAL EXIST, HOW CHASSIS MOVE */
        if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;

        chassis->SetSpeed(vx / 10, vy / 10, wz);
        chassis->SteerUpdateTarget();
        constexpr float WHEEL_SPEED_FACTOR = 4;
        chassis->WheelUpdateSpeed(WHEEL_SPEED_FACTOR);
        chassis->SteerCalcOutput();


        // chassis->Update((float)referee->game_robot_status.chassis_power_limit,
        //                   referee->power_heat_data.chassis_power,
        //                   (float)referee->power_heat_data.chassis_power_buffer);
        chassis->Update(5000.0,
                        0.0,
                        0.0);


        if (Dead) {
        chassis->SetSpeed(0,0,0);
        motor5->SetOutput(0);
        motor6->SetOutput(0);
        motor7->SetOutput(0);
        motor8->SetOutput(0);
        }
        // chassis->PrintData();
        control::MotorCANBase::TransmitOutput(wheel_motors, 4);
        control::MotorCANBase::TransmitOutput(steer_motors, 4);


        osDelay(CHASSIS_TASK_DELAY);



    }
}



void init_chassis(){
    motor1 = new control::Motor6020(can1, 0x205);
    motor2 = new control::Motor6020(can1, 0x206);
    motor3 = new control::Motor6020(can1, 0x207);
    motor4 = new control::Motor6020(can1, 0x208);

    motor5 = new control::Motor3508(can2, 0x201);
    motor6 = new control::Motor3508(can2, 0x202);
    motor7 = new control::Motor3508(can2, 0x203);
    motor8 = new control::Motor3508(can2, 0x204);

    chassis_data = new control::engineer_steering_chassis_t();

  control::servo_t servo_data;
  servo_data.motor = motor1;
  servo_data.max_speed = RUN_SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = 8;
  servo_data.omega_pid_param = new float[3]{3, 2, 1};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;

  /*ALIGNMENT IS NOT APPLICABLE IN THIS VEHICLE*/
  steering_motor1 = new control::ServoMotor(servo_data);
  servo_data.motor = motor2;
  steering_motor2 = new control::ServoMotor(servo_data);
  servo_data.motor = motor3;
  steering_motor3 = new control::ServoMotor(servo_data);
  servo_data.motor = motor4;
  steering_motor4 = new control::ServoMotor(servo_data);

  chassis_data = new control::engineer_steering_chassis_t();

  chassis_data->fl_steer_motor = steering_motor4;
  chassis_data->fr_steer_motor = steering_motor3;
  chassis_data->bl_steer_motor = steering_motor1;
  chassis_data->br_steer_motor = steering_motor2;

  chassis_data->fl_steer_motor_raw = motor4;
  chassis_data->fr_steer_motor_raw = motor3;
  chassis_data->bl_steer_motor_raw = motor1;
  chassis_data->br_steer_motor_raw = motor2;

  chassis_data->fl_wheel_motor = motor8;
  chassis_data->fr_wheel_motor = motor7;
  chassis_data->bl_wheel_motor = motor5;
  chassis_data->br_wheel_motor = motor6;

  chassis = new control::EngineerSteeringChassis(chassis_data);


}
void kill_chassis(){
    RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

    control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

    RGB->Display(display::color_blue);

    chassis->SteerAlignFalse();   // set alignment status of each wheel to false

    while (true) {
        motor5->SetOutput(0);
        motor6->SetOutput(0);
        motor7->SetOutput(0);
        motor8->SetOutput(0);

        control::MotorCANBase::TransmitOutput(wheel_motors, 4);

        osDelay(KILLALL_DELAY);
    }

}

bool steering_align_detect1() { return motor1->GetTheta() < 0.1 && motor1->GetTheta() > -0.1; }
bool steering_align_detect2() { return motor1->GetTheta() < 0.1 && motor1->GetTheta() > -0.1; }
bool steering_align_detect3() { return motor1->GetTheta() < 0.1 && motor1->GetTheta() > -0.1; }
bool steering_align_detect4() { return motor1->GetTheta() < 0.1 && motor1->GetTheta() > -0.1; }



