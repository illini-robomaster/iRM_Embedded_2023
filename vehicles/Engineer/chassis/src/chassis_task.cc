
#include "chassis_task.h"


//Constants 
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

static control::Steering6020* steering_motor1 = nullptr;
static control::Steering6020* steering_motor2 = nullptr;
static control::Steering6020* steering_motor3 = nullptr;
static control::Steering6020* steering_motor4 = nullptr;

static control::engineer_steering_chassis_t* chassis_data;
static control::EngineerSteeringChassis* chassis;

static volatile bool Dead = false;


void chassisTask(void* arg){
    UNUSED(arg);

    control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
    control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

    chassis->SteerSetMaxSpeed(ALIGN_SPEED);

    bool aligned = false;
    while(!aligned){
        aligned = chassis->Calibrate();
        chassis->SteerCalcOutput();
        control::MotorCANBase::TransmitOutput(steer_motors, 4);
        osDelay(1);
    }

    chassis->SteerSetMaxSpeed(RUN_SPEED);
    chassis->SetWheelSpeed(0,0,0,0);

    while (true) {
        float relative_angle = 0;
        float sin_yaw, cos_yaw, vx_set, vy_set;
        float vx, vy, wz; 

        // vx_set = -receive->vx;
        // vy_set = receive->vy;

        vx_set = -dbus->ch3;
        vy_set = dbus->ch2;

       

        chassis->SteerSetMaxSpeed(RUN_SPEED);
        sin_yaw = sin(relative_angle);
        cos_yaw = cos(relative_angle);
        vx = cos_yaw * vx_set + sin_yaw * vy_set;
        vy = -sin_yaw * vx_set + cos_yaw * vy_set;
        wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * relative_angle);       /* TODO : ASK IF GIMBAL EXIST, HOW CHASSIS MOVE */
        wz = dbus->ch0;
        // if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;

        chassis->SetSpeed(vx / 10, vy / 10, wz/10);
        chassis->SteerUpdateTarget();
        constexpr float WHEEL_SPEED_FACTOR = 4;
        chassis->WheelUpdateSpeed(WHEEL_SPEED_FACTOR);
        chassis->SteerCalcOutput();
        chassis->PrintData();


        // chassis->Update((float)referee->game_robot_status.chassis_power_limit,
        //                   referee->power_heat_data.chassis_power,
        //                   (float)referee->power_heat_data.chassis_power_buffer);
        chassis->Update(0,
                        0,
                        0);


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

        // TODO: Enable code below when referee ready.
        // receive->cmd.id = bsp::SHOOTER_POWER;
        // receive->cmd.data_bool = referee->game_robot_status.mains_power_shooter_output;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::COOLING_HEAT1;
        // receive->cmd.data_float = (float)referee->power_heat_data.shooter_id1_17mm_cooling_heat;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::COOLING_HEAT2;
        // receive->cmd.data_float = (float)referee->power_heat_data.shooter_id2_17mm_cooling_heat;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::COOLING_LIMIT1;
        // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_cooling_limit;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::COOLING_LIMIT2;
        // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_cooling_limit;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::SPEED_LIMIT1;
        // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_speed_limit;
        // receive->TransmitOutput();

        // receive->cmd.id = bsp::SPEED_LIMIT2;
        // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_speed_limit;
        // receive->TransmitOutput();



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

  control::steering6020_t servo_data;
  servo_data.motor = motor1;
  servo_data.max_speed = RUN_SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = 1;
  servo_data.omega_pid_param = new float[3]{20000, 100, 500};
  servo_data.max_iout = 30000;
  servo_data.max_out = 20000;
  servo_data.install_offset = BL_MOTOR_OFFSET;


  /*ALIGNMENT IS NOT APPLICABLE IN THIS VEHICLE*/
  steering_motor1 = new control::Steering6020(servo_data);
  servo_data.motor = motor2;
  servo_data.install_offset = BR_MOTOR_OFFSET;
  steering_motor2 = new control::Steering6020(servo_data);
  servo_data.motor = motor3;
  servo_data.install_offset = FR_MOTOR_OFFSET;
  steering_motor3 = new control::Steering6020(servo_data);
  servo_data.motor = motor4;
  servo_data.install_offset = FL_MOTOR_OFFSET;
  steering_motor4 = new control::Steering6020(servo_data);

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
   // set alignment status of each wheel to false

    while (true) {
        motor5->SetOutput(0);
        motor6->SetOutput(0);
        motor7->SetOutput(0);
        motor8->SetOutput(0);

        control::MotorCANBase::TransmitOutput(wheel_motors, 4);

        osDelay(KILLALL_DELAY);
    }

}

