
#include "chassis_task.h"
//#define SINGLEBOARD

//Constants 
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;

static const float NORMALIZATION_FACTOR = 1;
  // Prevent vehicle from flipping due to sudden changes in velocity.
  // Speed at which safety will not kick in
  // TODO: Find good numbers

static const float MOMENTUM_SAFE_SPEED = V_MAX * 0.3; // Magic number
static const float MOMENTUM_SAFE_FACTOR_LINE = 0.3;   // also ^
static const float MOMENTUM_SAFE_FACTOR_TURN = 0.3;   // also ^
static const float MOMENTUM_FACTOR_PER_SEC = 0.1;     // the velocity at half a second before only have 10% impact on curr target
static const float MOMENTUM_FACTOR = 1-pow(MOMENTUM_FACTOR_PER_SEC, 1.0/500/0.5);  


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

int16_t loop_count = 0;

void chassisTask(void* arg){
    UNUSED(arg);

    control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
    control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

    chassis->SteerSetMaxSpeed(ALIGN_SPEED);
	chassis->Calibrate();
    // bool aligned = false;
    // while(!aligned){
    //     aligned = chassis->Calibrate();
    //     chassis->SteerCalcOutput();
    //     control::MotorCANBase::TransmitOutput(steer_motors, 4);
	// 	
    //     osDelay(1);
    // }

    chassis->SteerSetMaxSpeed(RUN_SPEED);
    chassis->SetWheelSpeed(0,0,0,0);

	float prev_vx = 0;
	float prev_vy = 0;
	// float prev_mag = 0;

    while (true) {
        float relative_angle = 0;
        float sin_yaw, cos_yaw, vx_set, vy_set;
        float vx, vy, wz;

#ifndef SINGLEBOARD
         vx_set = -receive->vx;
         vy_set = receive->vy;
#else
        vx_set = 0;
        vy_set = 0;
#endif


        // The following is for joystick only, not for keyboard
        // Max joy stick max = 660
        float joy_x = dbus->ch1 / 660.0;
        float joy_y = dbus->ch0 / 660.0;    

        // Deadzone
        const float DEADZONE = 0.1;
        if (fabs(joy_x) < DEADZONE) joy_x = 0;
        if (fabs(joy_y) < DEADZONE) joy_y = 0;

        // Continous mapping at the edge of the deadzone
        if (joy_x > DEADZONE) joy_x = (joy_x-DEADZONE) / (1-DEADZONE);
        if (joy_x < -DEADZONE) joy_x = (joy_x+DEADZONE) / (1-DEADZONE);
        if (joy_y > DEADZONE) joy_y = (joy_y-DEADZONE) / (1-DEADZONE);
        if (joy_y < -DEADZONE) joy_y = (joy_y+DEADZONE) / (1-DEADZONE);


        // To change direction, change the signs of joy_x and joy_y
        vx_set = joy_x * V_TRANS_MAX; // in m/s
        vy_set = -joy_y * V_TRANS_MAX; // in m/s

        
        // Normalize the vector
        float norm_scale_factor = 1;
        float mag = sqrt(vx_set * vx_set + vy_set * vy_set);
        if (mag > V_TRANS_MAX){
            norm_scale_factor = 1 / mag;
        }

		vx_set *= norm_scale_factor;
		vy_set *= norm_scale_factor;

		// "Momentum" smoothing mechanism
        // Deprecated now
		vx_set = vx_set * MOMENTUM_FACTOR +
				prev_vx * (1 - MOMENTUM_FACTOR);
		vy_set = vy_set * MOMENTUM_FACTOR +
				prev_vy * (1 - MOMENTUM_FACTOR);

        // Translational acceleration kinematic constraints
        // The benefit of this compared to the previous one is that it has a constant acceleration and converges faster
        // Calculate the delta V
        float delta_vx = vx_set - prev_vx;
        float delta_vy = vy_set - prev_vy;
        // find the maximum delta V that is under the acceleration limit
        float max_delta_v_mag = ACC_TRANS_MAX * CHASSIS_TASK_DELAY / 1000.0; 
        // cap delta_v to this magnitude
        float delta_v_mag = sqrt(delta_vx * delta_vx + delta_vy * delta_vy);
        if (delta_v_mag > max_delta_v_mag){
            float scale_factor = max_delta_v_mag / delta_v_mag;
            delta_vx *= scale_factor;
            delta_vy *= scale_factor;
        }
        // v_set = v_prev + delta_v
        vx_set = prev_vx + delta_vx;
        vy_set = prev_vy + delta_vy;

        // TODO: Rotational acceleration constraints (which needs to deal with each module's angle)

		prev_vx = vx_set;
		prev_vy = vy_set;

        chassis->SteerSetMaxSpeed(RUN_SPEED);
        sin_yaw = sin(relative_angle);
        cos_yaw = cos(relative_angle);
        vx = cos_yaw * vx_set + sin_yaw * vy_set;
        vy = -sin_yaw * vx_set + cos_yaw * vy_set;
        // wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * dbus->ch2);       /* TODO : ASK IF GIMBAL EXIST, HOW CHASSIS MOVE */
        wz = dbus->ch2 / 660.0 * V_ROT_MAX; // in m/s
#ifndef  SINGLEBOARD
        // wz = receive->relative_angle;
#else
        wz = 0;
#endif
        // if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
        chassis->SetSpeed(vx, vy, wz);
        chassis->SteerUpdateTarget();
        chassis->WheelUpdateSpeed();
        chassis->SteerCalcOutput();


		// if(loop_count == 100){
		// 	clear_screen();
   		// 	set_cursor(0, 0);

        //     print("fl_steer error: %f \r\n", chassis_data->fl_steer_motor->GetTarget()-chassis_data->fl_steer_motor->GetTheta());
		// 	print("fr_steer error: %f \r\n", chassis_data->fl_steer_motor->GetTarget()-chassis_data->fr_steer_motor->GetTheta());
		// 	print("bl_steer error: %f \r\n", chassis_data->bl_steer_motor->GetTarget()-chassis_data->bl_steer_motor->GetTheta());
		// 	print("br_steer error: %f \r\n", chassis_data->br_steer_motor->GetTarget()-chassis_data->br_steer_motor->GetTheta());
        //     loop_count = 0;
        // }
        // loop_count ++;
        
#ifdef REFEREE
        chassis->Update((float)referee->game_robot_status.chassis_power_limit,
                           referee->power_heat_data.chassis_power,
                           (float)referee->power_heat_data.chassis_power_buffer);
#endif
        chassis->Update(50,
                       50,
                       50);


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

  control::steering6020_t servo_data;
  servo_data.motor = motor1;
  servo_data.max_speed = RUN_SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = 1;
  servo_data.omega_pid_param = new float[3]{80000, 0, 2000};
//   servo_data.omega_pid_param = new float[3]{0, 0, 0};
  servo_data.max_iout = 30000;
  servo_data.max_out = 20000;


  /*ALIGNMENT IS NOT APPLICABLE IN THIS VEHICLE*/
  servo_data.install_offset = BL_MOTOR_OFFSET;
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

