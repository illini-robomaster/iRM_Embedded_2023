
#include "chassis_task.h"
#include "geometry.h"
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

    chassis->SteerSetMaxSpeed(RUN_SPEED);
    chassis->SetWheelSpeed(0,0,0,0);

	Vector2d prev_target_vel(0, 0);

    int loop_cnt = 0;

    while (true) {
        float relative_angle = 0;
        float wz = 0;

        Vector2d joystick_vector(0, 0);

#ifndef SINGLEBOARD
        joystick_vector = Vector2d(receive->vx, receive->vy);
#else
        joystick_vector = Vector2d(dbus->ch1/ 660.0, dbus->ch0/660.0)
#endif


        // The following is for joystick only, not for keyboard
        // Max joy stick max = 660

        // Deadzone
        const float DEADZONE = 0.1;
        if (joystick_vector.getMagnitude() < DEADZONE){
            joystick_vector = Vector2d(0, 0);
        }
        else{ 
            // make vector continous at the edge of the deadzone
            // (v - 0.1 / |v| * v) / 0.9 
            joystick_vector = joystick_vector.minus(joystick_vector.normalize().times(0.1)).times(1.0/(1-DEADZONE));
        }

        // Normalize the vector if it is too large
        if (joystick_vector.getMagnitude() > 1){
            joystick_vector = joystick_vector.normalize();
        }

        // To change direction, change the signs of joy_x and joy_y
        Vector2d target_vel(joystick_vector.getX() * V_TRANS_MAX, -joystick_vector.getY() * V_TRANS_MAX); // in m/s

        // Translational acceleration kinematic constraints
        // The benefit of this compared to the previous one is that it has a constant acceleration and converges faster
        // Calculate the delta V

        Vector2d delta_v = target_vel.minus(prev_target_vel);
        // find the maximum delta V that is under the acceleration limit
        float max_delta_v_mag = ACC_TRANS_MAX * CHASSIS_TASK_DELAY / 1000.0; 
        // cap delta_v to this magnitude
        if (delta_v.getMagnitude() > max_delta_v_mag){
            delta_v = delta_v.normalize().times(max_delta_v_mag);
        }
        // v_set = v_prev + delta_v
        target_vel = prev_target_vel.plus(delta_v);
        // TODO: Rotational acceleration constraints (which needs to deal with each module's angle)
        prev_target_vel = target_vel;

        if(loop_cnt == 100){
            loop_cnt = 0;
            set_cursor(0, 0);
            clear_screen();
            print("joy_x: %f, joy_y: %f \r\n", joystick_vector.getX(), joystick_vector.getY());
            print("vx: %f, vy: %f, wz: %f \r\n", target_vel.getX(), target_vel.getY(), wz);
        }
        loop_cnt++;

        // calculate camera oriented velocity vector
        Vector2d target_vel_cam = target_vel.rotateBy(Angle2d(relative_angle)); // now relative_angle is 0

        // wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * dbus->ch2);       /* TODO : ASK IF GIMBAL EXIST, HOW CHASSIS MOVE */
        wz = dbus->ch2 / 660.0 * V_ROT_MAX; // in m/s

        
#ifndef  SINGLEBOARD
        wz = receive->relative_angle;
#else
        wz = 0;
#endif


        // if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
        chassis->SteerSetMaxSpeed(RUN_SPEED);
        chassis->SetSpeed(target_vel_cam.getX(), target_vel_cam.getY(), wz);
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

