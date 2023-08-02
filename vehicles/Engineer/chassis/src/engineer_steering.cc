 #include "engineer_steering.h"

template <typename T>
bool equal(T value, T number) {
  const T diff = value-number;
  return abs(diff) < 0.5 ? true : false;
}
    
namespace control{

    EngineerSteeringChassis::EngineerSteeringChassis(engineer_steering_chassis_t* _chassis) {
  if (_chassis == nullptr) {
    RM_ASSERT_TRUE(false, "Chassis pointer is null\r\n");
  }

  if (_chassis->fl_steer_motor == nullptr || _chassis->fr_steer_motor == nullptr ||
      _chassis->bl_steer_motor == nullptr || _chassis->br_steer_motor == nullptr ||
      _chassis->fl_wheel_motor == nullptr || _chassis->fr_wheel_motor == nullptr ||
      _chassis->bl_wheel_motor == nullptr || _chassis->br_wheel_motor == nullptr) {
    RM_ASSERT_TRUE(false, "At least one motor pointer is null\r\n");
  }

  // Init Steer Motors
  fl_steer_motor = _chassis->fl_steer_motor;
  fr_steer_motor = _chassis->fr_steer_motor;
  bl_steer_motor = _chassis->bl_steer_motor;
  br_steer_motor = _chassis->br_steer_motor;

  fl_steer_motor_raw = _chassis->fl_steer_motor_raw;
  fr_steer_motor_raw = _chassis->fr_steer_motor_raw;
  bl_steer_motor_raw = _chassis->bl_steer_motor_raw;
  br_steer_motor_raw = _chassis->br_steer_motor_raw;

  // Init Wheel Motors
  fl_wheel_motor = _chassis->fl_wheel_motor;
  fr_wheel_motor = _chassis->fr_wheel_motor;
  bl_wheel_motor = _chassis->bl_wheel_motor;
  br_wheel_motor = _chassis->br_wheel_motor;

  fl_steer_motor->CalcOutput();
  fr_steer_motor->CalcOutput();
  bl_steer_motor->CalcOutput();
  br_steer_motor->CalcOutput();

  fl_wheel_motor->SetOutput(0.0);
  fr_wheel_motor->SetOutput(0.0);
  bl_wheel_motor->SetOutput(0.0);
  br_wheel_motor->SetOutput(0.0);
  // safety lock ends

  // Init private variables starts
  vx = 0.0;
  vy = 0.0;
  vw = 0.0;

  v_fl_ = 0.0;
  v_fr_ = 0.0;
  v_bl_ = 0.0;
  v_br_ = 0.0;

  in_position = false;
  in_position_detector = new BoolEdgeDetector(false);


  delta_angle_fl_ = wrap<double>(FL_MOTOR_OFFSET,0,2*PI);
  delta_angle_fr_ = wrap<double>(FR_MOTOR_OFFSET,0,2*PI);
  delta_angle_bl_ = wrap<double>(BL_MOTOR_OFFSET,0,2*PI);
  delta_angle_br_ = wrap<double>(BR_MOTOR_OFFSET,0,2*PI);

  theta_fl_ = wrap<double>(FL_MOTOR_OFFSET,0,2*PI);
  theta_fr_ = wrap<double>(FR_MOTOR_OFFSET,0,2*PI);
  theta_bl_ = wrap<double>(BL_MOTOR_OFFSET,0,2*PI);
  theta_br_ = wrap<double>(BR_MOTOR_OFFSET,0,2*PI);


  // Init private variables ends
//   SteerThetaReset();
  // just for wheel speed pid (not for steer motor)
  float* pid_params = new float[3]{12, 3, 1};
  float motor_max_iout = 2000;
  float motor_max_out = 20000;
  for (int i = 0; i < MOTOR_NUM; i++) {
    pids[i].Reinit(pid_params, motor_max_iout, motor_max_out);
  }

  power_limit = new PowerLimit(MOTOR_NUM);
}

EngineerSteeringChassis::~EngineerSteeringChassis() {
  fl_steer_motor = nullptr;
  fr_steer_motor = nullptr;
  bl_steer_motor = nullptr;
  br_steer_motor = nullptr;
  fl_wheel_motor = nullptr;
  fr_wheel_motor = nullptr;
  bl_wheel_motor = nullptr;
  br_wheel_motor = nullptr;
}


 // front -> positive, back -> negative
  void EngineerSteeringChassis::SetXSpeed(float _vx){ vx = _vx; }

  // left -> positive, right -> negative
  void EngineerSteeringChassis::SetYSpeed(float _vy){ vy = _vy; }

  // counterclockwise -> positive
  void EngineerSteeringChassis::SetWSpeed(float _vw){ vw = _vw; }

  void EngineerSteeringChassis::Update(float _power_limit, float _chassis_power, float _chassis_power_buffer){
    // Update Wheels
    float PID_output[MOTOR_NUM];
    float output[MOTOR_NUM];

    // compute PID output
    PID_output[0] = pids[0].ComputeOutput(fl_wheel_motor->GetOmegaDelta(v_fl_));
    PID_output[1] = pids[1].ComputeOutput(fr_wheel_motor->GetOmegaDelta(v_fr_));
    PID_output[2] = pids[2].ComputeOutput(bl_wheel_motor->GetOmegaDelta(v_bl_));
    PID_output[3] = pids[3].ComputeOutput(br_wheel_motor->GetOmegaDelta(v_br_));

    // compute power limit
    power_limit_info.power_limit = _power_limit;
    power_limit_info.WARNING_power = _power_limit * 0.9;
    power_limit_info.WARNING_power_buff = 50;
    power_limit_info.buffer_total_current_limit = 3500 * MOTOR_NUM;
    power_limit_info.power_total_current_limit = 5000 * MOTOR_NUM / 80.0 * _power_limit;
    power_limit->Output(true, power_limit_info, _chassis_power, _chassis_power_buffer, PID_output,
                        output);

    // set final output
    fl_wheel_motor->SetOutput(PID_output[0]);
    fr_wheel_motor->SetOutput(PID_output[1]);
    bl_wheel_motor->SetOutput(PID_output[2]);
    br_wheel_motor->SetOutput(PID_output[3]);
  }

  void EngineerSteeringChassis::SetSpeed(const float _vx, const float _vy, const float _vw){
    vx = _vx;
    vy = _vy;
    vw = _vw;
  }

  void EngineerSteeringChassis::SetWheelSpeed(float _v_fl, float _v_fr, float _v_bl, float _v_br){
    v_fl_ = _v_fl;
    v_fr_ = _v_fr;
    v_bl_ = _v_bl;
    v_br_ = _v_br;
  } 

  bool EngineerSteeringChassis::Calibrate(){
    fl_steer_motor->SetTarget(delta_angle_fl_,true);
    fr_steer_motor->SetTarget(delta_angle_fr_,true);
    bl_steer_motor->SetTarget(delta_angle_bl_,true);
    br_steer_motor->SetTarget(delta_angle_br_,true);

    bool calibrate_done = SteerInPosition();
    if(calibrate_done){
      in_position = true;
      in_position_detector->input(in_position);
    }

    return calibrate_done;

  }

  void EngineerSteeringChassis::SteerCalcOutput(){
    fl_steer_motor->CalcOutput();
    fr_steer_motor->CalcOutput();
    bl_steer_motor->CalcOutput();
    br_steer_motor->CalcOutput();
  }

  void EngineerSteeringChassis::SteerUpdateTarget(){
    if (vx == 0 && vy == 0 && vw == 0) {
     bool ret1 = fl_steer_motor->SetTarget(theta_fl_) == 0 ? false : true;
     bool ret2 = fr_steer_motor->SetTarget(theta_fr_) == 0 ? false : true;
     bool ret3 = bl_steer_motor->SetTarget(theta_bl_) == 0 ? false : true;
     bool ret4 = br_steer_motor->SetTarget(theta_br_) == 0 ? false : true;
     in_position = ret1 && ret2 && ret3 && ret4;
     in_position_detector->input(in_position);
    } else {

    // Compute 2 position proposals, theta and theta + PI.
    theta_fl_ = wrap<double>(atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4))+delta_angle_fl_,-PI,PI);
    theta_fr_ = wrap<double>(atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4))+delta_angle_fr_,-PI,PI);
    theta_bl_ = wrap<double>(atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4))+delta_angle_bl_,-PI,PI);
    theta_br_ = wrap<double>(atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4))+delta_angle_br_,-PI,PI);
    
    //in_position logic might not working here.
    bool ret1 = fl_steer_motor->SetTarget(theta_fl_) == 0 ? false : true;
    bool ret2 = fr_steer_motor->SetTarget(theta_fr_) == 0 ? false : true;
    bool ret3 = bl_steer_motor->SetTarget(theta_bl_) == 0 ? false : true;
    bool ret4 = br_steer_motor->SetTarget(theta_br_) == 0 ? false : true;

    in_position = ret1 && ret2 && ret3 && ret4;
    in_position_detector->input(in_position);

    wheel_dir_fl_ = 1.0;
    wheel_dir_fr_ = 1.0;
    wheel_dir_bl_ = 1.0;
    wheel_dir_br_ = 1.0;
    }
}

  void EngineerSteeringChassis::WheelUpdateSpeed(float wheel_speed_factor){
     // Stay at current position when no command is given
    if (vx == 0 && vy == 0 && vw == 0) {
        SetWheelSpeed(0,0,0,0);
    // TODO: Potential bug, wheel will rotate during turning.
    // } else if (in_position_detector->posEdge()) {
    } else {

        // Wheels move only when all SteeringMotors are in position
        v_fl_ = wheel_dir_fl_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
        v_fr_ = wheel_dir_fr_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
        v_bl_ = wheel_dir_bl_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
        v_br_ = wheel_dir_br_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));

        v_fl_ = v_fl_ * wheel_speed_factor;
        v_fr_ = v_fr_ * wheel_speed_factor;
        v_bl_ = v_bl_ * wheel_speed_factor;
        v_br_ = v_br_ * wheel_speed_factor;
    }
  }

  void EngineerSteeringChassis::SteerSetMaxSpeed(const float max_speed){
    fl_steer_motor->SetMaxSpeed(max_speed);
    fr_steer_motor->SetMaxSpeed(max_speed);
    bl_steer_motor->SetMaxSpeed(max_speed);
    br_steer_motor->SetMaxSpeed(max_speed);
  }

  void EngineerSteeringChassis::PrintData(){
    clear_screen();
    set_cursor(0, 0);

    print("fl_steer_motor: \r\n");
    fl_steer_motor->PrintData();
    osDelay(10);
    print("fr_steer_motor: \r\n");
    fr_steer_motor->PrintData();
    osDelay(10);
    print("bl_steer_motor: \r\n");
    bl_steer_motor->PrintData();
    osDelay(10);
    print("br_steer_motor: \r\n");
    br_steer_motor->PrintData();
  }

  bool EngineerSteeringChassis::SteerInPosition(){
    bool result = fl_steer_motor->inPosition() && 
            fr_steer_motor->inPosition() &&
            bl_steer_motor->inPosition() &&
            br_steer_motor->inPosition();
    in_position = result;
    in_position_detector->input(in_position);
    return in_position;
  }





}