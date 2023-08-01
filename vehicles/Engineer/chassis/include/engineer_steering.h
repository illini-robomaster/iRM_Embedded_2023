#pragma once

#include "motor.h"
#include "engineer_steering.h"
#include "power_limit.h"
#include "cmsis_os.h"
#include "steering_6020.h"

namespace control {

constexpr uint16_t MOTOR_NUM = 4;

typedef struct {
  control::ServoMotor* fl_steer_motor = nullptr;
  control::ServoMotor* fr_steer_motor = nullptr;
  control::ServoMotor* bl_steer_motor = nullptr;
  control::ServoMotor* br_steer_motor = nullptr;

  control::MotorCANBase* fl_steer_motor_raw = nullptr;
  control::MotorCANBase* fr_steer_motor_raw = nullptr;
  control::MotorCANBase* bl_steer_motor_raw = nullptr;
  control::MotorCANBase* br_steer_motor_raw = nullptr;

  control::MotorCANBase* fl_wheel_motor = nullptr;
  control::MotorCANBase* fr_wheel_motor = nullptr;
  control::MotorCANBase* bl_wheel_motor = nullptr;
  control::MotorCANBase* br_wheel_motor = nullptr;
} engineer_steering_chassis_t;


class EngineerSteeringChassis{
public:
    EngineerSteeringChassis(engineer_steering_chassis_t* chassis);

    ~EngineerSteeringChassis();

  // front -> positive, back -> negative
  void SetXSpeed(float _vx);

  // left -> positive, right -> negative
  void SetYSpeed(float _vy);

  // counterclockwise -> positive
  void SetWSpeed(float _vw);

  void Update(float power_limit, float chassis_power, float chassis_power_buffer);

  /**
   * @brief set the speed for chassis steer motors
   *
   * @param x_speed: chassis speed on x-direction, positive = front
   * @param y_speed: chassis speed on y-direction, positive = left
   * @param turn_speed: chassis counterclockwise turning speed
   */
  void SetSpeed(const float _vx, const float _vy, const float _vw);

  /**
   * @brief reset theta to 0
   * use after ReAlign()
   */
  void SteerThetaReset();

  /**
   * @brief set the speed for chassis wheel motors
   * Only change values, need to call WheelSetOutput()
   */
  void SetWheelSpeed(float v_fl, float v_fr, float v_bl, float v_br);

  /**
   * @brief find the aligned position
   *        If the motors don't have an aligned position, the motors rotate until their detectors return True.
   *        If the motors have one, this function does nothing and return True.
   * @note  Calibrate() only find the aligned position, it doesn't guarantee the motor ends in the aligned position or stops
   *
   *
   * @return True only when all four motors have a aligned position
   */
  bool Calibrate();

  /**
   * @brief Call ReAlign() for all four SteeringMotor
   *        Turn the motor to the aligned position
   *        Do nothing if the motor doesn't have an aligned position
   * @return sum of all returned values
   *         0 if success
   */
  int ReAlign();

  /**
   * @brief Call SteeringMotor::CalcOutput() for all 4 Steering Motors
   */
  void SteerCalcOutput();

  /**
   * @brief Compute theta for 4 SteerMotors
   *        Set Target for Steer Motors
   *        When vx, vy, and vw are all 0s, stay at current position
   * @note  Steer motors will turn as little as possible to reach the desired configuration
   *        The idea is compute 2 potential position and choose the closer one
   */
  void SteerUpdateTarget();

  /**
   * @brief Compute speed for 4 WheelMotors
   *        Set Target for Steer Motors
   *        When vx, vy, and vw are all 0s, speed = 0
   * @note  Only update member variables, need to SetOutput()
   */
  void WheelUpdateSpeed(float wheel_speed_factor);

  /**
   * @brief Call SetMaxSpeed() for all 4 Steering Motor
   */
  void SteerSetMaxSpeed(const float max_speed);

  /**
   * @brief Call PrintData() for all 4 Steering Motor
   * The order is FrontLeft, FrontRight, BackLeft, BackRight
   * It prints current target, current position, and align position
   */
  void PrintData();

  /**
   * @brief Set align_complete_ of all 4 Steering Motor to False
   */
  void SteerAlignFalse();

  /**
   * @brief Check align_complete_ flag for all 4 Steering Motors
   * @return True if all 4 align_complete_ flags are true; false otherwise
   */
  bool SteerAlignCheck();

  // speed of four wheels
  float v_fl_;
  float v_fr_;
  float v_bl_;
  float v_br_;

 private:
  control::ServoMotor* fl_steer_motor;
  control::ServoMotor* fr_steer_motor;
  control::ServoMotor* bl_steer_motor;
  control::ServoMotor* br_steer_motor;

  control::MotorCANBase* fl_steer_motor_raw;
  control::MotorCANBase* fr_steer_motor_raw;
  control::MotorCANBase* bl_steer_motor_raw;
  control::MotorCANBase* br_steer_motor_raw;


  control::MotorCANBase* fl_wheel_motor;
  control::MotorCANBase* fr_wheel_motor;
  control::MotorCANBase* bl_wheel_motor;
  control::MotorCANBase* br_wheel_motor;

  // current velocity of the entire chassis
  // left -> positive, right -> negative
  // front -> positive, back -> negative
  // counterclockwise -> positive
  float vx;
  float vy;
  float vw;

  //angle offset for 6020
  double delta_angle_fl_;
  double delta_angle_fr_;
  double delta_angle_bl_;
  double delta_angle_br_;

  // current angle of 4 steers motors
  double theta_fl_;
  double theta_fr_;
  double theta_bl_;
  double theta_br_;

  float wheel_dir_fl_;
  float wheel_dir_fr_;
  float wheel_dir_bl_;
  float wheel_dir_br_;

  // ret values of TurnRelative(), used to sync steer and wheels
  // ret_**_ == 0 means the corresponding steer motor is in position so the wheels can turn
  int ret_fl_;
  int ret_fr_;
  int ret_bl_;
  int ret_br_;

  // same as class Chassis
  ConstrainedPID pids[4];
  PowerLimit* power_limit;
  power_limit_t power_limit_info;

};  // class SteeringChassis ends

}  // namespace control