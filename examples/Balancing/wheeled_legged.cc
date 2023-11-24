/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "bsp_imu.h"
#include "bsp_print.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "motor.h"
#include "dbus.h"
#include "utils.h"
#include "wheeled_legged.h"
#include "controller.h"

#define RX_SIGNAL (1 << 0)
static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* left_wheel_motor = nullptr;
static control::MotorCANBase* right_wheel_motor = nullptr;
static control::Motor4310* left_front_leg_motor = nullptr;
static control::Motor4310* left_back_leg_motor = nullptr;
static control::Motor4310* right_front_leg_motor = nullptr;
static control::Motor4310* right_back_leg_motor = nullptr;
static remote::DBUS* dbus = nullptr;

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      imu->Update();
    }
  }
}

void RM_RTOS_Init(void) {
  //dbus and print initialization
  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart3);

  // can initialization
  can1 = new bsp::CAN(&hcan1, true);

  // wheel motor initialization
  left_wheel_motor = new control::Motor3508(can1, 0x201);
  right_wheel_motor = new control::Motor3508(can1, 0x202);

  // leg motor initialization
  left_front_leg_motor = new control::Motor4310(can1, 0x02, 0x01, control::MIT);
  left_back_leg_motor = new control::Motor4310(can1, 0x04, 0x03, control::MIT);
  right_front_leg_motor = new control::Motor4310(can1, 0x06, 0x05, control::MIT);
  right_back_leg_motor = new control::Motor4310(can1, 0x08, 0x07, control::MIT);

  // imu initialization
  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;
  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;
  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
  imu = new IMU(imu_init, false);
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

//static float balance_pid[3]{600,0,50};
//static float speed_pid[3]{3,0,0};
//static float rotation_pid[3]{200,0,0};
control::ConstrainedPID* balance_controller = new control::ConstrainedPID(1200.0, 0.0, 800.0, 14000.0, 14000.0);
control::ConstrainedPID* velocity_controller = new control::ConstrainedPID(6000.0, 0.0, 1500.0, 14000.0, 14000.0);
control::ConstrainedPID* rotation_controller = new control::ConstrainedPID(1.0, 0.0, 1.0, 14000.0, 14000.0);
static float balance_output = 0.0;
static float velocity_output = 0.0;
static float rotation_output = 0.0;

//static float velocity_lowpass;

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);
  // start switch
  while(true){
    if (dbus->swr == remote::DOWN) {
      break;
    }
  }

  imu->Calibrate();

  control::MotorCANBase* wheel_motors[] = {left_wheel_motor, right_wheel_motor};
  control::Motor4310* leg_motors[] = {left_front_leg_motor, left_back_leg_motor, right_front_leg_motor, right_back_leg_motor};

  while (imu->CaliDone() && imu->DataReady());
  osDelay(10);
  print("Calibration done\r\n");
  osDelay(1000);
  // leg motor activation
//  left_front_leg_motor->SetZeroPos();
  left_front_leg_motor->MotorEnable();
  osDelay(100);
//  left_back_leg_motor->SetZeroPos();
  left_back_leg_motor->MotorEnable();
  osDelay(100);
//  right_front_leg_motor->SetZeroPos();
  right_front_leg_motor->MotorEnable();
  osDelay(100);
//  right_back_leg_motor->SetZeroPos();
  right_back_leg_motor->MotorEnable();

  while (dbus->swl != remote::DOWN);

  left_front_leg_motor->SetRelativeTarget(0);
  left_back_leg_motor->SetRelativeTarget(0);
  right_front_leg_motor->SetRelativeTarget(0);
  right_back_leg_motor->SetRelativeTarget(0);

  for (int j = 0; j < 300; j++){
    left_front_leg_motor->SetRelativeTarget(left_front_leg_motor->GetRelativeTarget() + demo_height / 300);  // increase position gradually
    left_front_leg_motor->SetOutput(left_front_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
    left_back_leg_motor->SetRelativeTarget(left_back_leg_motor->GetRelativeTarget() + demo_height / 300);
    left_back_leg_motor->SetOutput(-left_back_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
    right_front_leg_motor->SetRelativeTarget(right_front_leg_motor->GetRelativeTarget() + demo_height / 300);
    right_front_leg_motor->SetOutput(-right_front_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
    right_back_leg_motor->SetRelativeTarget(right_back_leg_motor->GetRelativeTarget() + demo_height / 300);
    right_back_leg_motor->SetOutput(right_back_leg_motor->GetRelativeTarget(), 1, 115, 0.5, 0);
    control::Motor4310::TransmitOutput(leg_motors, 4);
    osDelay(1);
  }
  float pos = demo_height;
  float min_pos = -PI/4;
  float max_pos = PI/4;

  float balance_angle = -3.2; // measured by IMU
  float left_output = 0.0;
  float right_output = 0.0;
  float balance_difference = 0.0;
  float velocity_difference = 0.0;
//  float rotation_difference = 0.0;


//  float velocity_integral = 0.0;
//  int16_t max_output = 3000;


  while (true) {
    // Update wheel omega and robot velocity
    left_wheel_omega = left_wheel_motor->GetOmega() * rpm_rads;
    right_wheel_omega = right_wheel_motor->GetOmega() * rpm_rads;
    left_wheel_speed = left_wheel_omega * wheel_radius;
    right_wheel_speed = right_wheel_omega * wheel_radius;
    // need to check the sign for each speed
    robot_speed = (left_wheel_speed - right_wheel_speed) / 2;

    // update IMU data(roll, pitch, yaw)
    yaw = imu->INS_angle[0]; // CCW is angle increase
    pitch = imu->INS_angle[2]; // CCW is angle increase(falling down backward is positive)
    roll = imu->INS_angle[1]; // CW is angle increase
    yaw_omega = imu->GetGyro()[0];
    pitch_omega = imu->GetGyro()[2];
    roll_omega = imu->GetGyro()[1];

    // get the input from remote controller
    move_set = dbus->ch1/660.0; // map to speed
    rotate_set = dbus->ch2/660.0; // map to rotation angle
    elevation_speed = dbus->ch3/660.0; // map to elevation angle

    clip<float>(move_set, -1, 1);
    // LQR control
    torque_left_wheel = -(K1* robot_speed*0.015 + K2*(robot_speed - move_set) + K3*pitch + K4*pitch_omega + K15*(yaw-rotate_set) + K16*yaw_omega);
    torque_right_wheel = -(K1* robot_speed*0.015 + K2*(robot_speed - move_set) + K3*pitch + K4*pitch_omega + K25*(yaw-rotate_set) + K26*yaw_omega);
    // clip the torque
//    torque_left_wheel = clip<float>(torque_left_wheel, -max_torque_wheel, max_torque_wheel);
//    torque_right_wheel = clip<float>(torque_right_wheel, -max_torque_wheel, max_torque_wheel);

    // convert to the corresponding current
    current_left_wheel = torque_left_wheel / torque_constant * current_mapping_constant_3508;
    current_right_wheel = torque_right_wheel / torque_constant * current_mapping_constant_3508;
    // set the current
    left_wheel_motor->SetOutput(int16_t(clip<float>(current_left_wheel, -3000, 3000)));
    right_wheel_motor->SetOutput(-int16_t(clip<float>(current_right_wheel, -3000, 3000)));

    // leg motor control
    // Update leg data
    // angle
    left_front_leg_angle = left_front_leg_motor->GetTheta() + leg_angle_offset;
    left_back_leg_angle = left_back_leg_motor->GetTheta() - leg_angle_offset;
    right_front_leg_angle = right_front_leg_motor->GetTheta() - leg_angle_offset;
    right_back_leg_angle = right_back_leg_motor->GetTheta() + leg_angle_offset;

    // height
    left_height = sqrt(pow(l_3,2)-pow(l_1 - l_2* cos(left_front_leg_angle),2)) - l_2*sin(left_front_leg_angle);
    right_height = sqrt(pow(l_3,2)-pow(l_1 - l_2* cos(right_front_leg_angle),2)) - l_2*sin(right_front_leg_angle);
    robot_height = (left_height + right_height) / 2;
    // velocity and acceleration
//    robot_acceleration_z = imu->GetAccel()[2] - gravity_constant;
//    robot_velocity_z = robot_velocity_z + robot_acceleration_z * 0.005;
    // end-effector constant calculation
    left_leg_force_to_torque = -((((l_1 - l_2* cos(left_front_leg_angle))*l_2*sin(left_front_leg_angle))
                                  / sqrt(pow(l_3,2)-pow(l_1 - l_2* cos(left_front_leg_angle),2)))
                                  + l_2*cos(left_front_leg_angle));
    right_leg_force_to_torque = -((((l_1 - l_2* cos(right_front_leg_angle))*l_2*sin(right_front_leg_angle))
                                   / sqrt(pow(l_3,2)-pow(l_1 - l_2* cos(right_front_leg_angle),2)))
                                  + l_2*cos(right_front_leg_angle));

    // VMC control
    left_leg_force = (k_1*(elevation_speed*0.01) - c_1*elevation_speed + mass * 9.81) / 2
                     - (k_2*(0-roll) - c_2*roll_omega) / distance;
    right_leg_force = (k_1*(elevation_speed*0.01) - c_1*elevation_speed + mass * 9.81) / 2
                      + (k_2*(0-roll) - c_2*roll_omega) / distance;

    // get the torque
    left_front_leg_torque = 0.5 * left_leg_force * left_leg_force_to_torque;
    left_back_leg_torque = -0.5 * left_leg_force * left_leg_force_to_torque;
    right_front_leg_torque = -0.5 * right_leg_force * right_leg_force_to_torque;
    right_back_leg_torque = 0.5 * right_leg_force * right_leg_force_to_torque;

    //============================================================================================
    // position control
    balance_difference =  balance_angle - imu->INS_angle[2] / PI * 180;
    // velocity control
//    velocity_difference = (dbus->ch1/660.0)
//                          - (-left_wheel_motor->GetOmega() + right_wheel_motor->GetOmega()) * rpm_rads * wheel_radius * 0.5; // want stop
    velocity_difference = 0 - ((-left_wheel_motor->GetOmega() + right_wheel_motor->GetOmega()) * rpm_rads * wheel_radius * 0.5);
//    velocity_lowpass = 0.3 * velocity_difference + 0.7 * velocity_lowpass;
//    velocity_integral += velocity_lowpass;
//    velocity_integral = clip<float>(velocity_integral, -5000, 5000);
    // rotation control
//    rotation_difference = 0 - imu->GetGyro()[0];

    balance_output = -balance_controller->ComputeOutput(balance_difference);
    velocity_output = velocity_controller->ComputeOutput(velocity_difference);
//    rotation_output = rotation_controller->ComputeOutput(rotation_difference);
    rotation_output = 0;
    print("balance_output: %f, velocity_output: %f, rotation_output: %f\n", balance_output, velocity_output, rotation_output);
    if (dbus->swr == remote::DOWN || imu->INS_angle[2] / PI * 180 < balance_angle - 75 || imu->INS_angle[2] / PI * 180 > balance_angle + 75
        || left_wheel_motor->GetCurr() > 16384 || right_wheel_motor->GetCurr() > 16384) {
      left_output = 0;
      right_output = 0;
//      velocity_integral = 0;
    } else {
      left_output = balance_output + velocity_output + rotation_output;
      right_output = balance_output + velocity_output - rotation_output;
      left_output = clip<float>(left_output, -14000, 14000);
      right_output = clip<float>(right_output, -14000, 14000);
    }
    left_wheel_motor->SetOutput((int16_t)left_output);
    right_wheel_motor->SetOutput(-(int16_t)right_output);

    float vel;
    vel = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15);
    pos += vel / 200;
    pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

    left_front_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);
    left_back_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
    right_front_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
    right_back_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);

//    left_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, left_front_leg_torque);
//    left_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, left_back_leg_torque);
//    right_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, right_front_leg_torque);
//    right_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, right_back_leg_torque);

    if (dbus->swr == remote::UP) {
      left_wheel_motor->SetOutput(0);
      right_wheel_motor->SetOutput(0);
      left_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
      left_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
      right_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
      right_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
    }

    if (dbus->swl != remote::UP) {
      control::MotorCANBase::TransmitOutput(wheel_motors, 2);
    }
    control::Motor4310::TransmitOutput(leg_motors, 4);

    osDelay(10);
  }
}
