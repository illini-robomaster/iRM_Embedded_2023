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
#include "controller.h"
#include "balance.h"

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

control::ConstrainedPID* balance_controller = new control::ConstrainedPID(1000.0, 0.0, 400.0, 14000.0, 14000.0);
control::ConstrainedPID* velocity_controller = new control::ConstrainedPID(30000.0, 0.0, 8000.0, 14000.0, 14000.0);
control::ConstrainedPID* rotation_controller = new control::ConstrainedPID(3000.0, 0.0, 500.0, 14000.0, 14000.0);
control::ConstrainedPID* left_wheel_vel_pid = new control::ConstrainedPID(2000, 10, 500, 14000.0, 14000.0);
control::ConstrainedPID* right_wheel_vel_pid = new control::ConstrainedPID(2000, 10, 500, 14000.0, 14000.0);
static float balance_output = 0.0;
static float velocity_output = 0.0;
static float rotation_output = 0.0;

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
 float rotation_difference = 0.0;
 current_raw_yaw = imu->INS_angle[0];
 last_raw_yaw = imu->INS_angle[0];
 rotation_angle = current_raw_yaw;

 while (true) {
   // position control
   balance_difference =  balance_angle - imu->INS_angle[2] / PI * 180;
   // velocity control
   velocity_difference = (dbus->ch1/660.0) / 2.0
                         - (-left_wheel_motor->GetOmega() + right_wheel_motor->GetOmega()) * rpm_rads * wheel_radius * 0.5;
   // rotation control
   rotation_angle += -(dbus->ch2/660.0) * 0.3;
   last_raw_yaw = current_raw_yaw;
   current_raw_yaw = imu->INS_angle[0];
   current_yaw = turn_count * 2 * PI + current_raw_yaw;
   if (fabs(current_raw_yaw - last_raw_yaw) > 1.9f * PI)
   {
     if ((current_raw_yaw - last_raw_yaw) < 0)
       turn_count++;
     else
       turn_count--;
   }

   rotation_difference = rotation_angle - current_yaw;

   balance_output = -balance_controller->ComputeOutput(balance_difference);
   velocity_output = velocity_controller->ComputeOutput(velocity_difference);
   rotation_output = rotation_controller->ComputeOutput(rotation_difference);
   if (dbus->swr == remote::DOWN || imu->INS_angle[2] / PI * 180 < balance_angle - 75 || imu->INS_angle[2] / PI * 180 > balance_angle + 75
       || left_wheel_motor->GetCurr() > 16384 || right_wheel_motor->GetCurr() > 16384) {
     left_output = 0;
     right_output = 0;
   } else {
     left_output = balance_output + velocity_output + rotation_output;
     right_output = balance_output + velocity_output - rotation_output;

     left_output = left_wheel_vel_pid->ComputeOutput(left_output * magic_rpm - left_wheel_motor->GetOmega());
     right_output = right_wheel_vel_pid->ComputeOutput(right_output * magic_rpm + right_wheel_motor->GetOmega());

     left_output = clip<float>(left_output, -14000, 14000);
     right_output = clip<float>(right_output, -14000, 14000);
   }
   left_wheel_motor->SetOutput((int16_t)left_output);
   right_wheel_motor->SetOutput(-(int16_t)right_output);

   if (dbus->swl == remote::MID) {
     float vel;
     vel = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15);
     pos += vel / 200;
     pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

     left_front_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);
     left_back_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
     right_front_leg_motor->SetOutput(-pos, vel, 30, 0.5, 0);
     right_back_leg_motor->SetOutput(pos, vel, 30, 0.5, 0);
   } else if (dbus->swl == remote::UP && jump_flag == true) {
     jump_flag = false;
     left_front_leg_motor->SetOutput(jump_height, 10, 30, 0.5, 0);
     left_back_leg_motor->SetOutput(-jump_height, 10, 30, 0.5, 0);
     right_front_leg_motor->SetOutput(-jump_height, 10, 30, 0.5, 0);
     right_back_leg_motor->SetOutput(jump_height, 10, 30, 0.5, 0);
     control::Motor4310::TransmitOutput(leg_motors, 4);
     osDelay(200);
     left_front_leg_motor->SetOutput(pos, 10, 30, 0.5, 0);
     left_back_leg_motor->SetOutput(-pos, 10, 30, 0.5, 0);
     right_front_leg_motor->SetOutput(-pos, 10, 30, 0.5, 0);
     right_back_leg_motor->SetOutput(pos, 10, 30, 0.5, 0);
     control::Motor4310::TransmitOutput(leg_motors, 4);
     osDelay(10);
   }

   if (dbus->swr == remote::UP) {
     left_wheel_motor->SetOutput(0);
     right_wheel_motor->SetOutput(0);
     left_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
     left_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
     right_front_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
     right_back_leg_motor->SetOutput(0.0, 0.0, 0.0, 0.0, 0);
   }

   control::MotorCANBase::TransmitOutput(wheel_motors, 2);
   if (dbus->swl != remote::UP) {
     jump_flag = true;
     control::Motor4310::TransmitOutput(leg_motors, 4);
   }
   osDelay(10);
 }
}
