/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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
 *  along with this program. If not, see <http://www.gnu.org/licenses/\>.    *
 *                                                                          *
 ****************************************************************************/

/**
 * @brief Omni robot control for DM_MC_02 board
 *
 * Hardware differences from TypeC (main.cc):
 * - Uses FDCAN instead of CAN (hfdcan1)
 * - Print via UART10 (huart10)
 * - DBUS via UART5 (huart5)
 * - IMU via SPI2 (hspi2) with BMI088 only (no IST8310/heater)
 * - IMU initialized in imuTask (HAL_Delay in BMI088 ctor needs RTOS running)
 */

#include <cmath>
#include <memory>

#include "MahonyAHRS.h"
#include "bsp_buzzer.h"
#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "minipc_protocol.h"
#include "motor.h"
#include "spi.h"

#define RX_SIGNAL (1 << 0)

// Convert radians to degrees and vice versa
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

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

const osThreadAttr_t minipcTaskAttribute = {.name = "minipcTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityAboveNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t minipcTaskHandle;

#define MINIPC_RX_SIGNAL (1 << 0)

// IMU data
static bsp::BMI088* bmi088 = nullptr;
static float gyro[3];
static float accel[3];
static float temp;
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float INS_angle[3] = {0.0f, 0.0f, 0.0f};  // yaw, pitch, roll

/**
 * @brief IMU Gyroscope interrupt handler class
 */
class IMU_GYRO_INT : public bsp::GPIT {
 public:
    IMU_GYRO_INT() : GPIT(INT1_GYRO_Pin) {}
    
 protected:
    void IntCallback() override {
        osThreadFlagsSet(imuTaskHandle, RX_SIGNAL);
    }
};

/**
 * @brief IMU Accelerometer interrupt handler class
 */
class IMU_ACCEL_INT : public bsp::GPIT {
 public:
    IMU_ACCEL_INT() : GPIT(INT1_ACCEL_Pin) {}
    
 protected:
    void IntCallback() override {
        // Accelerometer data ready - optional handling
    }
};

static IMU_GYRO_INT* gyro_int = nullptr;
static IMU_ACCEL_INT* accel_int = nullptr;

// Minipc UART class — signals minipcTask on RX complete
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() override final { osThreadFlagsSet(minipcTaskHandle, MINIPC_RX_SIGNAL); }
};

// Handshake protocol markers (see docs/HANDSHAKE_IMPLEMENTATION.md)
#define HANDSHAKE_REQUEST 0xFF  // Jetson -> MCU
#define HANDSHAKE_ACK 0xFE      // MCU -> Jetson

// Shared data between minipc thread and default task
static volatile float jetson_rel_yaw = 0.0f;
static volatile float jetson_rel_pitch = 0.0f;
static volatile uint8_t jetson_mode = 0;           // 0=ST, 1=MY
static volatile bool jetson_data_ready = false;    // consumed flag
static volatile bool jetson_handshake_ok = false;  // true after handshake exchange
/*
// Gimbal feedback: written by default task, read by minipcTask for TX to Jetson
static volatile float feedback_yaw = 0.0f;    // current gimbal yaw (field-referenced)
static volatile float feedback_pitch = 0.0f;  // current pitch encoder position
static volatile uint8_t feedback_mode = 0;    // mirrors jetson_mode for echo
*/
// Jetson chassis control: written by minipcTask, read by default task
static volatile float jetson_vx = 0.0f;             // forward velocity from Jetson
static volatile float jetson_vy = 0.0f;             // leftward velocity from Jetson
static volatile float jetson_vw = 0.0f;             // angular velocity from Jetson
static volatile bool jetson_chassis_ready = false;  // new chassis command available

// How quickly yaw_target / pitch_target ramp toward the Jetson-derived desired
// position each 200 Hz tick.  A value of 1.0 snaps immediately (causes D kicks);
// 0.3 gives ~12 ms rise time, which is well within the 33 ms Jetson packet period
// and keeps target changes smooth so the inner PID D term stays quiet.
// Increase for faster response, decrease if the gimbal still shakes.
static const float AUTOAIM_YAW_SMOOTH = 0.16f;
static const float AUTOAIM_PITCH_SMOOTH = 0.02f;

// Buzzer for handshake notification (TIM12_CH2 on PB15, MC02 APB1 timer clock = 80 MHz, prescaler = 24)
#define BUZZER_CLOCK_FREQ (80000000 / 24)
static bsp::Buzzer* buzzer = nullptr;

using Note = bsp::BuzzerNote;

// Short ascending chime to signal successful Jetson handshake
static const bsp::BuzzerNoteDelayed handshake_melody[] = {
    {Note::Do1M, 120},
    {Note::Mi3M, 120},
    {Note::So5M, 120},
    {Note::Do1H, 250},
    {Note::Silent, 0},
    {Note::Finish, 0},
};

void imuTask(void* arg) {
  UNUSED(arg);

  // Initialize BMI088 here (after RTOS scheduler starts) so HAL_Delay() works
  bmi088 = new bsp::BMI088(&hspi2,
                           CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin,
                           CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);

  // Create interrupt handlers after BMI088 is ready
  gyro_int = new IMU_GYRO_INT();
  accel_int = new IMU_ACCEL_INT();

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      bmi088->Read(gyro, accel, &temp);

      float gx_rad = DEG2RAD(gyro[0]);
      float gy_rad = DEG2RAD(gyro[1]);
      float gz_rad = DEG2RAD(gyro[2]);

      MahonyAHRSupdateIMU(quat, gx_rad, gy_rad, gz_rad, 
                         accel[0], accel[1], accel[2]);

      INS_angle[0] = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
                           1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));  // yaw
      INS_angle[1] = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));          // pitch
      INS_angle[2] = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                           1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));  // roll
    }
  }
}

void minipcTask(void* arg) {
  UNUSED(arg);

  auto uart = std::make_unique<CustomUART>(&huart7);
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();

  //communication::color_data_t color_data;
  //color_data.my_color = 0;  // RED=0 default; updated when Jetson sends COLOR_CMD_ID

  const communication::status_data_t* status_data;
  //uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
  uint8_t* data;
  int32_t length;
  //bool handshake_done = false;  // true after handshake exchange completed

  // Gimbal feedback timing: send at ~100Hz (every 10ms)
  // Guide: "Send feedback at 50-100 Hz (every 10-20ms)"
  const uint32_t FEEDBACK_INTERVAL_MS = 10;
  //uint32_t last_feedback_tick = 0;

  while (true) {
    // Use 10ms timeout so we can send periodic feedback even without RX
    uint32_t flags = osThreadFlagsWait(MINIPC_RX_SIGNAL, osFlagsWaitAll, FEEDBACK_INTERVAL_MS);

    // --- Process incoming packet if RX signal received ---
    if ((flags & MINIPC_RX_SIGNAL) && !(flags & osFlagsError)) {
      length = uart->Read(&data);
      minipc_session.ParseUartBuffer(data, length);

      if (minipc_session.GetValidFlag()) {
        uint8_t recv_cmd_id = minipc_session.GetCmdId();
        status_data = minipc_session.GetStatus();

        // ---- Handshake protocol ----
        /*
        // Jetson sends GIMBAL packet with debug_int=0xFF to request handshake.
        // MCU replies with GIMBAL packet with debug_int=0xFE as ACK.
        if (recv_cmd_id == communication::GIMBAL_CMD_ID &&
            status_data->debug_int == HANDSHAKE_REQUEST) {
          // Build ACK with current gimbal position
          communication::gimbal_data_t ack;
          ack.rel_yaw = feedback_yaw;
          ack.rel_pitch = feedback_pitch;
          ack.mode = 0;  // ST
          ack.debug_int = HANDSHAKE_ACK;
          minipc_session.Pack(packet_to_send, (void*)&ack, communication::GIMBAL_CMD_ID);
          uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::GIMBAL_CMD_ID));

          if (!handshake_done) {
            handshake_done = true;
            jetson_handshake_ok = true;
            print("Jetson handshake OK (0xFF -> 0xFE)\r\n");
            if (buzzer != nullptr) {
              buzzer->SingSong(handshake_melody, [](uint32_t ms) { osDelay(ms); });
            }
          }
          // Do NOT treat handshake packets as autoaim data
          // Fall through to feedback sending below
        }
        // ---- Normal gimbal commands ----
        else if (recv_cmd_id == communication::GIMBAL_CMD_ID) {
          jetson_rel_yaw = status_data->rel_yaw;
          jetson_rel_pitch = status_data->rel_pitch;
          jetson_mode = status_data->mode;
          jetson_data_ready = true;
        }
          
        // ---- Color update from Jetson ----
        if (recv_cmd_id == communication::COLOR_CMD_ID) {
          color_data.my_color = status_data->my_color;
        }
          */
        // ---- Chassis control from Jetson ----
        if (recv_cmd_id == communication::CHASSIS_CMD_ID) {
          jetson_vx = status_data->vx;
          jetson_vy = status_data->vy;
          jetson_vw = status_data->vw;
          jetson_chassis_ready = true;
        }
      }
    }

    // --- Send periodic gimbal feedback + color after handshake ---
    /*
    if (handshake_done) {
      uint32_t now = HAL_GetTick();
      if (now - last_feedback_tick >= FEEDBACK_INTERVAL_MS) {
        last_feedback_tick = now;

        // Send GIMBAL feedback with current position
        communication::gimbal_data_t fb;
        fb.rel_yaw = feedback_yaw;
        fb.rel_pitch = feedback_pitch;
        fb.mode = feedback_mode;
        fb.debug_int = 0;  // normal operation
        minipc_session.Pack(packet_to_send, (void*)&fb, communication::GIMBAL_CMD_ID);
        uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::GIMBAL_CMD_ID));

        // Also send color data
        minipc_session.Pack(packet_to_send, (void*)&color_data, communication::COLOR_CMD_ID);
        uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::COLOR_CMD_ID));
      }
    }
      */
  }
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  minipcTaskHandle = osThreadNew(minipcTask, nullptr, &minipcTaskAttribute);
}

// Global peripherals
bsp::CAN* can = nullptr;
control::MotorDM3519* motor[4];
control::Motor3508* flywheel_motor[2];
control::Motor3508* feeder_motor;
control::Motor6020* yaw_motor;
control::Motor4310* pitch_motor;
remote::DBUS* dbus = nullptr;

float original_max_output_vel_rad_per_s = 395/60.0f*2*PI; //395 rpm = 41.36 rad/s
float original_gear_ratio = 3591.0f / 187.0f;
float new_gear_ratio = 268.0f / 17.0f;
const float YAW_MOTOR_OFFSET = 4.54f; // in rad
const float PITCH_ENCODER_MAX = 0.0f;                     // encoder 0° = upper limit (facing up)
const float PITCH_ENCODER_MIN = -23.77f * PI / 180.0f;    // encoder -23.77° = lower limit (facing down)
const float PITCH_INITIAL_TARGET = -10.0f * PI / 180.0f;  // initial pitch target in encoder coords
const float PITCH_INIT_RATE = 0.3f;                       // rad/s ramp speed toward PITCH_INITIAL_TARGET on enable

void RM_RTOS_Init() {
  print_use_uart(&huart10);

  // MC02 uses FDCAN instead of CAN
  can = new bsp::CAN(&hfdcan1, 0);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor[0] = new control::MotorDM3519(can, 0x10, 0x11, control::VEL); // front right
  motor[1] = new control::MotorDM3519(can, 0x12, 0x13, control::VEL); // back left
  motor[2] = new control::MotorDM3519(can, 0x18, 0x19, control::VEL); // back right
  motor[3] = new control::MotorDM3519(can, 0x16, 0x17, control::VEL);  // front left
  yaw_motor = new control::Motor6020(can, 0x205);
  pitch_motor = new control::Motor4310(can, 0x0D, 0x0C, control::MIT);

  // MC02 uses UART5 for DBUS
  dbus = new remote::DBUS(&huart5);

  // Buzzer on TIM12_CH2 (PB15)
  buzzer = new bsp::Buzzer(&htim12, 2, BUZZER_CLOCK_FREQ);

  // Shooter motors
  flywheel_motor[0] = new control::Motor3508(can, 0x207); // left flywheel
  flywheel_motor[1] = new control::Motor3508(can, 0x208); // right flywheel
  feeder_motor = new control::Motor3508(can, 0x206);

  // IMU initialization is in imuTask() - HAL_Delay() in BMI088 constructor
  // hangs when called before the RTOS scheduler starts
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::MotorDM3519* motors[] = {motor[0], motor[1], motor[2], motor[3]};
  control::Motor4310* pitch_motors[] = {pitch_motor};
  control::MotorCANBase* dji_motors[] = {yaw_motor, feeder_motor, flywheel_motor[0], flywheel_motor[1]};
  control::PIDController pid_(40000.0, 0.0, 300000.0);

  control::PIDController left_flywheel_pid_(40.0, 15.0, 30.0);
  control::PIDController right_flywheel_pid_(40.0, 15.0, 30.0);
  control::PIDController feeder_pid_(40.0, 5.0, 10.0);

  while (dbus->swr == remote::DOWN);

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  osDelay(1000);
  motor[0]->SetZeroPos();
  motor[0]->MotorEnable();
  motor[1]->SetZeroPos();
  motor[1]->MotorEnable();
  motor[2]->SetZeroPos();
  motor[2]->MotorEnable();
  motor[3]->SetZeroPos();
  motor[3]->MotorEnable();
  // pitch_motor->SetZeroPos();  // pitch motor zero pos, comment if calibrated already
  pitch_motor->MotorEnable();
  float pitch_target = pitch_motor->GetTheta();  // ramp from current; do NOT snap to PITCH_INITIAL_TARGET
  bool pitch_init_done = false;
  osDelay(100);

  bool enabled = false;
  bool flywheel_enabled = false;
  float yaw_target = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;
  int16_t yaw_feedback;
  float gimbal_yaw_measured_in_field_reference = 0.0f;

  float left_target_velocity = 0;
  float right_target_velocity = 0;
  float feeder_target_velocity = 0;

  while (true) {
    // set_cursor(0, 0);
    // clear_screen();

    // disable logic
    if(dbus->swr == remote::DOWN){
      if(enabled){
        motor[0]->MotorDisable();
        motor[1]->MotorDisable();
        motor[2]->MotorDisable();
        motor[3]->MotorDisable();

        // disable gimbal motors
        pitch_motor->MotorDisable();
        pitch_target = pitch_motor->GetTheta();  // ramp from current on next enable
        pitch_init_done = false;
        enabled = false;

        // disable flywheel
        left_target_velocity = 0;
        right_target_velocity = 0;
        feeder_target_velocity = 0;
        print("Disabled \r\n");
      }
      osDelay(100);
      continue;
    } else {
      if(!enabled){
        print("Enabled \r\n");
        motor[0]->MotorEnable();
        motor[1]->MotorEnable();
        motor[2]->MotorEnable();
        motor[3]->MotorEnable();
        pitch_motor->MotorEnable();
        pitch_target = pitch_motor->GetTheta();  // ramp from current on re-enable
        pitch_init_done = false;
        osDelay(100);
        enabled = true;
      }

      if(dbus->swr == remote::UP){
        if (!flywheel_enabled) {  // enable flywheel if right switch is UP
          left_target_velocity = -900;
          right_target_velocity = 900;
          flywheel_enabled = true;
        }
      } else {                   // MID
        if (flywheel_enabled) {  // disable flywheel if right switch is MID
          flywheel_enabled = false;
        }
      }
    }

    // Gradually ramp down flywheel target velocity when disabled
    if (!flywheel_enabled) {
      const float ramp_step = 1.0f;  // decrease per loop iteration
      if (left_target_velocity < -ramp_step)
        left_target_velocity += ramp_step;
      else if (left_target_velocity > ramp_step)
        left_target_velocity -= ramp_step;
      else
        left_target_velocity = 0;

      if (right_target_velocity < -ramp_step)
        right_target_velocity += ramp_step;
      else if (right_target_velocity > ramp_step)
        right_target_velocity -= ramp_step;
      else
        right_target_velocity = 0;
    }

    if(dbus->swl == remote::DOWN){
      feeder_target_velocity = 80.0f;
    } else {
      feeder_target_velocity = 0.0f;
    }

    // shooter
    float left_diff = flywheel_motor[0]->GetOmegaDelta(left_target_velocity);
    float right_diff = flywheel_motor[1]->GetOmegaDelta(right_target_velocity);
    float feeder_diff = feeder_motor->GetOmegaDelta(feeder_target_velocity);
    int16_t left_output = left_flywheel_pid_.ComputeConstrainedOutput(left_diff);
    int16_t right_output = right_flywheel_pid_.ComputeConstrainedOutput(right_diff);
    int16_t feeder_output = feeder_pid_.ComputeConstrainedOutput(feeder_diff);

    flywheel_motor[0]->SetOutput(left_output);
    flywheel_motor[1]->SetOutput(right_output);
    feeder_motor->SetOutput(feeder_output);

    // chassis
    gimbal_yaw_measured_in_field_reference = INS_angle[0] + yaw_motor->GetTheta() - YAW_MOTOR_OFFSET;

    float vel[4];

    float y = -clip<float>(dbus->ch0 / 660.0 * 30.0, -30, 30); // forward
    float x = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30); // left
    float yaw_omega = clip<float>(-dbus->ch2 / 660.0 * 10.0, -10, 10);               // yaw (slower)
    float pitch_omega = clip<float>(dbus->ch3 / 660.0 * 5.0, -5, 5);                 // pitch (slower)

    // Closed-loop autoaim: the Jetson sends angular offsets (rel_yaw, rel_pitch)
    // at ~30 Hz, but the control loop runs at 200 Hz.  Snapping yaw_target to a
    // new absolute position every 30 Hz packet causes large D-term kicks in the
    // inner PID (Kd=300000).  Instead:
    //   1. On each incoming packet, store the desired absolute target position.
    //   2. Every 200 Hz tick, smoothly ramp yaw_target toward that desired value.
    // This removes step discontinuities and keeps the D term quiet between packets.
    static float desired_yaw = 0.0f;
    static float desired_pitch = 0.0f;
    static bool autoaim_active = false;

    if (jetson_mode == 1) {
      if (!autoaim_active) {
        // First tick entering autoaim: seed desired at current position.
        desired_yaw = gimbal_yaw_measured_in_field_reference;
        desired_pitch = pitch_motor->GetTheta();
        yaw_target = desired_yaw;
        pitch_target = desired_pitch;
        autoaim_active = true;
      }
      if (jetson_data_ready) {
        // New packet: update desired absolute target from current position + offset.
        desired_yaw = gimbal_yaw_measured_in_field_reference + (float)jetson_rel_yaw;
        desired_pitch = pitch_motor->GetTheta() - (float)jetson_rel_pitch;
        jetson_data_ready = false;
      }
      // Every 200 Hz tick: ramp toward desired — no sudden steps, no D kicks.
      yaw_target += AUTOAIM_YAW_SMOOTH * (desired_yaw - yaw_target);
      if (pitch_init_done)
        pitch_target += AUTOAIM_PITCH_SMOOTH * (desired_pitch - pitch_target);
    } else {
      if (autoaim_active) {
        autoaim_active = false;
      }
      // Manual DBUS control (ST mode or no data — operator searches for targets)
      yaw_target += yaw_omega * 0.005f;
      if (pitch_init_done)
        pitch_target += pitch_omega * 0.005f;
      if (jetson_data_ready) jetson_data_ready = false;
    }

    // Startup pitch ramp: move pitch_target to PITCH_INITIAL_TARGET at PITCH_INIT_RATE
    // before handing off to DBUS / autoaim.  Overrides whatever the blocks above set.
    if (!pitch_init_done) {
      const float step = PITCH_INIT_RATE * 0.005f;
      const float diff = PITCH_INITIAL_TARGET - pitch_target;
      if (fabsf(diff) <= step) {
        pitch_target = PITCH_INITIAL_TARGET;
        pitch_init_done = true;
      } else {
        pitch_target += (diff > 0.0f ? step : -step);
      }
    }

    pitch_target = clip<float>(pitch_target, PITCH_ENCODER_MIN, PITCH_ENCODER_MAX);  // clamp to encoder limits

    float delta_yaw = yaw_target - gimbal_yaw_measured_in_field_reference;
    float cos = cosf(delta_yaw);
    float sin = sinf(delta_yaw);
    delta_yaw = atan2f(sin, cos);  // wrap to [-pi, pi]
    yaw_feedback = pid_.ComputeConstrainedOutput(delta_yaw);

    float chasis_yaw_measured_in_field_reference = gimbal_yaw_measured_in_field_reference - INS_angle[0];  // in rad

    // rotate the x,y according to the robot's heading
    float temp_x = x * cosf(chasis_yaw_measured_in_field_reference) - y * sinf(chasis_yaw_measured_in_field_reference);
    float temp_y = x * sinf(chasis_yaw_measured_in_field_reference) + y * cosf(chasis_yaw_measured_in_field_reference);
    x = temp_x;
    y = temp_y;

    // alpha, beta equals to x y rotated by -45 degrees
    float alpha = (x - y) / 1.4142f;
    float beta = (y + x) / 1.4142f;

    // max output for omega while ensure translation
    float max_omega = abs(30.0f - max(fabsf(alpha), fabsf(beta)));
    float chassis_yaw_omega_target = dbus->swl == remote::UP ? 15.0f : 0.0f;
    chassis_yaw_omega_target = clip<float>(chassis_yaw_omega_target, -max_omega, max_omega);

    vel[0] = -beta + chassis_yaw_omega_target;
    vel[1] = beta + chassis_yaw_omega_target;
    vel[2] = -alpha + chassis_yaw_omega_target;
    vel[3] = alpha + chassis_yaw_omega_target;

    UNUSED(yaw_feedback);
    motor[0]->SetOutput(vel[0]);
    motor[1]->SetOutput(vel[1]);
    motor[2]->SetOutput(vel[2]);
    motor[3]->SetOutput(vel[3]);
    float yaw_kF = 400.0f;
    int16_t yaw_output = yaw_feedback + chassis_yaw_omega_target * yaw_kF;
    yaw_motor->SetOutput(yaw_output);

    // yaw motor tuning
    // print("err: %.2f m_o : %d \r\n", delta_yaw, yaw_output);
    // print("pitch theta: %.2f deg, pitch target: %.2f deg \r\n", RAD2DEG(pitch_motor->GetTheta()), RAD2DEG(pitch_target));
    if (HAL_GetTick() % 1000 < 5) {  // print every ~1s
      print("y_s=%.2f p_s=%.2f  err_yaw=%.3f err_pitch=%.3f mode=%d\r\n",
            AUTOAIM_YAW_SMOOTH, AUTOAIM_PITCH_SMOOTH, jetson_rel_yaw, jetson_rel_pitch, jetson_mode);
    }

    pitch_motor->SetOutput(pitch_target, pitch_omega, 30, 0.5, 0);
    control::MotorDM3519::TransmitOutput(motors, 4);
    control::Motor4310::TransmitOutput(pitch_motors, 1);
    control::Motor3508::TransmitOutput(dji_motors, 4);

    // Update gimbal feedback for minipcTask to send to Jetson
    /*
    feedback_yaw = gimbal_yaw_measured_in_field_reference;
    feedback_pitch = pitch_motor->GetTheta();
    feedback_mode = jetson_mode;
    */

    osDelay(5);
  }
}
