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
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "bsp_buzzer.h"
#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "weighscale.h"

#define LEFT_MOTOR_PWM_CHANNEL 1
#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50
#define IDLE_THROTTLE 1500
#define DEFAULT_TASK_DELAY 100

#define KEY_GPIO_GROUP K1_GPIO_Port
#define KEY_GPIO_PIN K1_Pin

#define MAX_IOUT 16384
#define MAX_OUT 60000

#define MOTOR_TEMP_HIGH_THRESHOLD 80
#define MOTOR_TEMP_LOW_THRESHOLD 40
#define ALARM_INTERVAL 100

#define MAP_RANGE(x, in_min, in_max, out_min, out_max) \
  (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / \
  ((float)(in_max) - (float)(in_min)) + (float)(out_min))

// Loading state machine
enum class LoadState { IDLE,
                       LOADING_DOWN,
                       LOADED,
                       REVERSING };
enum class LoadControlMode { AUTO_RELOAD,
                             MANUAL };

#define TRIGGER_HOLD_OUTPUT 600       // PWM offset to hold dart
#define TRIGGER_RELEASE_OUTPUT 0      // PWM offset to release
#define TRIGGER_MID_OUTPUT 300        // PWM offset for manual mid state
#define LOAD_DOWN_SPEED (-150.0f)     // rad/s downward
#define REVERSE_SPEED 150.0f          // rad/s reverse (slower)
#define REVERSE_RELEASE_CURRENT 4000  // |current| below this → dart released
#define MANUAL_LOAD_FORWARD_SPEED 300.0f
#define MANUAL_LOAD_REVERSE_SPEED (-270.0f)
#define LOAD_MODE_SWITCH_THRESHOLD 500

// Peripherals
bsp::GPIO* key = nullptr;
bsp::GPIO* bump_switch = nullptr;
bsp::Buzzer* buzzer = nullptr;
bsp::Laser* laser = nullptr;

// Alarm sound for overheating
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed AlarmSound[] = {
    {Note::Do1H, 100}, {Note::Silent, 50}, {Note::Do1H, 100}, {Note::Silent, 50},
    {Note::Do1H, 100}, {Note::Silent, 200}, {Note::Silent, 0}, {Note::Finish, 0}};
static bsp::BuzzerNoteDelayed AutoModeSwitchSound[] = {
    {Note::So5M, 60}, {Note::Silent, 30}, {Note::Do1H, 80}, {Note::Silent, 0}, {Note::Finish, 0}};
static bsp::BuzzerNoteDelayed ManualModeSwitchSound[] = {
    {Note::Do1H, 60}, {Note::Silent, 30}, {Note::So5M, 80}, {Note::Silent, 0}, {Note::Finish, 0}};

// Motors
control::MotorPWMBase* trigger_motor = nullptr;
control::MotorCANBase* load_motor_1 = nullptr;
control::MotorCANBase* load_motor_2 = nullptr;
control::MotorCANBase* force_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

// Communication
static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;

// WeighScale
static control::WeighScale* scale = nullptr;
static const uint8_t NUM_CHANNELS = 4;

// State variables (disabled for now)
static uint16_t load_motor_temperature = 0;
static bool motor_cooling_state = false;
static uint32_t alarm_counter = 0;

// PID parameters
float Kp_load = 50;
float Ki_load = 15;
float Kd_load = 65;

osThreadId_t dartLoadTaskHandle;
const osThreadAttr_t dartLoadTaskAttribute = {.name = "dartLoadTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

void dartLoadTask(void* arg) {
  UNUSED(arg);

  float param[] = {Kp_load, Ki_load, Kd_load};
  control::PIDController pid_yaw(50, 5, 10);
  control::ConstrainedPID pid_left(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_right(param, MAX_IOUT, MAX_OUT);
  control::ConstrainedPID pid_force(param, MAX_IOUT, MAX_OUT);

  control::MotorCANBase* motors_can1_load[] = {load_motor_1, load_motor_2, force_motor};
  control::MotorCANBase* yaw_motors[] = {yaw_motor};

  float load_target_speed = 0;
  float force_target_speed = 0;
  float yaw_target_speed = 0;
  LoadControlMode load_control_mode = LoadControlMode::AUTO_RELOAD;
  LoadState load_state = LoadState::IDLE;
  uint32_t reverse_debounce = 0;
  BoolEdgeDetector load_trigger(false);
  BoolEdgeDetector reverse_trigger(false);
  BoolEdgeDetector release_trigger(false);
  BoolEdgeDetector load_mode_switch(false);

  trigger_motor->SetOutput(TRIGGER_RELEASE_OUTPUT);
  while (true) {
    // ---- Temperature protection ----
    load_motor_temperature = load_motor_1->GetTemp();
    if (load_motor_temperature > MOTOR_TEMP_HIGH_THRESHOLD && !motor_cooling_state) {
      motor_cooling_state = true;
      print("Motor overheated! Temperature: %d C\r\n", load_motor_temperature);
    } else if (load_motor_temperature < MOTOR_TEMP_LOW_THRESHOLD && motor_cooling_state) {
      motor_cooling_state = false;
      print("Motor cooled down! Temperature: %d C\r\n", load_motor_temperature);
    }
    if (motor_cooling_state) {
      alarm_counter++;
      if (alarm_counter >= ALARM_INTERVAL) {
        buzzer->SingSong(AlarmSound);
        alarm_counter = 0;
      }
    } else {
      alarm_counter = 0;
    }

    // ---- Load mode switching ----
    load_mode_switch.input(dbus->ch0 > LOAD_MODE_SWITCH_THRESHOLD);
    if (load_mode_switch.posEdge()) {
      if (load_control_mode == LoadControlMode::AUTO_RELOAD) {
        load_control_mode = LoadControlMode::MANUAL;
        load_state = LoadState::IDLE;
        reverse_debounce = 0;
        buzzer->SingSong(ManualModeSwitchSound, [](uint32_t milli) { osDelay(milli); });
        print(">>> Load mode switched to MANUAL\r\n");
      } else {
        load_control_mode = LoadControlMode::AUTO_RELOAD;
        load_state = LoadState::IDLE;
        reverse_debounce = 0;
        trigger_motor->SetOutput(TRIGGER_RELEASE_OUTPUT);
        buzzer->SingSong(AutoModeSwitchSound, [](uint32_t milli) { osDelay(milli); });
        print(">>> Load mode switched to AUTO_RELOAD\r\n");
      }
    }

    if (load_control_mode == LoadControlMode::AUTO_RELOAD) {
      // ---- Automatic load state machine ----
      // Bump switch: reads 0 when hit (active low), 1 otherwise
      bool bump_hit = !bump_switch->Read();
      load_trigger.input(dbus->swl == remote::UP);
      reverse_trigger.input(dbus->swl == remote::DOWN);
      release_trigger.input(dbus->swr == remote::UP);
      switch (load_state) {
        case LoadState::IDLE:
          if (release_trigger.posEdge())
            trigger_motor->SetOutput(TRIGGER_RELEASE_OUTPUT);
          load_target_speed = 0;
          if (load_trigger.posEdge()) {
            load_state = LoadState::LOADING_DOWN;
            print(">>> Load: IDLE -> LOADING_DOWN\r\n");
          }
          break;
        // TODO: This will be the place holder for extra loading mechanism for getting the dart out of the storing catridges
        case LoadState::LOADING_DOWN:
          if (bump_hit) {
            // Bump switch hit — dart is seated; hold trigger and stop descent
            trigger_motor->SetOutput(TRIGGER_HOLD_OUTPUT);
            load_target_speed = 0;
            load_state = LoadState::LOADED;
            print(">>> Load: LOADING_DOWN -> LOADED\r\n");
          } else {
            load_target_speed = LOAD_DOWN_SPEED;
          }
          break;

        case LoadState::LOADED:
          trigger_motor->SetOutput(TRIGGER_HOLD_OUTPUT);
          load_target_speed = 0;
          if (reverse_trigger.posEdge()) {
            reverse_debounce = 0;
            load_state = LoadState::REVERSING;
            print(">>> Load: LOADED -> REVERSING\r\n");
          }
          break;

        case LoadState::REVERSING:
          // Velocity loop at slower speed; monitor current for dart release
          load_target_speed = REVERSE_SPEED;
          if (abs(load_motor_1->GetCurr()) >= REVERSE_RELEASE_CURRENT) {
            if (++reverse_debounce > 10) {  // 20 × 5 ms = 100 ms hold
              load_state = LoadState::IDLE;
              print(">>> Load: REVERSING -> IDLE\r\n");
            }
          } else {
            reverse_debounce = 0;
            print(">>> Load: REVERSING, current=%d (debounce reset)\r\n", load_motor_1->GetCurr());
          }
          break;
      }
    } else {
      // ---- Manual load control (legacy behavior) ----
      if (dbus->swr == remote::UP) {
        trigger_motor->SetOutput(TRIGGER_RELEASE_OUTPUT);
      } else {
        trigger_motor->SetOutput(TRIGGER_HOLD_OUTPUT);
      }

      if (dbus->swl == remote::UP) {
        load_target_speed = MANUAL_LOAD_FORWARD_SPEED;
      } else if (dbus->swl == remote::DOWN) {
        load_target_speed = MANUAL_LOAD_REVERSE_SPEED;
      } else {
        load_target_speed = 0;
      }
    }

    // ---- Load motor PID ----
    {
      float diff_load_1 = load_motor_1->GetOmegaDelta(-load_target_speed);
      float diff_load_2 = load_motor_2->GetOmegaDelta(load_target_speed);
      load_motor_1->SetOutput(pid_left.ComputeConstrainedOutput(diff_load_1));
      load_motor_2->SetOutput(pid_right.ComputeConstrainedOutput(diff_load_2));
    }

    // ---- Force motor ----
    force_target_speed = MAP_RANGE(dbus->ch3, -660, 660, -500, 500);
    float diff_force = force_motor->GetOmegaDelta(force_target_speed);
    force_motor->SetOutput(pid_force.ComputeConstrainedOutput(diff_force));

    control::MotorCANBase::TransmitOutput(motors_can1_load, 3);

    // ---- Yaw motor ----
    if (dbus->ch0 > 300) {
      yaw_target_speed = 100;
    } else if (dbus->ch0 < -300) {
      yaw_target_speed = -100;
    } else {
      yaw_target_speed = 0;
    }
    float diff_yaw = yaw_motor->GetOmegaDelta(yaw_target_speed);
    yaw_motor->SetOutput(pid_yaw.ComputeConstrainedOutput(diff_yaw));
    control::MotorCANBase::TransmitOutput(yaw_motors, 1);

    osDelay(5);
  }
}

void RM_RTOS_Init() {
  print_use_usb();

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  buzzer = new bsp::Buzzer(&htim12, 1, 1000000);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
  bump_switch = new bsp::GPIO(P1_GPIO_Port, P1_Pin);

  trigger_motor = new control::MotorPWMBase(&htim4, LEFT_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ,
                                            MOTOR_OUT_FREQ, IDLE_THROTTLE);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);

  // CAN load motors
  load_motor_1 = new control::Motor3508(can1, 0x201);
  load_motor_2 = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor2006(can1, 0x204);
  yaw_motor = new control::Motor3508(can1, 0x205);

  // Initialize weighing scale with address 1, standard frame, using CAN2
  // Constructor automatically registers CAN callbacks for weight responses
  scale = new control::WeighScale(can2, 1, control::WeighScaleFrameType::STANDARD, NUM_CHANNELS);

  dbus = new remote::DBUS(&huart1);
  laser->On();
}

void RM_RTOS_Threads_Init(void) {
  dartLoadTaskHandle = osThreadNew(dartLoadTask, NULL, &dartLoadTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // Wait for system to stabilize

  print("=== Dart Gimbal + WeighScale ===\r\n");
  print("Trigger: DBUS swr (UP=0, MID=300, DOWN=600)\r\n");
  print("Load mode toggle: DBUS ch0 > 500 (edge-triggered)\r\n");
  print("WeighScale: CAN2, Addr=1, %d channels\r\n", NUM_CHANNELS);
  print("Press K1 to Tare\r\n");
  print("================================\r\n\r\n");

  // Initial tare
  print("Initial Tare...\r\n");
  scale->Tare(control::WEIGHSCALE_ALL_CHANNELS);
  osDelay(500);

  uint32_t loop_count = 0;
  bool last_key_state = false;

  while (true) {
    // Check button press for manual tare
    bool key_pressed = (key->Read() == 0);  // Active low
    if (key_pressed && !last_key_state) {
      print(">>> Manual Tare (all channels)...\r\n");
      scale->Tare(control::WEIGHSCALE_ALL_CHANNELS);
      osDelay(200);
    }
    last_key_state = key_pressed;

    // Send ReadWeights request (responses handled by internal callbacks)
    control::WeighScaleData_t temp_data;
    scale->ReadWeights(&temp_data, NUM_CHANNELS);

    // Wait for responses
    osDelay(300);

    // Get parsed weight data from the scale object
    const control::WeighScaleData_t& weight_data = scale->GetData();

    // Print weight values and trigger motor status
    print("[%lu] Trigger: swr=%d | Weights: ", loop_count, dbus->swr);
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
      print("CH%d=%.2fkg ", ch + 1, weight_data.weight[ch] / 1000.0f);
    }
    print("\r\n");

    osDelay(200);
    loop_count++;
  }
}
