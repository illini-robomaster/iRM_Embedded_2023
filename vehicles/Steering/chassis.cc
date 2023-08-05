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

#include "bsp_can_bridge.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "steering.h"
#include "supercap.h"
#include "user_interface.h"
#include <cmath>

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;
static BoolEdgeDetector ReCali(false);
static BoolEdgeDetector Revival(false);

static bool volatile pitch_motor_flag = false;
static bool volatile yaw_motor_flag = false;
static bool volatile sl_motor_flag = false;
static bool volatile sr_motor_flag = false;
static bool volatile ld_motor_flag = false;
static bool volatile fl_motor_flag = false;
static bool volatile fr_motor_flag = false;
static bool volatile bl_motor_flag = false;
static bool volatile br_motor_flag = false;
static bool volatile calibration_flag = false;
static bool volatile referee_flag = false;
static bool volatile dbus_flag = false;
static bool volatile lidar_flag = false;

//==================================================================================================
// SelfTest
//==================================================================================================

static bool fl_steer_motor_flag = false;
static bool fr_steer_motor_flag = false;
static bool bl_steer_motor_flag = false;
static bool br_steer_motor_flag = false;
static bool fl_wheel_motor_flag = false;
static bool fr_wheel_motor_flag = false;
static bool bl_wheel_motor_flag = false;
static bool br_wheel_motor_flag = false;

static bool transmission_flag = true;
const osThreadAttr_t selfTestingTask = {.name = "selfTestTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityBelowNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t selfTestTaskHandle;


static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;

static bsp::CanBridge* receive = nullptr;
static unsigned int flag_summary = 0;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 30;
static const int CHASSIS_TASK_DELAY = 2;
static const int UI_TASK_DELAY = 20;

// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);

// speed for chassis rotation (no unit)
constexpr float SPIN_SPEED = 40;
constexpr float FOLLOW_SPEED = 40;

//==================================================================================================
// Referee
//==================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)


const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 1024 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t refereeTaskHandle;
osThreadId_t chassisTaskHandle;
class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//==================================================================================================
// Chassis
//==================================================================================================

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

static control::MotorCANBase* motor1 = nullptr;
static control::MotorCANBase* motor2 = nullptr;
static control::MotorCANBase* motor3 = nullptr;
static control::MotorCANBase* motor4 = nullptr;
static control::MotorCANBase* motor5 = nullptr;
static control::MotorCANBase* motor6 = nullptr;
static control::MotorCANBase* motor7 = nullptr;
static control::MotorCANBase* motor8 = nullptr;

static control::SteeringMotor* steering_motor1 = nullptr;
static control::SteeringMotor* steering_motor2 = nullptr;
static control::SteeringMotor* steering_motor3 = nullptr;
static control::SteeringMotor* steering_motor4 = nullptr;

static bsp::GPIO* pe1 = nullptr;
static bsp::GPIO* pe2 = nullptr;
static bsp::GPIO* pe3 = nullptr;
static bsp::GPIO* pe4 = nullptr;

static control::steering_chassis_t* chassis_data;
static control::SteeringChassis* chassis;

static control::SuperCap* supercap = nullptr;

static const float CHASSIS_DEADZONE = 0.04;
static const float CHASSIS_DEADZONE2 = 0.1;
static float SPIN_DOWN_SPEED_FACTOR = 0.0;

bool steering_align_detect1() { return pe1->Read() == 0; }

bool steering_align_detect2() { return pe2->Read() == 0; }

bool steering_align_detect3() { return pe3->Read() == 0; }

bool steering_align_detect4() { return pe4->Read() == 0; }

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  while (!receive->start) osDelay(100);

  while (receive->start < 0.5) osDelay(100);

  // Alignment
  chassis->SteerSetMaxSpeed(ALIGN_SPEED);
  bool alignment_complete = false;
  while (!alignment_complete) {
    chassis->SteerCalcOutput();
    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    alignment_complete = chassis->Calibrate();
    osDelay(1);
  }
  chassis->ReAlign();
  chassis->SteerCalcOutput();
  control::MotorCANBase::TransmitOutput(steer_motors, 4);

  chassis->SteerSetMaxSpeed(RUN_SPEED);
  chassis->SteerThetaReset();
  chassis->SetWheelSpeed(0,0,0,0);

  float WHEEL_SPEED_FACTOR = 0.0;
  float power_limit = 0.0;
  float power_limit_lowerbound = 45.0;

  while (true) {
    float relative_angle = receive->relative_angle;
    float sin_yaw, cos_yaw, vx_set, vy_set;
    float vx, vy, wz;

    // TODO need to change the channels in gimbal.cc
    vx_set = -receive->vy;
    vy_set = receive->vx;

    ReCali.input(receive->recalibrate);   // detect force recalibration
    Revival.input(receive->dead);         // detect robot revival

    // realign on revival OR when key 'R' is pressed
    if (Revival.negEdge() || ReCali.posEdge()) {
      chassis->SteerAlignFalse();
      chassis->SteerSetMaxSpeed(ALIGN_SPEED);
      bool realignment_complete = false;
      while (!realignment_complete) {
        chassis->SteerCalcOutput();
        control::MotorCANBase::TransmitOutput(steer_motors, 4);
        realignment_complete = chassis->Calibrate();
        osDelay(1);
      }
      chassis->ReAlign();
      chassis->SteerCalcOutput();
      control::MotorCANBase::TransmitOutput(steer_motors, 4);
      
      chassis->SteerSetMaxSpeed(RUN_SPEED);
      chassis->SteerThetaReset();
      chassis->SetWheelSpeed(0,0,0,0);
    }

    float supercap_voltage = (float)(supercap->info.voltage / 1000.0);
    float maximum_energy = 0.5 * pow(27.0,2) * 6.0;

    float current_energy = pow(supercap_voltage,2) * 6 / 2;

    //consider using uart printing to check the power limit's value
    //log values out as files to obtain its trend
    //WHEEL_SPEED_FACTOR separated for two modes
    if (current_energy <= 0.1 * maximum_energy) {
      // case when remaining energy of capacitor is below 10%, recharge the super capacitor
      power_limit = power_limit_lowerbound;
      WHEEL_SPEED_FACTOR = 4.0;
    } else if (current_energy <= 0.2 * maximum_energy && current_energy > 0.1 * maximum_energy) {
      // case when remaining energy of capacitor is between 10% and 20%
      power_limit = power_limit_lowerbound + (power_limit_lowerbound / (0.1 * maximum_energy)) * (current_energy - 0.1 * maximum_energy);
      WHEEL_SPEED_FACTOR = 6.0 + (4.0 / ((0.1 * maximum_energy))) * (current_energy - 0.1 * maximum_energy);
    } else if (current_energy <= 0.3 * maximum_energy && current_energy > 0.2 * maximum_energy) {
      // case when remaining energy of capacitor is between 20% and 30%
      power_limit = 80.0 + (60.0 / (0.1 * maximum_energy)) * (current_energy - 0.2 * maximum_energy);
      WHEEL_SPEED_FACTOR = 10.0 + (2.0 / ((0.1 * maximum_energy))) * (current_energy - 0.2 * maximum_energy);
    } else if (current_energy > 0.3 * maximum_energy) {
      // case when remaining energy of capacitor is above 30%
      power_limit = 120;
      WHEEL_SPEED_FACTOR = 12.0;
      // WHEEL SPEED FACTOR will be 12
    }

    SPIN_DOWN_SPEED_FACTOR = 4.0 / WHEEL_SPEED_FACTOR;
    WHEEL_SPEED_FACTOR = receive->mode== 1 ? 4.0 : WHEEL_SPEED_FACTOR;

    if (receive->mode == 1) {  // spin mode
      // delay compensation
      // based on rule-of-thumb formula SPIN_SPEED = 80 = ~30 degree of error
      relative_angle = relative_angle - PI * 30.0 / 180.0 / 80.0 * SPIN_SPEED;
      relative_angle = relative_angle - PI / 9.0;
      chassis->SteerSetMaxSpeed(RUN_SPEED * 3 / 2);

      sin_yaw = sin(relative_angle);
      cos_yaw = cos(relative_angle);
      vx = cos_yaw * vx_set + sin_yaw * vy_set;
      vy = -sin_yaw * vx_set + cos_yaw * vy_set;
      vx = vx * 2.0;
      vy = vy * 2.0;
      wz = SPIN_SPEED * 2.0;
    } else {
      chassis->SteerSetMaxSpeed(RUN_SPEED / SPIN_DOWN_SPEED_FACTOR);
      sin_yaw = sin(relative_angle);
      cos_yaw = cos(relative_angle);
      vx = cos_yaw * vx_set + sin_yaw * vy_set;
      vy = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz = (float)(std::min(FOLLOW_SPEED , FOLLOW_SPEED * relative_angle ) * SPIN_DOWN_SPEED_FACTOR);
      //When relative angle in dead zone, do not follow gimbal.
      if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
      //If vx, vy are close to zero, dead zone is larger to avoid wheel to be 45 degree, leading to flipping.
      else if (vx <= 0.1 && vy <= 0.1 && (-CHASSIS_DEADZONE2 < relative_angle && relative_angle < CHASSIS_DEADZONE2)) wz = 0;
    }
    // set the desired speed
    chassis->SetSpeed(vx / 10, vy / 10, wz);
    chassis->SteerUpdateTarget();
    chassis->WheelUpdateSpeed(WHEEL_SPEED_FACTOR);
    // update the steering and wheel output
    chassis->SteerCalcOutput();
    chassis->Update((float)power_limit, referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);
    
    if (Dead) {
      chassis->SetSpeed(0,0,0);
      motor5->SetOutput(0);
      motor6->SetOutput(0);
      motor7->SetOutput(0);
      motor8->SetOutput(0);
    }

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    control::MotorCANBase::TransmitOutput(steer_motors, 4);

    receive->cmd.id = bsp::SHOOTER_POWER;
    receive->cmd.data_bool = referee->game_robot_status.mains_power_shooter_output;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT1;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT2;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id2_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_speed_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_speed_limit;
    receive->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);

  }
}

//==================================================================================================
// UI
//==================================================================================================

const osThreadAttr_t UITaskAttribute = {.name = "UITask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

osThreadId_t UITaskHandle;

// static distance::LIDAR07_UART* LIDAR = nullptr;
static communication::UserInterface* UI = nullptr;

void UITask(void* arg) {
  UNUSED(arg);

//  while (!selftestStart) osDelay(100);

  //   int tryLIDAR = 0;
  //   while (!LIDAR->begin()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }
  //   tryLIDAR = 0;
  //   while (!LIDAR->startFilter()) {
  //     if (++tryLIDAR >= 5) break;
  //     osDelay(10);
  //   }

  UI->SetID(referee->game_robot_status.robot_id);

  communication::package_t frame;
  communication::graphic_data_t graphGimbal;
  communication::graphic_data_t graphChassis;
  communication::graphic_data_t graphArrow;
  communication::graphic_data_t graphCali;
  communication::graphic_data_t graphEmpty2;
  communication::graphic_data_t graphCrosshair1;
  communication::graphic_data_t graphCrosshair2;
  communication::graphic_data_t graphCrosshair3;
  communication::graphic_data_t graphCrosshair4;
  communication::graphic_data_t graphCrosshair5;
  communication::graphic_data_t graphCrosshair6;
  communication::graphic_data_t graphCrosshair7;
  communication::graphic_data_t graphBarFrame;
  communication::graphic_data_t graphBar;
  communication::graphic_data_t graphPercent;
  communication::graphic_data_t graphDiag;
  communication::graphic_data_t graphMode;
  communication::graphic_data_t graphDist;
  //  communication::graphic_data_t graphLid;
  communication::graphic_data_t graphWheel;

  char msgBuffer1[30] = "PITCH MOTOR UNCONNECTED";
  char msgBuffer2[30] = "YAW MOTOR UNCONNECTED";
  char msgBuffer3[30] = "L SHOOTER MOTOR UNCONNECTED";
  char msgBuffer4[30] = "R SHOOTER MOTOR UNCONNECTED";
  char msgBuffer5[30] = "LOAD MOTOR UNCONNECTED";
  char msgBuffer6[30] = "FRONT L MOTOR UNCONNECTED";
  char msgBuffer7[30] = "FRONT R MOTOR UNCONNECTED";
  char msgBuffer8[30] = "BACK L MOTOR UNCONNECTED";
  char msgBuffer9[30] = "BACK R MOTOR UNCONNECTED";

  bool pitch_motor_flag_ui = pitch_motor_flag;
  bool yaw_motor_flag_ui = yaw_motor_flag;
  bool sl_motor_flag_ui = sl_motor_flag;
  bool sr_motor_flag_ui = sr_motor_flag;
  bool ld_motor_flag_ui = ld_motor_flag;
  bool fl_motor_flag_ui = fl_motor_flag;
  bool fr_motor_flag_ui = fr_motor_flag;
  bool bl_motor_flag_ui = bl_motor_flag;
  bool br_motor_flag_ui = br_motor_flag;

  // Initialize chassis GUI
  UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                   graphCali, graphEmpty2);
  referee->PrepareUIContent(communication::FIVE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize crosshair GUI
  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                   &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                   graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                   graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize supercapacitor GUI
  UI->CapGUIInit(&graphBarFrame, &graphBar);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize Supercapacitor string GUI
  UI->CapGUICharInit(&graphPercent);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                  UI->getPercentLen());
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize self-diagnosis GUI
  char diagStr[30] = "";
  UI->DiagGUIInit(&graphDiag, 30);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize current mode GUI
  char followModeStr[15] = "FOLLOW MODE";
  char spinModeStr[15] = "SPIN  MODE";
  uint32_t modeColor = UI_Color_Orange;
  UI->ModeGUIInit(&graphMode);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, followModeStr,
                  sizeof followModeStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Initialize distance GUI
  char distanceStr[15] = "0.0";
  UI->DistanceGUIInit(&graphDist);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr,
                  sizeof distanceStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // TODO: add lid UI in the future

  //  // Initialize lid status GUI
  //  char lidOpenStr[15] = "LID OPENED";
  //  char lidCloseStr[15] = "LID CLOSED";
  //  UI->LidGUIInit(&graphLid);
  //  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidOpenStr, sizeof
  //  lidOpenStr); referee->PrepareUIContent(communication::CHAR_GRAPH); frame =
  //  referee->Transmit(communication::STUDENT_INTERACTIVE); referee_uart->Write(frame.data,
  //  frame.length); osDelay(UI_TASK_DELAY);

  // Initialize flywheel status GUI
//  char wheelOnStr[15] = "FLYWHEEL ON";
  char wheelOffStr[15] = "FLYWHEEL OFF";
  UI->WheelGUIInit(&graphWheel);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelOffStr,
                  sizeof wheelOffStr);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  float j = 1;
  while (true) {
    //     lidar_flag = LIDAR->startMeasure();

    // Update chassis GUI
    UI->ChassisGUIUpdate(receive->relative_angle, calibration_flag);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
                     graphCali, graphEmpty2);
    referee->PrepareUIContent(communication::FIVE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor GUI
    UI->CapGUIUpdate(std::abs(sin(j)));
    UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
    referee->PrepareUIContent(communication::SINGLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    j += 0.1;
    osDelay(UI_TASK_DELAY);

    // Update supercapacitor string GUI
    UI->CapGUICharUpdate();
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
                    UI->getPercentLen());
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update current mode GUI
    char* modeStr = SpinMode ? spinModeStr : followModeStr;
    modeColor = SpinMode ? UI_Color_Green : UI_Color_Orange;
    UI->ModeGuiUpdate(&graphMode, modeColor);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, 15);
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Update distance GUI
    //     uint32_t distColor = UI_Color_Cyan;
    //     float currDist = LIDAR->distance / 1000.0;
    //     if (currDist < 60) {
    //       snprintf(distanceStr, 15, "%.2f m", currDist);
    //       distColor = UI_Color_Cyan;
    //     } else {
    //       snprintf(distanceStr, 15, "ERROR");
    //       distColor = UI_Color_Pink;
    //     }
    //     UI->DistanceGUIUpdate(&graphDist, distColor);
    //     UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr, 15);
    //     referee->PrepareUIContent(communication::CHAR_GRAPH);
    //     frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //     referee_uart->Write(frame.data, frame.length);
    //     osDelay(UI_TASK_DELAY);

    //    // Update lid status GUI
    //    char lidStr[15] = lidFlag ? lidOpenStr : lidCloseStr;
    //    uint32_t lidColor = lidFlag ? UI_Color_Pink : UI_Color_Green;
    //    UI->LidGuiUpdate(&graphLid, lidColor);
    //    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphLid, lidStr, 15);
    //    referee->PrepareUIContent(communication::CHAR_GRAPH);
    //    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //    referee_uart->Write(frame.data, frame.length);
    //    osDelay(UI_TASK_DELAY);

//    // Update wheel status GUI
//    char* wheelStr = flywheelFlag ? wheelOnStr : wheelOffStr;
//    uint32_t wheelColor = flywheelFlag ? UI_Color_Pink : UI_Color_Green;
//    UI->WheelGUIUpdate(&graphWheel, wheelColor);
//    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelStr, 15);
//    referee->PrepareUIContent(communication::CHAR_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_TASK_DELAY);

    // Update self-diagnosis messages
    if (!pitch_motor_flag_ui && !pitch_motor_flag) {
      UI->AddMessage(msgBuffer1, sizeof msgBuffer1, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      pitch_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!yaw_motor_flag_ui && !yaw_motor_flag) {
      UI->AddMessage(msgBuffer2, sizeof msgBuffer2, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      yaw_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!sl_motor_flag_ui && !sl_motor_flag) {
      UI->AddMessage(msgBuffer3, sizeof msgBuffer3, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      sl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!sr_motor_flag_ui && !sr_motor_flag) {
      UI->AddMessage(msgBuffer4, sizeof msgBuffer4, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      sr_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!ld_motor_flag_ui && !ld_motor_flag) {
      UI->AddMessage(msgBuffer5, sizeof msgBuffer5, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      ld_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!fl_motor_flag_ui && !fl_motor_flag) {
      UI->AddMessage(msgBuffer6, sizeof msgBuffer6, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      fl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!fr_motor_flag_ui && !fr_motor_flag) {
      UI->AddMessage(msgBuffer7, sizeof msgBuffer7, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      fr_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!bl_motor_flag_ui && !bl_motor_flag) {
      UI->AddMessage(msgBuffer8, sizeof msgBuffer8, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      bl_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

    if (!br_motor_flag_ui && !br_motor_flag) {
      UI->AddMessage(msgBuffer9, sizeof msgBuffer9, UI, referee, &graphDiag);
      referee->PrepareUIContent(communication::CHAR_GRAPH);
      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
      referee_uart->Write(frame.data, frame.length);
      br_motor_flag_ui = true;
      osDelay(UI_TASK_DELAY);
    }

//    // clear self-diagnosis messages
//    if (dbus->keyboard.bit.C) {
//      for (int i = 1; i <= UI->getMessageCount(); ++i) {
//        UI->DiagGUIClear(UI, referee, &graphDiag, i);
//        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//        referee_uart->Write(frame.data, frame.length);
//        osDelay(UI_TASK_DELAY);
//      }
//    }
  }
}

void self_Check_Task(void* arg){
  UNUSED(arg);

  while(true){
    osDelay(100);
    motor8->connection_flag_ = false;
    motor7->connection_flag_ = false;
    motor6->connection_flag_ = false;
    motor5->connection_flag_ = false;
    motor4->connection_flag_ = false;
    motor3->connection_flag_ = false;
    motor2->connection_flag_ = false;
    motor1->connection_flag_ = false;
    osDelay(100);
    fl_wheel_motor_flag = motor8->connection_flag_;
    fr_wheel_motor_flag = motor7->connection_flag_;
    bl_wheel_motor_flag = motor6->connection_flag_;
    br_wheel_motor_flag = motor5->connection_flag_;
    fl_steer_motor_flag = motor4->connection_flag_;
    fr_steer_motor_flag = motor3->connection_flag_;
    br_steer_motor_flag = motor2->connection_flag_;
    bl_steer_motor_flag = motor1->connection_flag_;
    flag_summary = bl_steer_motor_flag|
                   br_steer_motor_flag<<1|
                   fr_steer_motor_flag<<2|
                   fl_steer_motor_flag<<3|
                   br_wheel_motor_flag<<4|
                   bl_wheel_motor_flag<<5|
                   fr_wheel_motor_flag<<6|
                   fl_wheel_motor_flag<<7;
    osDelay(100);
    if(transmission_flag){
      receive->cmd.id = bsp::CHASSIS_FLAG;
      receive->cmd.data_uint = (unsigned int)flag_summary;
      receive->TransmitOutput();
      receive->cmd.id = bsp :: SUPERCAP_VOLTAGE;
      receive->cmd.data_float = supercap->info.voltage / 1000;
      receive->TransmitOutput();
    }
    transmission_flag = !transmission_flag;
  }
}
void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  pe1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  pe2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  pe3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  pe4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  chassis_data = new control::steering_chassis_t();



  control::steering_t steering_motor_data;
  steering_motor_data.motor = motor1;
  steering_motor_data.max_speed = RUN_SPEED;
  steering_motor_data.max_acceleration = ACCELERATION;
  steering_motor_data.transmission_ratio = 8;
  steering_motor_data.omega_pid_param = new float[3]{200, 7, 1};
  steering_motor_data.max_iout = 1000;
  steering_motor_data.max_out = 13000;
  steering_motor_data.calibrate_offset = 0;

  steering_motor_data.align_detect_func = steering_align_detect1;
  steering_motor1 = new control::SteeringMotor(steering_motor_data);

  steering_motor_data.motor = motor2;
  steering_motor_data.align_detect_func = steering_align_detect2;
  steering_motor2 = new control::SteeringMotor(steering_motor_data);
  steering_motor_data.motor = motor3;
  steering_motor_data.align_detect_func = steering_align_detect3;
  steering_motor3 = new control::SteeringMotor(steering_motor_data);
  steering_motor_data.motor = motor4;
  steering_motor_data.align_detect_func = steering_align_detect4;
  steering_motor4 = new control::SteeringMotor(steering_motor_data);

  chassis_data = new control::steering_chassis_t();

  chassis_data->fl_steer_motor = steering_motor4;
  chassis_data->fr_steer_motor = steering_motor3;
  chassis_data->bl_steer_motor = steering_motor1;
  chassis_data->br_steer_motor = steering_motor2;

  chassis_data->fl_wheel_motor = motor8;
  chassis_data->fr_wheel_motor = motor7;
  chassis_data->bl_wheel_motor = motor5;
  chassis_data->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(chassis_data);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;
  receive = new bsp::CanBridge(can2, 0x20B, 0x20A);
  supercap = new control::SuperCap(can2, 0x301);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  selfTestTaskHandle = osThreadNew(self_Check_Task, nullptr, &selfTestingTask);
//  UITaskHandle = osThreadNew(UITask, nullptr, &UITaskAttribute);
}

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  RGB->Display(display::color_blue);

  chassis->SteerAlignFalse();   // set alignment status of each wheel to false

  while (true) {
    if (!receive->dead) {
      SpinMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      break;
    }

    motor5->SetOutput(0);
    motor6->SetOutput(0);
    motor7->SetOutput(0);
    motor8->SetOutput(0);

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = false;

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    if (receive->dead) {
      Dead = true;
      KillAll();
    }

    receive->cmd.id = bsp::GIMBAL_POWER;
    receive->cmd.data_uint = referee->game_robot_status.mains_power_gimbal_output;
    receive->TransmitOutput();

    receive->cmd.id = bsp::IS_MY_COLOR_BLUE;
    receive->cmd.data_bool = (referee->game_robot_status.robot_id >= 100) ? true : false;
    receive->TransmitOutput();

//    print("type: %d\r\n", referee->robot_hurt.hurt_type);


    if (debug) {
      set_cursor(0, 0);
      clear_screen();
      print("vx: %f, vy: %f, angle: %f, mode: %f, dead: %f\r\n", receive->vx, receive->vy,
            receive->relative_angle, receive->mode, receive->dead);
    }
    osDelay(DEFAULT_TASK_DELAY);
  }
}
