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
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "chassis.h"
#include "user_interface.h"

//static remote::DBUS* dbus = nullptr;  // dbus is for test

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;

static volatile bool Dead = false;

static bsp::CanBridge* receive = nullptr;
static unsigned int flag_summary = 0;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;
static const int UI_TASK_DELAY = 20;

// speed for chassis rotation (no unit)
// TODO: the speed for the Sentry chassis(by server)
constexpr float SPIN_SPEED = 500;
constexpr float FOLLOW_SPEED = 200;
static const float CHASSIS_DEADZONE = 0.04;

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
// Chassis(just spinmode)
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
osThreadId_t chassisTaskHandle;

static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

void chassisTask(void* arg) {
 UNUSED(arg);

 control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

 // TODO NOT RECEIVING START SIGNAL?
// while (!receive->start) osDelay(100);

 while (true) {
   float relative_angle = receive->relative_angle;
   float sin_yaw, cos_yaw, vx_set, vy_set;
   float vx, vy, wz;

   vx_set = -receive->vy;
   vy_set = receive->vx;

   // may be need add the move
   // auto start
//   if (/*dbus->swr == remote::UP ||*/ referee->game_status.game_progress == 0x3 || referee->game_status.game_progress == 0x4) {  // spin mode
//     sin_yaw = arm_sin_f32(relative_angle);
//     cos_yaw = arm_cos_f32(relative_angle);
//     vx = cos_yaw * vx_set + sin_yaw * vy_set;
//     vy = -sin_yaw * vx_set + cos_yaw * vy_set;
//     wz = SPIN_SPEED;
//
//     // stop the chassis
//   } else if (/*dbus->swr == remote::DOWN ||*/ (referee->game_status.game_progress != 0x3 && referee->game_status.game_progress != 0x4) || Dead) {
//     vx = 0;
//     vy = 0;
//     wz = 0;
//   }

   if (receive->mode == 1) {  // spin mode
     sin_yaw = arm_sin_f32(relative_angle);
     cos_yaw = arm_cos_f32(relative_angle);
     vx = cos_yaw * vx_set + sin_yaw * vy_set;
     vy = -sin_yaw * vx_set + cos_yaw * vy_set;
     wz = SPIN_SPEED;
   } else {
     sin_yaw = arm_sin_f32(relative_angle);
     cos_yaw = arm_cos_f32(relative_angle);
     vx = cos_yaw * vx_set + sin_yaw * vy_set;
     vy = -sin_yaw * vx_set + cos_yaw * vy_set;
     wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * relative_angle);
     if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
   }

   chassis->SetSpeed(vx, vy, wz);
   chassis->Update(true, (float)referee->game_robot_status.chassis_power_limit,
                   referee->power_heat_data.chassis_power,
                   (float)referee->power_heat_data.chassis_power_buffer);

   control::MotorCANBase::TransmitOutput(motors, 4);

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
//  communication::graphic_data_t graphGimbal;
//  communication::graphic_data_t graphChassis;
//  communication::graphic_data_t graphArrow;
//  communication::graphic_data_t graphCali;
//  communication::graphic_data_t graphEmpty2;
  communication::graphic_data_t graphCrosshair1;
  communication::graphic_data_t graphCrosshair2;
  communication::graphic_data_t graphCrosshair3;
  communication::graphic_data_t graphCrosshair4;
  communication::graphic_data_t graphCrosshair5;
  communication::graphic_data_t graphCrosshair6;
  communication::graphic_data_t graphCrosshair7;
//  communication::graphic_data_t graphBarFrame;
//  communication::graphic_data_t graphBar;
//  communication::graphic_data_t graphPercent;
//  communication::graphic_data_t graphDiag;
//  communication::graphic_data_t graphMode;
//  communication::graphic_data_t graphDist;
  //  communication::graphic_data_t graphLid;
//  communication::graphic_data_t graphWheel;

//  char msgBuffer1[30] = "PITCH MOTOR UNCONNECTED";
//  char msgBuffer2[30] = "YAW MOTOR UNCONNECTED";
//  char msgBuffer3[30] = "L SHOOTER MOTOR UNCONNECTED";
//  char msgBuffer4[30] = "R SHOOTER MOTOR UNCONNECTED";
//  char msgBuffer5[30] = "LOAD MOTOR UNCONNECTED";
//  char msgBuffer6[30] = "FRONT L MOTOR UNCONNECTED";
//  char msgBuffer7[30] = "FRONT R MOTOR UNCONNECTED";
//  char msgBuffer8[30] = "BACK L MOTOR UNCONNECTED";
//  char msgBuffer9[30] = "BACK R MOTOR UNCONNECTED";
//
//  bool pitch_motor_flag_ui = pitch_motor_flag;
//  bool yaw_motor_flag_ui = yaw_motor_flag;
//  bool sl_motor_flag_ui = sl_motor_flag;
//  bool sr_motor_flag_ui = sr_motor_flag;
//  bool ld_motor_flag_ui = ld_motor_flag;
//  bool fl_motor_flag_ui = fl_motor_flag;
//  bool fr_motor_flag_ui = fr_motor_flag;
//  bool bl_motor_flag_ui = bl_motor_flag;
//  bool br_motor_flag_ui = br_motor_flag;

//  // Initialize chassis GUI
//  UI->ChassisGUIInit(&graphChassis, &graphArrow, &graphGimbal, &graphCali, &graphEmpty2);
//  UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
//                   graphCali, graphEmpty2);
//  referee->PrepareUIContent(communication::FIVE_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

  while (true) {
    // Initialize crosshair GUI
    UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                     &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
    UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                     graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                     graphCrosshair7);
    referee->PrepareUIContent(communication::SEVEN_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(100);
  }

//  // Initialize supercapacitor GUI
//  UI->CapGUIInit(&graphBarFrame, &graphBar);
//  UI->GraphRefresh((uint8_t*)(&referee->graphic_double), 2, graphBarFrame, graphBar);
//  referee->PrepareUIContent(communication::DOUBLE_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

//  // Initialize Supercapacitor string GUI
//  UI->CapGUICharInit(&graphPercent);
//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
//                  UI->getPercentLen());
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

//  // Initialize self-diagnosis GUI
//  char diagStr[30] = "";
//  UI->DiagGUIInit(&graphDiag, 30);
//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDiag, diagStr, 2);
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

//  // Initialize current mode GUI
//  char followModeStr[15] = "FOLLOW MODE";
//  char spinModeStr[15] = "SPIN  MODE";
//  uint32_t modeColor = UI_Color_Orange;
//  UI->ModeGUIInit(&graphMode);
//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, followModeStr,
//                  sizeof followModeStr);
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

//  // Initialize distance GUI
//  char distanceStr[15] = "0.0";
//  UI->DistanceGUIInit(&graphDist);
//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphDist, distanceStr,
//                  sizeof distanceStr);
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

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
//  char wheelOffStr[15] = "FLYWHEEL OFF";
//  UI->WheelGUIInit(&graphWheel);
//  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphWheel, wheelOffStr,
//                  sizeof wheelOffStr);
//  referee->PrepareUIContent(communication::CHAR_GRAPH);
//  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//  referee_uart->Write(frame.data, frame.length);
//  osDelay(UI_TASK_DELAY);

//  float j = 1;
//  while (true) {
//    //     lidar_flag = LIDAR->startMeasure();
//
//    // Update chassis GUI
//    UI->ChassisGUIUpdate(receive->relative_angle, calibration_flag);
//    UI->GraphRefresh((uint8_t*)(&referee->graphic_five), 5, graphChassis, graphArrow, graphGimbal,
//                     graphCali, graphEmpty2);
//    referee->PrepareUIContent(communication::FIVE_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_TASK_DELAY);
//
//    // Update supercapacitor GUI
//    UI->CapGUIUpdate(std::abs(sin(j)));
//    UI->GraphRefresh((uint8_t*)(&referee->graphic_single), 1, graphBar);
//    referee->PrepareUIContent(communication::SINGLE_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    j += 0.1;
//    osDelay(UI_TASK_DELAY);
//
//    // Update supercapacitor string GUI
//    UI->CapGUICharUpdate();
//    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphPercent, UI->getPercentStr(),
//                    UI->getPercentLen());
//    referee->PrepareUIContent(communication::CHAR_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_TASK_DELAY);
//
//    // Update current mode GUI
//    char* modeStr = receive->mode ? spinModeStr : followModeStr;
//    modeColor = receive->mode ? UI_Color_Green : UI_Color_Orange;
//    UI->ModeGuiUpdate(&graphMode, modeColor);
//    UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphMode, modeStr, 15);
//    referee->PrepareUIContent(communication::CHAR_GRAPH);
//    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//    referee_uart->Write(frame.data, frame.length);
//    osDelay(UI_TASK_DELAY);

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

//    // Update self-diagnosis messages
//    if (!pitch_motor_flag_ui && !pitch_motor_flag) {
//      UI->AddMessage(msgBuffer1, sizeof msgBuffer1, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      pitch_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!yaw_motor_flag_ui && !yaw_motor_flag) {
//      UI->AddMessage(msgBuffer2, sizeof msgBuffer2, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      yaw_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!sl_motor_flag_ui && !sl_motor_flag) {
//      UI->AddMessage(msgBuffer3, sizeof msgBuffer3, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      sl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!sr_motor_flag_ui && !sr_motor_flag) {
//      UI->AddMessage(msgBuffer4, sizeof msgBuffer4, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      sr_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!ld_motor_flag_ui && !ld_motor_flag) {
//      UI->AddMessage(msgBuffer5, sizeof msgBuffer5, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      ld_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!fl_motor_flag_ui && !fl_motor_flag) {
//      UI->AddMessage(msgBuffer6, sizeof msgBuffer6, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      fl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!fr_motor_flag_ui && !fr_motor_flag) {
//      UI->AddMessage(msgBuffer7, sizeof msgBuffer7, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      fr_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!bl_motor_flag_ui && !bl_motor_flag) {
//      UI->AddMessage(msgBuffer8, sizeof msgBuffer8, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      bl_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }
//
//    if (!br_motor_flag_ui && !br_motor_flag) {
//      UI->AddMessage(msgBuffer9, sizeof msgBuffer9, UI, referee, &graphDiag);
//      referee->PrepareUIContent(communication::CHAR_GRAPH);
//      frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
//      referee_uart->Write(frame.data, frame.length);
//      br_motor_flag_ui = true;
//      osDelay(UI_TASK_DELAY);
//    }

    //    // clear self-diagnosis messages
    //    if (dbus->keyboard.bit.C) {
    //      for (int i = 1; i <= UI->getMessageCount(); ++i) {
    //        UI->DiagGUIClear(UI, referee, &graphDiag, i);
    //        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    //        referee_uart->Write(frame.data, frame.length);
    //        osDelay(UI_TASK_DELAY);
    //      }
    //    }
//  }
}

//==================================================================================================
// SelfTest
//==================================================================================================

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

//static bool fl_motor_flag = false;
//static bool fr_motor_flag = false;
//static bool bl_motor_flag = false;
//static bool br_motor_flag = false;

static bool transmission_flag = true;

void self_Check_Task(void* arg){
 UNUSED(arg);

 while(true){
   osDelay(100);
   fl_motor->connection_flag_ = false;
   fr_motor->connection_flag_ = false;
   bl_motor->connection_flag_ = false;
   br_motor->connection_flag_ = false;

   osDelay(100);

   fl_motor_flag = fl_motor->connection_flag_;
   fr_motor_flag = fr_motor->connection_flag_;
   bl_motor_flag = bl_motor->connection_flag_;
   br_motor_flag = br_motor->connection_flag_;

   flag_summary = fl_motor_flag |
                  fr_motor_flag << 1 |
                  bl_motor_flag << 2 |
                  br_motor_flag << 3;

   osDelay(100);
   if(transmission_flag){
     receive->cmd.id = bsp::CHASSIS_FLAG;
     receive->cmd.data_uint = (unsigned int)flag_summary;
     receive->TransmitOutput();
   }
   transmission_flag = !transmission_flag;
 }
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init() {
 print_use_uart(&huart1);
 bsp::SetHighresClockTimer(&htim5);

 can1 = new bsp::CAN(&hcan1, true);
 can2 = new bsp::CAN(&hcan2, false);
 RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

 fl_motor = new control::Motor3508(can1, 0x202);
 fr_motor = new control::Motor3508(can1, 0x201);
 bl_motor = new control::Motor3508(can1, 0x203);
 br_motor = new control::Motor3508(can1, 0x204);

 control::MotorCANBase* motors[control::FourWheel::motor_num];
 motors[control::FourWheel::front_left] = fl_motor;
 motors[control::FourWheel::front_right] = fr_motor;
 motors[control::FourWheel::back_left] = bl_motor;
 motors[control::FourWheel::back_right] = br_motor;

 control::chassis_t chassis_data;
 chassis_data.motors = motors;
 chassis_data.model = control::CHASSIS_OMNI_WHEEL;
 chassis = new control::Chassis(chassis_data);

 referee_uart = new RefereeUART(&huart6);
 referee_uart->SetupRx(300);
 referee_uart->SetupTx(300);
 referee = new communication::Referee;

 receive = new bsp::CanBridge(can2, 0x20B, 0x20A);

// dbus = new remote::DBUS(&huart3);
}

//==================================================================================================
// RTOS Threads Inits
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
 refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
 chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
 selfTestTaskHandle = osThreadNew(self_Check_Task, nullptr, &selfTestingTask);
// UITaskHandle = osThreadNew(UITask, nullptr, &UITaskAttribute);
}

//==================================================================================================
// Kill All
//==================================================================================================

void KillAll() {
 RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

 control::MotorCANBase* motors_can1_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};

 RGB->Display(display::color_blue);

 while (true) {
   if (!receive->dead) {
     Dead = false;
     RGB->Display(display::color_green);
     break;
   }
   chassis->SetSpeed(0, 0, 0);
   control::MotorCANBase::TransmitOutput(motors_can1_chassis, 4);

   osDelay(KILLALL_DELAY);
 }
}

//==================================================================================================
// RTOS Default Task
//==================================================================================================

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
   //    print("out: %d\r\n", referee->game_robot_status.mains_power_gimbal_output);
   //    print("test");
   if (debug) {
     set_cursor(0, 0);
     clear_screen();
     //      print("vx: %f, vy: %f, angle: %f, mode: %f, dead: %f, start: %f\r\n", receive->vx, receive->vy,
     //            receive->relative_angle, receive->mode, receive->dead, receive->start);
     print("power limit: %.3f chassis power: %.3f power buffer: %.3f\r\n", (float)referee->game_robot_status.chassis_power_limit,
           referee->power_heat_data.chassis_power,
           (float)referee->power_heat_data.chassis_power_buffer);
   }
   osDelay(DEFAULT_TASK_DELAY);
 }
}
