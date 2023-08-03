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

#pragma once

#include "bsp_uart.h"
#include "protocol.h"

/********************** content ID data********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/********************** red ID ****************************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/********************** blue ID ***************************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/********************** red operator ID *******************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/********************** blue operator ID ******************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/********************** deletion **************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/********************** operations ************************/
#define UI_Graph_Add 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/********************** graph configurations - types ******/
#define UI_Graph_Line 0
#define UI_Graph_Rectangle 1
#define UI_Graph_Circle 2
#define UI_Graph_Ellipse 3
#define UI_Graph_Arc 4
#define UI_Graph_Float 5
#define UI_Graph_Int 6
#define UI_Graph_Char 7
/********************** graph configurations - colors *****/
#define UI_Color_Main 0
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4
#define UI_Color_Pink 5
#define UI_Color_Cyan 6
#define UI_Color_Black 7
#define UI_Color_White 8

namespace communication {

class UserInterface {
 public:
  UserInterface(bsp::UART* uart, communication::Referee* referee);
  bool SetID(int Robot_ID);
  static void LineDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                       uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                       uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
  static void RectangleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                            uint32_t start_x, uint32_t start_y, uint32_t end_x,
                            uint32_t end_y);
  static void CircleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                         uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                         uint32_t start_x, uint32_t start_y, uint32_t graph_radius);
  static void EllipseDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                          uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                          uint32_t start_x, uint32_t start_y, uint32_t x_length,
                          uint32_t y_length);
  static void ArcDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                      uint32_t graph_layer, uint32_t graph_color, uint32_t graph_startAngle,
                      uint32_t graph_endAngle, uint32_t graph_width, uint32_t start_x,
                      uint32_t start_y, uint32_t x_length, uint32_t y_length);
  static void FloatDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                        uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                        uint32_t graph_digit, uint32_t graph_width, uint32_t start_x,
                        uint32_t start_y, float graph_float);
  static void IntDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                      uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                      uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                      int graph_int);
  static void CharDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                       uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                       uint32_t char_length, uint32_t graph_width, uint32_t start_x,
                       uint32_t start_y);
  int WriteData(uint8_t* data_buffer, communication::content graph_content);
  int UIDelete(uint8_t del_operate, uint8_t del_layer);
  int GraphRefresh(int cnt, ...);
  int CharRefresh(graphic_data_t image, char* theString, int len);

  void DiagGUIInit(graphic_data_t* message, int len);
  void DiagGUIUpdate(int len);
  void DiagGUIClear(UserInterface* UI, Referee* referee, graphic_data_t* graph,
                    int currCount);
  void AddMessage(graphic_data_t* graph, char* messageStr, int len);

 private:
  bsp::UART* uart_;
  communication::Referee* referee_;
  communication::package_t frame_;
  int Robot_ID_;
  int Client_ID_;
  graphic_data_t* gimbal_;
  graphic_data_t* chassis_;
  graphic_data_t* arrow_;
  graphic_data_t* crosshair_;
  graphic_data_t* bar_;
  graphic_data_t* percent_;
  graphic_data_t* diag_;
  graphic_data_t* cali_;
  int centerX_ = 960;
  int centerY_ = 540;
  int chassisX_ = 1300;
  int chassisY_ = 120;
  int gimbalLen_;
  int chassisLen_;
  int barStartX_ = 1500;
  int barStartY_ = 380;
  char percentStr_[30];
  int percentLen_;
  float cap_;
  int diagStartX_ = 1470;
  int diagStartY_ = 910;  // 420 - 910
  int messageCount_ = 0;
  int modeStartX_ = 1220;
  int modeStartY_ = 45;
};

class ChassisGUI {
 public:
  ChassisGUI(UserInterface* UI, int16_t chassis_X = 1300, int16_t chassis_Y = 120,
             int16_t speed_center_X = 200, int16_t speed_center_Y = 780);
  void Init();
  void Delete();
  void Init2();
  void Delete2();
  void Update(float speed_x, float speed_y, float relative);

 private:
  UserInterface* UI_;
  int16_t chassis_X_;
  int16_t chassis_Y_;
  int16_t gimbalLen_;
  int16_t chassisLen_;
  int16_t speed_center_X_;
  int16_t speed_center_Y_;
  int16_t speed_circle_R_;
  graphic_data_t chassis_;
  graphic_data_t arrow_;
  graphic_data_t gimbal_;
  graphic_data_t speed_circle_;
  graphic_data_t speed_center_;
  graphic_data_t speed_x_axis_;
  graphic_data_t speed_y_axis_;
  graphic_data_t speed_x_val_;
  graphic_data_t speed_y_val_;
};

class CrossairGUI {
 public:
  CrossairGUI(UserInterface* UI);
  void Init();
  void Delete();

 private:
  UserInterface* UI_;
  graphic_data_t crosshair1_;
  graphic_data_t crosshair2_;
  graphic_data_t crosshair3_;
  graphic_data_t crosshair4_;
  graphic_data_t crosshair5_;
  graphic_data_t crosshair6_;
  graphic_data_t crosshair7_;
  int16_t centerX_;
  int16_t centerY_;
};

class Bar {
 public:
  Bar(int16_t barStartX = 1500, int16_t barStartY = 350, int16_t barWidth = 200,
      int16_t barHeight = 50, uint8_t color = UI_Color_Orange,
      uint8_t frame_color = UI_Color_Pink, bool isVertical = false);
  graphic_data_t Init();
  graphic_data_t Delete();
  graphic_data_t InitFrame();
  graphic_data_t DeleteFrame();
  graphic_data_t Update(float percent, int8_t color = -1);
  static uint8_t barcount_;

 private:
  uint8_t barID_;
  int16_t barStartX_;
  int16_t barStartY_;
  int16_t barWidth_;
  int16_t barHeight_;
  bool isVertical_;
  float percent_;
  uint8_t color_;
  uint8_t frame_color_;
  graphic_data_t barFrame_;
  graphic_data_t bar_;
  char name_[15];
  char name_frame_[15];
};

class GimbalGUI {
 public:
  GimbalGUI(UserInterface* UI, int16_t gimbal_speed_center_X = 1620,
            int16_t gimbal_speed_center_Y = 780, int16_t gimbal_speed_center_R = 100,
            int16_t pitch_bar_X = 1800, int16_t pitch_bar_Y = 440,
            int16_t pitch_bar_height = 400, int16_t pitch_bar_weight = 30,
            float pitch_max = 0.4253f);
  ~GimbalGUI();
  void Init();
  void Init2();
  void Delete();
  void Delete2();
  void Update(float vpitch, float vyaw, float pitch, float yaw, bool flags);

 private:
  UserInterface* UI_;
  int16_t gimbal_speed_center_X_;
  int16_t gimbal_speed_center_Y_;
  int16_t gimbal_speed_circle_R_;
  int16_t pitch_bar_X_;
  int16_t pitch_bar_Y_;
  int16_t pitch_bar_height_;
  int16_t pitch_bar_weight_;
  float pitch_max_;
  graphic_data_t speed_circle_;
  graphic_data_t speed_center_;
  graphic_data_t speed_x_axis_;
  graphic_data_t speed_y_axis_;
  graphic_data_t speed_x_val_;
  graphic_data_t speed_y_val_;
  graphic_data_t calibration_flag_;
  Bar* pitch_bar_;
  graphic_data_t pitch_bar_frame_;
  graphic_data_t pitch_bar_val_;
};

typedef void (*delay_t)(uint32_t milli);

class CapGUI {
 public:
  CapGUI(UserInterface* UI, char* Cap_name, int16_t Cap_bar_X = 1500, int16_t Cap_bar_Y = 380,
         int16_t Cap_bar_width = 310, int16_t Cap_bar_height = 20);
  ~CapGUI();
  void Init();
  void InitName();
  void Delete();
  void DeleteName();
  void UpdateBulk(float percent, graphic_data_t* bar = nullptr,
                  graphic_data_t* cap_percent = nullptr);
  void Update(float Percent);

 private:
  UserInterface* UI_;
  Bar* cap_bar_;
  graphic_data_t barFrame_;
  graphic_data_t bar_;
  graphic_data_t cap_percent_;
  char cap_percent_name_[15];
  graphic_data_t cap_name_;
  char cap_name_name_[15];
  graphic_data_t empty_;
  char empty_name_[15];
  char* cap_name_str_;
  delay_t delay_function_;
  int16_t cap_bar_X_;
  int16_t cap_bar_Y_;
  int16_t cap_bar_height_;
  int16_t cap_bar_width_;
  int8_t cap_ID_;
  static uint8_t cap_count_;
  int8_t name_length_;
};

class StringGUI {
 public:
  StringGUI(UserInterface* UI, char* string_content, int16_t string_X = 1500,
            int16_t string_Y = 380, int8_t color = UI_Color_Main, int16_t string_size = 15,
            char* string_name = nullptr);
  ~StringGUI();
  graphic_data_t InitBulk();
  void Init();
  void InitString(char* string = nullptr);
  graphic_data_t DeleteBulk();
  void Delete();
  void Update(char* string, int8_t color = -1, int16_t size = -1);
  static uint8_t string_count_;

 private:
  UserInterface* UI_;
  graphic_data_t string_;
  char string_name_[15];
  char* string_content_;
  uint8_t string_length_;
  int16_t string_X_;
  int16_t string_Y_;
  int16_t string_size_;
  uint8_t string_ID_;
  int8_t color_;
};

class DiagGUI {
 public:
  DiagGUI(UserInterface* UI, int16_t diag_X = 350, int16_t diag_Y = 850);
  ~DiagGUI();
  void Update(char* String, delay_t delay_function, int8_t color);
  void Clear(delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

 private:
  UserInterface* UI_;
  StringGUI* diag_string_[25];
  int16_t diag_X_;
  int16_t diag_Y_;
  int8_t count_;
};
}  // namespace communication