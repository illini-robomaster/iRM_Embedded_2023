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

#include "user_interface.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "arm_math.h"
#include "utils.h"

namespace communication {
UserInterface::UserInterface(bsp::UART* uart, communication::Referee* referee)
    : uart_(uart), referee_(referee) {
}

bool UserInterface::SetID(int Robot_ID) {
  switch (Robot_ID) {
    case 1:
      Robot_ID_ = UI_Data_RobotID_RHero;
      Client_ID_ = UI_Data_CilentID_RHero;
      break;
    case 2:
      Robot_ID_ = UI_Data_RobotID_REngineer;
      Client_ID_ = UI_Data_CilentID_REngineer;
      break;
    case 3:
      Robot_ID_ = UI_Data_RobotID_RStandard1;
      Client_ID_ = UI_Data_CilentID_RStandard1;
      break;
    case 4:
      Robot_ID_ = UI_Data_RobotID_RStandard2;
      Client_ID_ = UI_Data_CilentID_RStandard2;
      break;
    case 5:
      Robot_ID_ = UI_Data_RobotID_RStandard3;
      Client_ID_ = UI_Data_CilentID_RStandard3;
      break;
    case 6:
      Robot_ID_ = UI_Data_RobotID_RAerial;
      Client_ID_ = UI_Data_CilentID_RAerial;
      break;
    case 101:
      Robot_ID_ = UI_Data_RobotID_BHero;
      Client_ID_ = UI_Data_CilentID_BHero;
      break;
    case 102:
      Robot_ID_ = UI_Data_RobotID_BEngineer;
      Client_ID_ = UI_Data_CilentID_BEngineer;
      break;
    case 103:
      Robot_ID_ = UI_Data_RobotID_BStandard1;
      Client_ID_ = UI_Data_CilentID_BStandard1;
      break;
    case 104:
      Robot_ID_ = UI_Data_RobotID_BStandard2;
      Client_ID_ = UI_Data_CilentID_BStandard2;
      break;
    case 105:
      Robot_ID_ = UI_Data_RobotID_BStandard3;
      Client_ID_ = UI_Data_CilentID_BStandard3;
      break;
    case 106:
      Robot_ID_ = UI_Data_RobotID_BAerial;
      Client_ID_ = UI_Data_CilentID_BAerial;
      break;
    default:
      return false;
  }
  return true;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graphOperate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param end_x ending x coordinate
 * @param end_y ending y coordinate
 */
void UserInterface::LineDraw(graphic_data_t* image, const char name[3], uint32_t graphOperate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                             uint32_t start_x, uint32_t start_y, uint32_t end_x,
                             uint32_t end_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graphOperate;
  image->graphic_type = UI_Graph_Line;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = end_x;
  image->end_y = end_y;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param end_x ending x coordinate
 * @param end_y ending y coordinate
 */
void UserInterface::RectangleDraw(graphic_data_t* image, const char name[3],
                                  uint32_t graph_operate, uint32_t graph_layer,
                                  uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                                  uint32_t start_y, uint32_t end_x, uint32_t end_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Rectangle;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = end_x;
  image->end_y = end_y;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param graph_radius graph radius
 */
void UserInterface::CircleDraw(graphic_data_t* image, const char name[3],
                               uint32_t graph_operate, uint32_t graph_layer,
                               uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                               uint32_t start_y, uint32_t graph_radius) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Circle;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->radius = graph_radius;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_width graph width
 * @param start_x starting (center) x coordinate
 * @param start_y starting (center) y coordinate
 * @param x_length horizontal semi-axis length
 * @param y_length vertical semi-axis length
 */
void UserInterface::EllipseDraw(graphic_data_t* image, const char name[3],
                                uint32_t graph_operate, uint32_t graph_layer,
                                uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                                uint32_t start_y, uint32_t x_length, uint32_t y_length) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Ellipse;
  image->layer = graph_layer;
  image->color = graph_color;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = x_length;
  image->end_y = y_length;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_startAngle starting angle
 * @param graph_endAngle ending angle
 * @param graph_width graph width
 * @param start_x starting (center) x coordinate
 * @param start_y starting (center) y coordinate
 * @param x_length horizontal semi-axis length
 * @param y_length vertical semi-axis length
 */
void UserInterface::ArcDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color,
                            uint32_t graph_startAngle, uint32_t graph_endAngle,
                            uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                            uint32_t x_length, uint32_t y_length) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Arc;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_startAngle;
  image->end_angle = graph_endAngle;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  image->end_x = x_length;
  image->end_y = y_length;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_size graph size
 * @param graph_digit number of digits after decimal
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 * @param graph_float float to be displayed
 */
void UserInterface::FloatDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                              uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                              uint32_t graph_digit, uint32_t graph_width, uint32_t start_x,
                              uint32_t start_y, float graph_float) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Float;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->end_angle = graph_digit;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  float float_data = graph_float;
  int32_t float2int_data;
  memcpy(&float2int_data, (uint8_t*)&float_data, 4);
  image->radius = float2int_data & 0x3FF;
  image->end_x = (float2int_data & 0x1FFC00) >> 10;
  image->end_y = (float2int_data & 0xFFE00000) >> 21;
}

void UserInterface::IntDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                            uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                            int graph_int) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Int;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
  int32_t int_data = graph_int;
  image->radius = int_data & 0x3FF;
  image->end_x = (int_data & 0x1FFC00) >> 10;
  image->end_y = (int_data & 0xFFE00000) >> 21;
}

/**
 *
 * @param image pointer to a graphic_data_t variable; stores image data
 * @param name image name
 * @param graph_operate graph operations; check document
 * @param graph_layer layer 0 to 9
 * @param graph_color graph color
 * @param graph_size graph size
 * @param char_length character length
 * @param graph_width graph width
 * @param start_x starting x coordinate
 * @param start_y starting y coordinate
 */
void UserInterface::CharDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                             uint32_t char_length, uint32_t graph_width, uint32_t start_x,
                             uint32_t start_y) {
  for (int i = 0; i < 3 && name[i] != 0; i++)
    image->graphic_name[2 - i] = name[i];
  image->operate_type = graph_operate;
  image->graphic_type = UI_Graph_Char;
  image->layer = graph_layer;
  image->color = graph_color;
  image->start_angle = graph_size;
  image->end_angle = char_length;
  image->width = graph_width;
  image->start_x = start_x;
  image->start_y = start_y;
}

int UserInterface::WriteData(uint8_t* data_buffer, communication::content graph_content) {
  switch (graph_content) {
    case SINGLE_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_single), data_buffer,
             sizeof(graphic_single_t));
      break;
    case DOUBLE_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_double), data_buffer,
             sizeof(graphic_double_t));
      break;
    case FIVE_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_five), data_buffer, sizeof(graphic_five_t));
      break;
    case SEVEN_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_seven), data_buffer, sizeof(graphic_seven_t));
      break;
    case CHAR_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_character), data_buffer,
             sizeof(graphic_character_t));
      break;
    case DELETE_GRAPH:
      memcpy((uint8_t*)(&referee_->graphic_delete), data_buffer,
             sizeof(graphic_delete_t));
      break;
    default:
      break;
  }
  referee_->PrepareUIContent(graph_content);
  frame_ = referee_->Transmit(communication::STUDENT_INTERACTIVE);
  uart_->Write(frame_.data, frame_.length);
  return frame_.length;
}

int UserInterface::UIDelete(uint8_t del_operate, uint8_t del_layer) {
  graphic_delete_t del;
  del.header.data_cmd_id = UI_Data_ID_Del;
  del.header.sender_ID = Robot_ID_;
  del.header.receiver_ID = Client_ID_;
  del.operate_type = del_operate;
  del.layer = del_layer;
  return WriteData((uint8_t*)(&del), DELETE_GRAPH);
  //        int length = sizeof(graphic_delete_t);
  //        memcpy(data_buffer, &del, length);
  //        return length;
}

int UserInterface::GraphRefresh(int cnt, ...) {
  va_list arg;
  va_start(arg, cnt);
  UI_header_data_t header;
  header.sender_ID = Robot_ID_;
  header.receiver_ID = Client_ID_;
  //        int length = -1;
  communication::content graph_content;
  switch (cnt) {
    case 1: {
      header.data_cmd_id = UI_Data_ID_Draw1;
      graphic_single_t graph;
      graph.header = header;
      graph.graphic_data_struct = va_arg(arg, graphic_data_t);
      //                length = sizeof(graphic_single_t);
      graph_content = SINGLE_GRAPH;
      return WriteData((uint8_t*)(&graph), graph_content);
      //                memcpy(data_buffer, &graph, length);
      break;
    }
    case 2: {
      header.data_cmd_id = UI_Data_ID_Draw2;
      graphic_double_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i)
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      //                length = sizeof(graphic_double_t);
      graph_content = DOUBLE_GRAPH;
      return WriteData((uint8_t*)(&graph), graph_content);
      //                memcpy(data_buffer, &graph, length);
      break;
    }
    case 5: {
      header.data_cmd_id = UI_Data_ID_Draw5;
      graphic_five_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i)
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      //                length = sizeof(graphic_five_t);
      graph_content = FIVE_GRAPH;
      return WriteData((uint8_t*)(&graph), graph_content);
      //                memcpy(data_buffer, &graph, length);
      break;
    }
    case 7: {
      header.data_cmd_id = UI_Data_ID_Draw7;
      graphic_seven_t graph;
      graph.header = header;
      for (int i = 0; i < cnt; ++i)
        graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
      //                length = sizeof(graphic_seven_t);
      graph_content = SEVEN_GRAPH;
      return WriteData((uint8_t*)(&graph), graph_content);
      //                memcpy(data_buffer, &graph, length);
      break;
    }
    default:;
  }
  return -1;

  //        return length;
}

int UserInterface::CharRefresh(graphic_data_t image, char* theString, int len) {
  graphic_character_t char_graph;
  char_graph.header.data_cmd_id = UI_Data_ID_DrawChar;
  char_graph.header.sender_ID = Robot_ID_;
  char_graph.header.receiver_ID = Client_ID_;
  char_graph.graphic_data_struct = image;
  if (len > 30 || len <= 0)
    return -1;
  memset(char_graph.data, 0, sizeof(char_graph.data));
  memcpy(char_graph.data, theString, len);
  return WriteData((uint8_t*)(&char_graph), CHAR_GRAPH);
  //        int length = sizeof(graphic_character_t);
  //        memcpy(data_buffer, &char_graph, length);
  //        return length;
}

void UserInterface::DiagGUIInit(graphic_data_t* message, int len) {
  diag_ = message;
  CharDraw(message, "M0", UI_Graph_Add, 2, UI_Color_Pink, 10, len, 2, diagStartX_,
           diagStartY_);
}

void UserInterface::DiagGUIUpdate(int len) {
  int currY = diagStartY_ - messageCount_ * 20;
  char name[15];
  snprintf(name, 15, "M%d", messageCount_);
  CharDraw(diag_, name, UI_Graph_Add, 2, UI_Color_Pink, 10, len, 2, diagStartX_, currY);
}

void UserInterface::AddMessage(graphic_data_t* graph, char* messageStr, int len) {
  messageCount_++;
  if (messageCount_ > 25)
    return;
  DiagGUIUpdate(len);
  CharRefresh(*graph, messageStr, len);
}

void UserInterface::DiagGUIClear(UserInterface* UI, Referee* referee, graphic_data_t* graph,
                                 int currCount) {
  char str[] = " ";
  char name[15];
  snprintf(name, 15, "M%d", currCount);
  UI->CharDraw(diag_, name, UI_Graph_Change, 2, UI_Color_Pink, 10, 30, 2, diagStartX_,
               diagStartY_ - currCount * 20);
  UI->CharRefresh(*graph, str, 1);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
}

ChassisGUI::ChassisGUI(UserInterface* UI, int16_t chassis_X, int16_t chassis_Y,
                       int16_t speed_center_X, int16_t speed_center_Y)
    : UI_(UI) {
  chassis_X_ = chassis_X;
  chassis_Y_ = chassis_Y;
  speed_center_X_ = speed_center_X;
  speed_center_Y_ = speed_center_Y;
  gimbalLen_ = 90;
  chassisLen_ = 90;
  speed_circle_R_ = 100;
  Init();
}
void ChassisGUI::Init() {
  UI_->LineDraw(&chassis_, "c", UI_Graph_Add, 1, UI_Color_Yellow, 60, chassis_X_,
                chassis_Y_ - chassisLen_ / 2, chassis_X_, chassis_Y_ + chassisLen_ / 2);
  UI_->LineDraw(&arrow_, "a", UI_Graph_Add, 1, UI_Color_Yellow, 20, chassis_X_ - 7,
                chassis_Y_ + chassisLen_ / 2 - 7, chassis_X_ + 7,
                chassis_Y_ + chassisLen_ / 2 + 7);
  UI_->LineDraw(&gimbal_, "g", UI_Graph_Add, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                chassis_X_, chassis_Y_ + gimbalLen_);
  UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Add, 1, UI_Color_Yellow, 2, speed_center_X_,
                  speed_center_Y_, speed_circle_R_);
  UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Add, 2, UI_Color_Green, 10, speed_center_X_,
                  speed_center_Y_, 5);
  UI_->LineDraw(&speed_x_axis_, "sx", UI_Graph_Add, 3, UI_Color_Pink, 2,
                speed_center_X_ - speed_circle_R_, speed_center_Y_,
                speed_center_X_ + speed_circle_R_, speed_center_Y_);
  UI_->LineDraw(&speed_y_axis_, "sy", UI_Graph_Add, 3, UI_Color_Pink, 2, speed_center_X_,
                speed_center_Y_ - speed_circle_R_, speed_center_X_,
                speed_center_Y_ + speed_circle_R_);
  UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_axis_,
                    speed_y_axis_);
}

void ChassisGUI::Delete() {
  UI_->LineDraw(&chassis_, "c", UI_Graph_Del, 1, UI_Color_Yellow, 60, chassis_X_,
                chassis_Y_ - chassisLen_ / 2, chassis_X_, chassis_Y_ + chassisLen_ / 2);
  UI_->LineDraw(&arrow_, "a", UI_Graph_Del, 1, UI_Color_Yellow, 20, chassis_X_ - 7,
                chassis_Y_ + chassisLen_ / 2 - 7, chassis_X_ + 7,
                chassis_Y_ + chassisLen_ / 2 + 7);
  UI_->LineDraw(&gimbal_, "g", UI_Graph_Del, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                chassis_X_, chassis_Y_ + gimbalLen_);
  UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Del, 1, UI_Color_Yellow, 2, speed_center_X_,
                  speed_center_Y_, speed_circle_R_);
  UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Del, 2, UI_Color_Green, 10, speed_center_X_,
                  speed_center_Y_, 5);
  UI_->LineDraw(&speed_x_axis_, "sx", UI_Graph_Del, 3, UI_Color_Pink, 2,
                speed_center_X_ - speed_circle_R_, speed_center_Y_,
                speed_center_X_ + speed_circle_R_, speed_center_Y_);
  UI_->LineDraw(&speed_y_axis_, "sy", UI_Graph_Del, 3, UI_Color_Pink, 2, speed_center_X_,
                speed_center_Y_ - speed_circle_R_, speed_center_X_,
                speed_center_Y_ + speed_circle_R_);
  UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_axis_,
                    speed_y_axis_);
}

void ChassisGUI::Init2() {
  UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
               speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2, 0);
  UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
               speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10, 0);
  UI_->GraphRefresh(2, speed_x_val_, speed_y_val_);
}
void ChassisGUI::Delete2() {
  UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
               speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2, 0);
  UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
               speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10, 0);
  UI_->GraphRefresh(2, speed_x_val_, speed_y_val_);
}

void ChassisGUI::Update(float speed_x, float speed_y, float relative) {
  float x_end = chassis_X_ - chassisLen_ / 2.0 * arm_sin_f32(relative);
  float y_end = chassis_Y_ + chassisLen_ / 2.0 * arm_cos_f32(relative);
  float x_start = chassis_X_ + chassisLen_ / 2.0 * arm_sin_f32(relative);
  float y_start = chassis_Y_ - chassisLen_ / 2.0 * arm_cos_f32(relative);
  UI_->LineDraw(&chassis_, "c", UI_Graph_Change, 1, UI_Color_Yellow, 60, (uint32_t)x_start,
                (uint32_t)y_start, (uint32_t)x_end, (uint32_t)y_end);
  UI_->LineDraw(&arrow_, "a", UI_Graph_Change, 1, UI_Color_Yellow, 20,
                (uint32_t)(x_end + 10 * arm_sin_f32(relative + M_PI / 4)),
                (uint32_t)(y_end - 10 * arm_cos_f32(relative + M_PI / 4)),
                (uint32_t)(x_end - 10 * arm_sin_f32(relative + M_PI / 4)),
                (uint32_t)(y_end + 10 * arm_cos_f32(relative + M_PI / 4)));
  UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Change, 2, UI_Color_Green, 10,
                  speed_center_X_ + (int16_t)(speed_x * speed_circle_R_),
                  speed_center_Y_ + (int16_t)(speed_y * speed_circle_R_), 5);
  UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
               speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2,
               (int32_t)(speed_x * 100.0f));
  UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
               speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10,
               (int32_t)(speed_y * 100.0f));
  UI_->LineDraw(&gimbal_, "g", UI_Graph_Change, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                chassis_X_, chassis_Y_ + gimbalLen_);
  // Static element that can be remove
  UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Change, 1, UI_Color_Yellow, 2,
                  speed_center_X_, speed_center_Y_, speed_circle_R_);

  UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_val_,
                    speed_y_val_);
}

CrossairGUI::CrossairGUI(UserInterface* UI) : UI_(UI) {
  centerX_ = 960;
  centerY_ = 540;
  Init();
}

void CrossairGUI::Init() {
  UI_->LineDraw(&crosshair1_, "ch1", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50,
                centerY_ - 90, centerX_ + 50, centerY_ - 90);
  UI_->LineDraw(&crosshair2_, "ch2", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 120, centerX_ + 30, centerY_ - 120);
  UI_->LineDraw(&crosshair3_, "ch3", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 540, centerX_ + 30, centerY_ - 540);
  UI_->LineDraw(&crosshair4_, "ch4", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50,
                centerY_ - 540, centerX_ + 50, centerY_ - 540);
  UI_->LineDraw(&crosshair5_, "ch5", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 540, centerX_ + 30, centerY_ - 540);
  UI_->LineDraw(&crosshair6_, "ch6", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 540, centerX_ + 30, centerY_ - 540);
  UI_->LineDraw(&crosshair7_, "ch7", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_,
                centerY_ - 30, centerX_, centerY_ - 200);
  UI_->GraphRefresh(7, crosshair1_, crosshair2_, crosshair3_, crosshair4_, crosshair5_,
                    crosshair6_, crosshair7_);
}

void CrossairGUI::Delete() {
  UI_->LineDraw(&crosshair1_, "ch1", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 50,
                centerY_ - 40, centerX_ + 50, centerY_ - 40);
  UI_->LineDraw(&crosshair2_, "ch2", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 55, centerX_ + 30, centerY_ - 55);
  UI_->LineDraw(&crosshair3_, "ch3", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 60, centerX_ + 30, centerY_ - 60);
  UI_->LineDraw(&crosshair4_, "ch4", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 50,
                centerY_ - 70, centerX_ + 50, centerY_ - 70);
  UI_->LineDraw(&crosshair5_, "ch5", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 80, centerX_ + 30, centerY_ - 80);
  UI_->LineDraw(&crosshair6_, "ch6", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                centerY_ - 90, centerX_ + 30, centerY_ - 90);
  UI_->LineDraw(&crosshair7_, "ch7", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_,
                centerY_ - 30, centerX_, centerY_ - 100);
  UI_->GraphRefresh(7, crosshair1_, crosshair2_, crosshair3_, crosshair4_, crosshair5_,
                    crosshair6_, crosshair7_);
}

uint8_t Bar::barcount_ = 0;

Bar::Bar(int16_t barStartX, int16_t barStartY, int16_t barWidth, int16_t barHeight,
         uint8_t color, uint8_t frame_color, bool isVertical) {
  barStartX_ = barStartX;
  barStartY_ = barStartY;
  barWidth_ = barWidth;
  barHeight_ = barHeight;
  isVertical_ = isVertical;
  color_ = color;
  frame_color_ = frame_color;
  barID_ = Bar::barcount_;
  Bar::barcount_++;
}

graphic_data_t Bar::Init() {
  memset(name_, ' ', 15);
  snprintf(name_, 15, "b%d", barID_);
  communication::UserInterface::LineDraw(
      &bar_, name_, UI_Graph_Add, 0, color_, barHeight_ - 10, barStartX_ + barWidth_ / 2,
      barStartY_ + 5, barStartX_ + barWidth_ / 2, barStartY_ + barHeight_ - 5);
  return bar_;
}

graphic_data_t Bar::Delete() {
  communication::UserInterface::LineDraw(
      &bar_, name_, UI_Graph_Del, 0, color_, 2, barStartX_ + 5, barStartY_ + 5,
      barStartX_ + barWidth_ - 5, barStartY_ + barHeight_ - 5);
  return bar_;
}

graphic_data_t Bar::InitFrame() {
  memset(name_frame_, ' ', 15);
  snprintf(name_frame_, 15, "f%d", barID_);
  communication::UserInterface::RectangleDraw(
      &barFrame_, name_frame_, UI_Graph_Add, 0, frame_color_, 2, barStartX_, barStartY_,
      barStartX_ + barWidth_, barStartY_ + barHeight_);
  return barFrame_;
}

graphic_data_t Bar::DeleteFrame() {
  communication::UserInterface::RectangleDraw(
      &barFrame_, name_frame_, UI_Graph_Del, 0, frame_color_, 2, barStartX_, barStartY_,
      barStartX_ + barWidth_, barStartY_ + barHeight_);
  return barFrame_;
}

graphic_data_t Bar::Update(float percent, int8_t color) {
  percent_ = percent;
  if (color != -1)
    color_ = color;
  if (isVertical_) {
    communication::UserInterface::LineDraw(&bar_, name_, UI_Graph_Change, 0, color_,
                                           barWidth_ - 10, barStartX_ + barWidth_ / 2,
                                           barStartY_ + 5, barStartX_ + barWidth_ / 2,
                                           barStartY_ + 5 + (barHeight_ - 10) * percent_);
  } else {
    communication::UserInterface::LineDraw(
        &bar_, name_, UI_Graph_Change, 0, color_, barHeight_ - 10, barStartX_ + 5,
        barStartY_ + barHeight_ / 2, barStartX_ + 5 + (barWidth_ - 10) * percent_,
        barStartY_ + barHeight_ / 2);
  }
  return bar_;
}

GimbalGUI::GimbalGUI(UserInterface* UI, int16_t gimbal_speed_center_X,
                     int16_t gimbal_speed_center_Y, int16_t gimbal_speed_circle_R,
                     int16_t pitch_bar_X, int16_t pitch_bar_Y, int16_t pitch_bar_height,
                     int16_t pitch_bar_weight, float pitch_max)
    : UI_(UI) {
  gimbal_speed_center_X_ = gimbal_speed_center_X;
  gimbal_speed_center_Y_ = gimbal_speed_center_Y;
  gimbal_speed_circle_R_ = gimbal_speed_circle_R;
  pitch_bar_X_ = pitch_bar_X;
  pitch_bar_Y_ = pitch_bar_Y;
  pitch_bar_height_ = pitch_bar_height;
  pitch_bar_weight_ = pitch_bar_weight;
  pitch_max_ = pitch_max;
  pitch_bar_ = new Bar(pitch_bar_X_, pitch_bar_Y_, pitch_bar_weight_, pitch_bar_height_,
                       UI_Color_Green, UI_Color_Pink, true);
  Init();
}

GimbalGUI::~GimbalGUI() {
  delete pitch_bar_;
}
void GimbalGUI::Init() {
  UI_->CircleDraw(&speed_circle_, "gc", UI_Graph_Add, 1, UI_Color_Yellow, 2,
                  gimbal_speed_center_X_, gimbal_speed_center_Y_, gimbal_speed_circle_R_);
  UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Add, 2, UI_Color_Green, 10,
                  gimbal_speed_center_X_, gimbal_speed_center_Y_, 5);
  UI_->LineDraw(&speed_x_axis_, "gx", UI_Graph_Add, 3, UI_Color_Pink, 2,
                gimbal_speed_center_X_ - gimbal_speed_circle_R_, gimbal_speed_center_Y_,
                gimbal_speed_center_X_ + gimbal_speed_circle_R_, gimbal_speed_center_Y_);
  UI_->LineDraw(&speed_y_axis_, "gy", UI_Graph_Add, 3, UI_Color_Pink, 2,
                gimbal_speed_center_X_, gimbal_speed_center_Y_ - gimbal_speed_circle_R_,
                gimbal_speed_center_X_, gimbal_speed_center_Y_ + gimbal_speed_circle_R_);
  UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Add, 0, UI_Color_Pink, 14, 960 + 925,
                  540 + 330, 7);
  UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
               gimbal_speed_center_Y_ - 2, 0);
  UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ - 5,
               gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, 0);
  UI_->GraphRefresh(7, speed_circle_, speed_center_, speed_x_axis_, speed_y_axis_,
                    speed_x_val_, speed_y_val_, calibration_flag_);
}

void GimbalGUI::Delete() {
  UI_->CircleDraw(&speed_circle_, "gc", UI_Graph_Del, 1, UI_Color_Yellow, 2,
                  gimbal_speed_center_X_, gimbal_speed_center_Y_, gimbal_speed_circle_R_);
  UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Del, 2, UI_Color_Green, 10,
                  gimbal_speed_center_X_, gimbal_speed_center_Y_, 5);
  UI_->LineDraw(&speed_x_axis_, "gx", UI_Graph_Del, 3, UI_Color_Pink, 2,
                gimbal_speed_center_X_ - gimbal_speed_circle_R_, gimbal_speed_center_Y_,
                gimbal_speed_center_X_ + gimbal_speed_circle_R_, gimbal_speed_center_Y_);
  UI_->LineDraw(&speed_y_axis_, "gy", UI_Graph_Del, 3, UI_Color_Pink, 2,
                gimbal_speed_center_X_, gimbal_speed_center_Y_ - gimbal_speed_circle_R_,
                gimbal_speed_center_X_, gimbal_speed_center_Y_ + gimbal_speed_circle_R_);
  UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Del, 0, UI_Color_Pink, 14, 960 + 925,
                  540 + 330, 7);
  UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
               gimbal_speed_center_Y_ - 2, 0);
  UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ - 5,
               gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, 0);
  UI_->GraphRefresh(7, speed_circle_, speed_center_, speed_x_axis_, speed_y_axis_,
                    speed_x_val_, speed_y_val_, calibration_flag_);
}

void GimbalGUI::Init2() {
  pitch_bar_val_ = pitch_bar_->Init();
  pitch_bar_frame_ = pitch_bar_->InitFrame();
  UI_->GraphRefresh(2, pitch_bar_frame_, pitch_bar_val_);
}
void GimbalGUI::Delete2() {
  pitch_bar_val_ = pitch_bar_->Delete();
  pitch_bar_frame_ = pitch_bar_->DeleteFrame();
  UI_->GraphRefresh(2, pitch_bar_frame_, pitch_bar_val_);
}

void GimbalGUI::Update(float vpitch, float vyaw, float pitch, float yaw, bool flags) {
  UNUSED(yaw);
  vpitch = clip<float>(vpitch, -1, 1);
  vyaw = clip<float>(vyaw, -1, 1);
  if (flags) {
    UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Change, 0, UI_Color_Green, 14,
                    960 + 925, 540 + 330, 7);
  } else {
    UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Change, 0, UI_Color_Pink, 14,
                    960 + 925, 540 + 330, 7);
  }
  UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Change, 2, UI_Color_Green, 10,
                  gimbal_speed_center_X_ + (int16_t)(vyaw * gimbal_speed_circle_R_),
                  gimbal_speed_center_Y_ + (int16_t)(vpitch * gimbal_speed_circle_R_), 5);
  UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
               gimbal_speed_center_Y_ - 2, (int32_t)(vyaw * 100));
  UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
               gimbal_speed_center_X_ - 5,
               gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, (int32_t)(vpitch * 100));
  float pitch_percent = 1 - (pitch + pitch_max_) / (2 * pitch_max_);
  pitch_percent = clip<float>(pitch_percent, 0, 1);
  pitch_bar_val_ = pitch_bar_->Update(pitch_percent);
  UI_->GraphRefresh(5, speed_center_, speed_x_val_, speed_y_val_, calibration_flag_,
                    pitch_bar_val_);
}

uint8_t CapGUI::cap_count_ = 0;

CapGUI::CapGUI(UserInterface* UI, char* cap_name, int16_t cap_bar_X, int16_t cap_bar_Y,
               int16_t cap_bar_width, int16_t cap_bar_height)
    : UI_(UI), cap_name_str_(cap_name) {
  cap_bar_X_ = cap_bar_X;
  cap_bar_Y_ = cap_bar_Y;
  cap_bar_height_ = cap_bar_height;
  cap_bar_width_ = cap_bar_width;
  cap_bar_ = new Bar(cap_bar_X_, cap_bar_Y_, cap_bar_width_, cap_bar_height_, UI_Color_Green,
                     UI_Color_Yellow);
  cap_ID_ = CapGUI::cap_count_;
  CapGUI::cap_count_++;
  Init();
}

void CapGUI::Init() {
  name_length_ = strlen(cap_name_str_);
  bar_ = cap_bar_->Init();
  barFrame_ = cap_bar_->InitFrame();
  memset(cap_name_name_, ' ', 15);
  memset(empty_name_, ' ', 15);
  memset(cap_percent_name_, ' ', 15);
  snprintf(cap_name_name_, 15, "V%d", cap_ID_);
  snprintf(cap_percent_name_, 15, "v%d", cap_ID_);
  snprintf(empty_name_, 15, "W%d", cap_ID_);
  UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Add, 1, UI_Color_Yellow, 15,
                name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
  UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Add, 3, UI_Color_Yellow, 15, 2,
               cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10, 100);

  UI_->CircleDraw(&empty_, empty_name_, UI_Graph_Add, 0, UI_Color_Green, 0, 0, 0, 0);
  UI_->GraphRefresh(5, barFrame_, bar_, cap_percent_, cap_name_, empty_);
}
void CapGUI::Delete() {
  bar_ = cap_bar_->Delete();
  barFrame_ = cap_bar_->DeleteFrame();
  UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Del, 1, UI_Color_Yellow, 15,
                name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
  UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Del, 3, UI_Color_Yellow, 15, 2,
               cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10, 100);
  UI_->CircleDraw(&empty_, empty_name_, UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);
  UI_->GraphRefresh(5, barFrame_, bar_, cap_name_, cap_percent_, empty_);
}
void CapGUI::InitName() {
  char theRealName[15];
  snprintf(theRealName, 15, "%s     %%", cap_name_str_);
  UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Change, 1, UI_Color_Yellow, 15,
                name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
  UI_->CharRefresh(cap_name_, theRealName, strlen(theRealName));
}
void CapGUI::DeleteName() {
}
void CapGUI::UpdateBulk(float percent, graphic_data_t* bar, graphic_data_t* cap_percent) {
  percent = clip<float>(percent, 0, 1);
  if (percent > 0.8)
    bar_ = cap_bar_->Update(percent, UI_Color_Green);
  else if (percent > 0.2)
    bar_ = cap_bar_->Update(percent, UI_Color_Orange);
  else
    bar_ = cap_bar_->Update(percent, UI_Color_Pink);
  UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Change, 3, UI_Color_Yellow, 15, 2,
               cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10,
               (int32_t)(100 * percent));
  if (bar != nullptr)
    *bar = bar_;
  if (cap_percent != nullptr)
    *cap_percent = cap_percent_;
}
void CapGUI::Update(float percent) {
  UpdateBulk(percent);
  UI_->GraphRefresh(2, bar_, cap_percent_);
}

uint8_t StringGUI::string_count_ = 0;

StringGUI::StringGUI(communication::UserInterface* UI, char* string_content, int16_t string_X,
                     int16_t string_Y, int8_t color, int16_t string_size, char* string_name)
    : UI_(UI), string_content_(string_content) {
  string_X_ = string_X;
  string_Y_ = string_Y;
  string_size_ = string_size;
  color_ = color;
  string_length_ = strlen(string_content_);
  if (string_name == nullptr) {
    string_ID_ = StringGUI::string_count_;
    StringGUI::string_count_++;
    memset(string_name_, ' ', 15);
    snprintf(string_name_, 15, "S%d", string_ID_);
  } else {
    strcpy(string_name_, string_name);
  }
}

StringGUI::~StringGUI() {
}

graphic_data_t StringGUI::InitBulk() {
  UI_->CharDraw(&string_, string_name_, UI_Graph_Add, 4, color_, string_size_, string_length_,
                2, string_X_, string_Y_);
  return string_;
}

void StringGUI::Init() {
  InitBulk();
  UI_->CharRefresh(string_, string_content_, string_length_);
}

void StringGUI::InitString(char* string) {
  if (string != nullptr)
    string_content_ = string;
  string_length_ = strlen(string_content_);
  UI_->CharDraw(&string_, string_name_, UI_Graph_Change, 4, color_, string_size_,
                string_length_, 2, string_X_, string_Y_);
  UI_->CharRefresh(string_, string_content_, string_length_);
}

graphic_data_t StringGUI::DeleteBulk() {
  UI_->CharDraw(&string_, string_name_, UI_Graph_Del, 4, color_, string_size_, string_length_,
                2, string_X_, string_Y_);
  return string_;
}

void StringGUI::Delete() {
  DeleteBulk();
  UI_->GraphRefresh(1, string_);
}

void StringGUI::Update(char* string, int8_t color, int16_t size) {
  if (string != nullptr)
    string_content_ = string;
  if (color != -1)
    color_ = color;
  if (size != -1)
    string_size_ = size;
  string_length_ = strlen(string_content_);
  UI_->CharDraw(&string_, string_name_, UI_Graph_Change, 4, color_, string_size_,
                string_length_, 2, string_X_, string_Y_);
  UI_->CharRefresh(string_, string_content_, string_length_);
}

DiagGUI::DiagGUI(communication::UserInterface* UI, int16_t diag_X, int16_t diag_Y) : UI_(UI) {
  diag_X_ = diag_X;
  diag_Y_ = diag_Y;
  count_ = 0;
  for (uint8_t i = 0; i < 25; i++) {
    diag_string_[i] = nullptr;
  }
}

DiagGUI::~DiagGUI() {
  for (uint8_t i = 0; i < 25; i++) {
    if (diag_string_[i] != nullptr) {
      diag_string_[i]->Delete();
      delete diag_string_[i];
      diag_string_[i] = nullptr;
    }
  }
}

void DiagGUI::Update(char* String, delay_t delay_function, int8_t color) {
  if (count_ >= 25) {
    return;
  }
  delay_function(100);
  if (color == -1)
    color = UI_Color_Main;
  char name[15];
  snprintf(name, 15, "M%d", count_);
  if (diag_string_[count_] == nullptr) {
    diag_string_[count_] =
        new StringGUI(UI_, String, diag_X_, diag_Y_ - count_ * 20, color, 10, name);
  }
  diag_string_[count_]->Init();
  count_++;
}
void DiagGUI::Clear(delay_t delay_func) {
  for (uint8_t i = 0; i < 25; i++) {
    delay_func(100);
    graphic_data_t temp;
    char name[15];
    snprintf(name, 15, "M%d", i);
    UI_->CharDraw(&temp, name, UI_Graph_Del, 4, UI_Color_Main, 10, 1, 2, diag_X_, diag_Y_);
    UI_->GraphRefresh(1, temp);

    delete diag_string_[i];
    diag_string_[i] = nullptr;
  }
  count_ = 0;
}

}  // namespace communication