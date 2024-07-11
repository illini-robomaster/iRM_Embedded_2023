/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

#include "uiTask.h"

communication::UserInterface* UI = nullptr;

communication::graphic_data_t curr_pitch_msg;
communication::graphic_data_t curr_pitch_val;

communication::graphic_data_t target_pitch_msg;
communication::graphic_data_t target_pitch_val;

communication::graphic_data_t curr_yaw_msg;
communication::graphic_data_t curr_yaw_val;


void UI_task(void* args) {
  UNUSED(args);
  communication::package_t frame;

  float current_pitch = 0;
  float suggested_pitch = 0;
  float current_yaw = 0;

  while (referee->connection_flag_ == false) {
    osDelay(UI_DELAY);
  }

  UI->SetID(referee->game_robot_status.robot_id);
  osDelay(UI_DELAY);
  char curr_pitch_message[20] = "CURRENT PITCH:";
  UI->CharDraw(&curr_pitch_msg,"CPM", UI_Graph_Add, 1, UI_Color_Orange, 30,strlen(curr_pitch_message),2,600,640);
  char suggested_pitch_message[20]="TARGET PITCH:";
  UI->CharDraw(&target_pitch_msg, "TPM", UI_Graph_Add, 1, UI_Color_Green, 30, strlen(suggested_pitch_message), 2, 600, 600);
  char curr_yaw_message[20] = "CURRENT YAW:";
  UI->CharDraw(&curr_yaw_msg, "CYM", UI_Graph_Add, 1, UI_Color_Orange, 30, strlen(curr_yaw_message), 2, 1150, 640);
  uint8_t *buffer = (uint8_t *)malloc(100);
  UI->CharRefresh(buffer,curr_pitch_msg,curr_pitch_message,strlen(curr_pitch_message));
  osDelay(UI_DELAY);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data,frame.length);
  UI->CharRefresh(buffer, target_pitch_msg, suggested_pitch_message, strlen(suggested_pitch_message));
  osDelay(UI_DELAY);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data,frame.length);
  UI->CharRefresh(buffer, curr_yaw_msg, curr_yaw_message, strlen(curr_yaw_message));
  osDelay(UI_DELAY);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data,frame.length);


  while (true) {
    // TODO: need to update after calculation of rotated euler angles is done
    // current_pitch = pitch_curr;
    // suggested_pitch = pitch_target;
    UI->FloatDraw(&curr_pitch_val,"CPV", UI_Graph_Add, 1,
                  UI_Color_Orange,30, 2, 10,
                  670, 640,current_pitch);
    UI->FloatDraw(&target_pitch_val, "TPV", UI_Graph_Add, 1,
                  UI_Color_Green, 30, 2, 10, 670,
                  600, suggested_pitch);
    UI->FloatDraw(&curr_yaw_val, "CYV", UI_Graph_Add, 1,
                  UI_Color_Orange, 30, 2, 10, 1220,
                  640, current_yaw);

    UI->GraphRefresh((uint8_t*) &referee->graphic_double, 2,
                     curr_pitch_val,curr_yaw_val);
    referee->PrepareUIContent(communication::DOUBLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);

    UI->GraphRefresh((uint8_t*)&referee->graphic_single,1,target_pitch_val);
    referee->PrepareUIContent(communication::SINGLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);

    osDelay(UI_DELAY);
  }
}

void init_ui() {
  UI = new communication::UserInterface;
}
