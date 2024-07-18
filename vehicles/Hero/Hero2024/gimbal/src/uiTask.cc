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

communication::package_t frame;

float current_pitch = 0;
float target_pitch = 0;
float current_yaw = 0;

uint32_t ui_val_graph_size = 10;
uint32_t ui_val_width = 1;
uint32_t ui_val_char_length = 15;



void UI_task(void* args) {
  UNUSED(args);




  while (!(referee->connection_flag_)) {
    print("Waiting for referee system to connect\r\n");
    osDelay(1000);
  }
  while(dbus->swr!=remote::DOWN){
    osDelay(100);
  }


  UI->SetID(referee->game_robot_status.robot_id);
  print("UI task started, ID: %d\r\n",referee->game_robot_status.robot_id);
  osDelay(UI_TASK_DELAY);

  refresh();

  while (true) {
    // TODO: need to update after calculation of rotated euler angles is done
    current_pitch = dbus->ch3;
    target_pitch = dbus->ch0;
    current_yaw = dbus->ch2;

    char current_pitch_value[20];

    sprintf(current_pitch_value, "%.3f", current_pitch);

    UI->CharDraw(&curr_pitch_val,"CPV", UI_Graph_Change, 1,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length, ui_val_width,
                 710, 640);

    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    curr_pitch_val, current_pitch_value, strlen(current_pitch_value));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);

    osDelay(UI_TASK_DELAY);

    UI->CharDraw(&target_pitch_val, "TPV", UI_Graph_Change, 1,
                 UI_Color_Green, ui_val_graph_size, ui_val_char_length, ui_val_width, 730,
                 600);

    char target_pitch_value[20];
    sprintf(target_pitch_value, "%.3f", target_pitch);

    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    target_pitch_val, target_pitch_value, strlen(target_pitch_value));

    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);

    referee_uart->Write(frame.data, frame.length);

    osDelay(UI_TASK_DELAY);

    UI->CharDraw(&curr_yaw_val, "CYV", UI_Graph_Change, 1,
                 UI_Color_Orange, ui_val_graph_size, ui_val_char_length, ui_val_width, 1240,
                 640);

    char current_yaw_value[20];
    sprintf(current_yaw_value, "%.3f", current_yaw);

    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    curr_yaw_val, current_yaw_value, strlen(current_yaw_value));

    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);

    referee_uart->Write(frame.data, frame.length);


    osDelay(UI_TASK_DELAY);
//    print("UI task running\r\n");
    if(dbus->keyboard.bit.CTRL){
      refresh();
    }
  }
}

void refresh(){
  char curr_pitch_message[20] = "CURR PITCH:";
  UI->CharDraw(&curr_pitch_msg,"CPM", UI_Graph_Add, 1, UI_Color_Orange, 10,strlen(curr_pitch_message),2,600,640);



  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  curr_pitch_msg, curr_pitch_message, strlen(curr_pitch_message));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);

  


  char suggested_pitch_message[20]="TARGET PITCH:";
  UI->CharDraw(&target_pitch_msg, "TPM", UI_Graph_Add, 1, UI_Color_Green, 10, 15, 2, 600, 600);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  target_pitch_msg, suggested_pitch_message, strlen(suggested_pitch_message));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  
  referee_uart->Write(frame.data, frame.length);

  char curr_yaw_message[20] = "CURR YAW:";
  UI->CharDraw(&curr_yaw_msg, "CYM", UI_Graph_Add, 1, UI_Color_Orange, 10, 15, 2, 1150, 640);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  curr_yaw_msg, curr_yaw_message, strlen(suggested_pitch_message));


  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);

  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  char current_pitch_value[20];

  sprintf(current_pitch_value, "%.3f", current_pitch);

  UI->CharDraw(&curr_pitch_val,"CPV", UI_Graph_Add, 1,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length, ui_val_width,
               710, 640);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  curr_pitch_val, current_pitch_value, strlen(current_pitch_value));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  
  referee_uart->Write(frame.data, frame.length);

  osDelay(UI_TASK_DELAY);

  UI->CharDraw(&target_pitch_val, "TPV", UI_Graph_Add, 1,
               UI_Color_Green, ui_val_graph_size, ui_val_char_length, ui_val_width, 730,
               600);

  char target_pitch_value[20];
  sprintf(target_pitch_value, "%.3f", target_pitch);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  target_pitch_val, target_pitch_value, strlen(target_pitch_value));

  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  
  referee_uart->Write(frame.data, frame.length);

  osDelay(UI_TASK_DELAY);
  UI->CharDraw(&curr_yaw_val, "CYV", UI_Graph_Add, 1,
               UI_Color_Orange, ui_val_graph_size, ui_val_char_length, ui_val_width, 1240,
               640);

  char current_yaw_value[20];
  sprintf(current_yaw_value, "%.3f", current_yaw);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  curr_yaw_val, current_yaw_value, strlen(current_yaw_value));

  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  
  referee_uart->Write(frame.data, frame.length);

  osDelay(UI_TASK_DELAY);
//    print("UI task running\r\n");
}

void init_ui() {
  print_use_uart(&huart1);
  UI = new communication::UserInterface;
}
