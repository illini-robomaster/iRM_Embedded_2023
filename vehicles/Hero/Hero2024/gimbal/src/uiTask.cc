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

communication::graphic_data_t suggested_pitch_msg;
communication::graphic_data_t suggested_pitch_val;



void UI_task(void* args) {
  UNUSED(args);
    float current_pitch = 0;
    float suggested_pitch = 0;

  while (referee->connection_flag_ == false) {
    osDelay(UI_DELAY);
  }

  UI->SetID(referee->game_robot_status.robot_id);
  osDelay(UI_DELAY);
  char curr_pitch_message[20] = "Current Pitch:";
  UI->CharDraw(&curr_pitch_msg,"CPM", UI_Graph_Add, 1, UI_Color_Orange, 30,strlen(curr_pitch_message),2,600,640);
  char suggested_pitch_message[20]="Suggested Pitch:";
  UI->CharDraw(&suggested_pitch_msg,"TPM", UI_Graph_Add, 1, UI_Color_Orange, 30,strlen(suggested_pitch_message),2,600,600);
  uint8_t *buffer = (uint8_t *)malloc(100);
  UI->CharRefresh(buffer,curr_pitch_msg,curr_pitch_message,strlen(curr_pitch_message));
  UI->CharRefresh(buffer,suggested_pitch_msg,suggested_pitch_message,strlen(suggested_pitch_message));

  while (true) {
    // update parameters
    // current_pitch = pitch_curr;
    // suggested_pitch = pitch_target;
    UI->FloatDraw(&curr_pitch_val,"CPV", UI_Graph_Add, 1, UI_Color_Orange,30, 2, 10,  700, 640,current_pitch);
    UI->FloatDraw(&suggested_pitch_val,"TPV", UI_Graph_Add, 1, UI_Color_Orange,30, 2, 10,  700, 600,suggested_pitch);

    osDelay(UI_DELAY);
  }
}

void UI_Init() {
  UI = new communication::UserInterface;
}
