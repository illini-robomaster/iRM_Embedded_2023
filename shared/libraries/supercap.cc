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

#include "supercap.h"

#include <cstring>

namespace control {

static void supercap_callback(const uint8_t data[], const CAN_RxHeaderTypeDef& header, void* args) {
  (void)header;
  SuperCap* supercap = reinterpret_cast<SuperCap*>(args);
  supercap->UpdateData(data);
}

SuperCap::SuperCap(bsp::CAN* can, uint16_t rx_id) {
  can->RegisterRxCallback(rx_id, supercap_callback, this);
}

void SuperCap::UpdateData(const uint8_t* data) {
  memcpy(&info, data, sizeof(cap_message_t));
  connection_flag_ = true;
}

}  // namespace control
