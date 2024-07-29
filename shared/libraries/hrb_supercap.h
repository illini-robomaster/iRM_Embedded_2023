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
#include "bsp_can.h"

namespace control {

typedef struct {
  float energy;
} __packed cap_recv_message_t;

typedef struct {
    float referee_power_limit;
} __packed cap_send_message_t;

class HRB_SuperCap {
    public:
        HRB_SuperCap(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        void ReceiveData(const uint8_t* data);
        void SendData(cap_send_message_t data);

        volatile bool connection_flag_ = false;

        cap_recv_message_t info;
    private:
        bsp::CAN* can_;
        uint16_t rx_id_;
        uint16_t tx_id_;
};

}  // namespace control
