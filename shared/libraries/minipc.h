/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "crc8.h"

namespace communication {

class MiniPCProtocol {
 public:
  MiniPCProtocol();
  void Receive(const uint8_t* data, uint8_t len);
  // dummy send
  void Send();
  uint8_t get(void);

 private:
  int index;
  static constexpr uint8_t PKG_LEN = 17;
  uint8_t flag;
  uint8_t host_command[PKG_LEN];
  void handle();
}; /* class MiniPCProtocol */

} /* namespace communication */
