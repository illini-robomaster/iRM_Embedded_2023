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

#include <map>

#include "bsp_error_handler.h"
#include "can.h"

#define MAX_CAN_DATA_SIZE 8
#define MAX_CAN_DEVICES 12

namespace bsp {

/* can callback function pointer */
typedef void (*can_rx_callback_t)(const uint8_t data[], void* args);

class CAN {
 public:
  /**
   * @brief constructor for bsp CAN instance
   *
   * @param hcan     HAL can handle
   * @param start_id lowest possible stdid for rx
   */
  CAN(CAN_HandleTypeDef* hcan, uint32_t start_id, bool is_master = true);
  /**
   * @brief check if it is associated with a given CAN handle
   *
   * @param hcan  HAL can handle to be checked
   *
   * @return true if associated, otherwise false
   */
  bool Uses(CAN_HandleTypeDef* hcan) { return hcan_ == hcan; }

  /**
   * @brief register callback function for a specific ID on this CAN line
   *
   * @param std_id    rx id
   * @param callback  callback function
   * @param args      argument passed into the callback function
   *
   * @return return 0 if success, -1 if invalid std_id
   */
  int RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args = NULL);

  /**
   * @brief transmit can messages
   *
   * @param id      tx id
   * @param data[]  data bytes
   * @param length  length of data, must be in (0, 8]
   *
   * @return  number of bytes transmitted, -1 if failed
   */
  int Transmit(uint16_t id, const uint8_t data[], uint32_t length);

  /**
   * @brief callback wrapper called from IRQ context
   *
   * @note should not be called explicitly form the application side
   */
  void RxCallback();

 private:
  void ConfigureFilter(bool is_master);

  CAN_HandleTypeDef* hcan_;
  const uint32_t start_id_;

  can_rx_callback_t rx_callbacks_[MAX_CAN_DEVICES] = {0};
  void* rx_args_[MAX_CAN_DEVICES] = {NULL};

  static std::map<CAN_HandleTypeDef*, CAN*> ptr_map;
  static CAN* FindInstance(CAN_HandleTypeDef* hcan);
  static bool HandleExists(CAN_HandleTypeDef* hcan);
  static void RxFIFO0MessagePendingCallback(CAN_HandleTypeDef* hcan);
};

} /* namespace bsp */
