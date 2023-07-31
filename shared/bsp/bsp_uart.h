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

#include <map>

#include "usart.h"

namespace bsp {

class UART {
 public:
  /**
   * @brief constructor for uart instance
   *
   * @param huart pointer to a HAL uart handle
   */
  explicit UART(UART_HandleTypeDef* huart);

  /**
   * @brief destructor (potentially deallocate buffer memories associated with tx / rx)
   */
  virtual ~UART();

  /**
   * @brief set up uart receiver in the background optionally registering a callback
   *
   * @param rx_buffer_size  receive buffer size (all data that has not been read
   *                        out is queued into this buffer)
   */
  void SetupRx(uint32_t rx_buffer_size);

  /**
   * @brief set up non blocking transmission functionality
   *
   * @param tx_buffer_size  transmission buffer size (burst transmission calls will
   *                        be queued into this buffer)
   */
  void SetupTx(uint32_t tx_buffer_size);

  /**
   * @brief read out the pending received data
   *
   * @tparam FromISR  set to true to call inside an interrupt handler
   * @param data  pointer to an array address that gets set to the receive buffer address
   *
   * @return number of bytes read, -1 if failure
   *
   * @note memory is not copied for optimal performance, so second call to this
   *       method will invalidate the buffer produced by the previous call
   */
  template <bool FromISR = false>
  int32_t Read(uint8_t** data);

  /**
   * @brief write data to uart without blocking
   *
   * @tparam FromISR  set to true to call inside an interrupt handler
   * @param data    pointer to the data buffer to be transmitted
   * @param length  length of the data to be transmitted
   *
   * @return number of bytes written
   *
   * @note multiple burst calls to this function can potentially cause tx buffer
   *       to fill up, so remember to check return value for the actual number
   *       of bytes successfully transmitted
   */
  template <bool FromISR = false>
  int32_t Write(const uint8_t* data, uint32_t length);

 protected:
  /**
   * @brief Transmission complete call back.
   */
  void TxCompleteCallback();

  /**
   * @brief Reception complete call back.
   */
  virtual void RxCompleteCallback();

  UART_HandleTypeDef* huart_;
  /* rx */
  uint32_t rx_size_;
  uint8_t* rx_data_[2];
  uint8_t rx_index_;
  /* tx */
  uint32_t tx_size_;
  uint32_t tx_pending_;
  uint8_t* tx_write_;
  uint8_t* tx_read_;

 private:
  friend void RxCompleteCallbackWrapper(UART_HandleTypeDef* huart);
  friend void TxCompleteCallbackWrapper(UART_HandleTypeDef* huart);

  static std::map<UART_HandleTypeDef*, UART*> ptr_map;
  static UART* FindInstance(UART_HandleTypeDef* huart);
};

} /* namespace bsp */
