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

#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

namespace bsp {

class VirtualUSB {
 public:
  /**
   * @brief constructor for usb instance
   *
   * @param true if use callback for receiving, otherwise manual read is required. The
   *        default value is false
   */
  explicit VirtualUSB();

  /**
   * @brief destructor (potentially deallocate buffer memories associated with tx / rx)
   */
  virtual ~VirtualUSB();

  /**
   * @brief set up usb non blocking transmission
   *
   * @param tx_buffer_size  transmission buffer size (burst transmission calls will
   *                        be queued into this buffer)
   */
  void SetupTx(uint32_t tx_buffer_size);

  /**
   * @brief set up usb receiver
   *
   * @param rx_buffer_size  receive buffer size (all data that has not been read out is
   *                        queued into this buffer if there is no callbacks registered)
   */
  void SetupRx(uint32_t rx_buffer_size);

  /**
   * @brief read out the pending received data
   *
   * @param data  reference to an array address that gets set to the receive buffer address
   *
   * @return number of bytes read
   *
   * @note memory is not copied for optimal performance, so second call to this
   *       method will invalidate the buffer produced by the previous call
   */
  uint32_t Read(uint8_t** data);

  /**
   * @brief write data to usb without blocking
   *
   * @param data    pointer to the data buffer to be transmitted
   * @param length  length of the data to be transmitted
   *
   * @return number of bytes written
   *
   * @note multiple burst calls to this function can potentially cause tx buffer
   *       to fill up, so remember to check return value for the actual number
   *       of bytes successfully transmitted
   */
  uint32_t Write(uint8_t* data, uint32_t length);

 protected:
  /**
   * @brief Transmission complete callback function
   */
  void TxCompleteCallback();

  /**
   * @brief Reception complete call back
   *
   * @param data    pointer to the data buffer received
   * @param length  length of the data received
   */
  virtual void RxCompleteCallback();

  /**
   * @brief Queue up received data if callback is not used
   *
   * @param data    pointer to the data buffer received
   * @param length  length of the data received
   *
   * @return number of bytes actually written to the buffer
   */
  uint32_t QueueUpRxData(const uint8_t* data, uint32_t length);

 public:
  /**
   * @brief Wrapper function of transmission complete callback
   *
   * @param data    pointer to the data buffer received
   * @param length  length of the data received
   */
  friend void TxCompleteCallbackWrapper();

  /**
   * @brief Wrapper function of reception complete callback
   *
   * @param data    pointer to the data buffer received
   * @param length  length of the data received
   */
  friend void RxCompleteCallbackWrapper(uint8_t* data, uint32_t length);

 protected:
  /* rx */
  uint32_t rx_size_;
  uint32_t rx_pending_;
  uint8_t* rx_write_;
  uint8_t* rx_read_;
  /* tx */
  uint32_t tx_size_;
  uint32_t tx_pending_;
  uint8_t* tx_write_;
  uint8_t* tx_read_;
};

} /* namespace bsp */
