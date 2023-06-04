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

#include "bsp_can.h"

#include "bsp_error_handler.h"
#include "cmsis_os.h"

namespace bsp {

std::map<CAN_HandleTypeDef*, CAN*> CAN::ptr_map;

/**
 * @brief find instantiated can line
 *
 * @param hcan  HAL can handle
 *
 * @return can instance if found, otherwise NULL
 */
CAN* CAN::FindInstance(CAN_HandleTypeDef* hcan) {
  const auto it = ptr_map.find(hcan);
  if (it == ptr_map.end()) return nullptr;

  return it->second;
}

/**
 * @brief check if any associated CAN instance is instantiated or not
 *
 * @param hcan  HAL can handle
 *
 * @return true if found, otherwise false
 */
bool CAN::HandleExists(CAN_HandleTypeDef* hcan) { return FindInstance(hcan) != nullptr; }

/**
 * @brief callback handler for CAN rx feedback data
 *
 * @param hcan  HAL can handle
 */
void CAN::RxFIFO0MessagePendingCallback(CAN_HandleTypeDef* hcan) {
  CAN* can = FindInstance(hcan);
  if (!can) return;
  can->RxCallback();
}

CAN::CAN(CAN_HandleTypeDef* hcan, uint32_t start_id, bool is_master)
    : hcan_(hcan), start_id_(start_id) {
  RM_ASSERT_FALSE(HandleExists(hcan), "Repeated CAN initialization");
  ConfigureFilter(is_master);
  // activate rx interrupt
  RM_ASSERT_HAL_OK(HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
                                            RxFIFO0MessagePendingCallback),
                   "Cannot register CAN rx callback");
  RM_ASSERT_HAL_OK(HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING),
                   "Cannot activate CAN rx message pending notification");
  RM_ASSERT_HAL_OK(HAL_CAN_Start(hcan), "Cannot start CAN");

  // save can instance as global pointer
  ptr_map[hcan] = this;
}

int CAN::RegisterRxCallback(uint32_t std_id, can_rx_callback_t callback, void* args) {
  int callback_id = std_id - start_id_;

  if (callback_id < 0 || callback_id >= MAX_CAN_DEVICES) return -1;

  rx_args_[callback_id] = args;
  rx_callbacks_[callback_id] = callback;

  return 0;
}

int CAN::Transmit(uint16_t id, const uint8_t data[], uint32_t length) {
  RM_EXPECT_TRUE(IS_CAN_DLC(length), "CAN tx data length exceeds limit");
  if (!IS_CAN_DLC(length)) return -1;

  CAN_TxHeaderTypeDef header = {
      .StdId = id,
      .ExtId = 0x0,  // don't care since we use standard id mode
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
      .DLC = length,
      .TransmitGlobalTime = DISABLE,
  };

  uint32_t mailbox;

  if (HAL_CAN_AddTxMessage(hcan_, &header, (uint8_t*)data, &mailbox) != HAL_OK) return -1;

  // poll for can transmission to complete
  while (HAL_CAN_IsTxMessagePending(hcan_, mailbox))
    ;

  return length;
}

void CAN::RxCallback() {
  CAN_RxHeaderTypeDef header;
  uint8_t data[MAX_CAN_DATA_SIZE];
  HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &header, data);
  int callback_id = header.StdId - start_id_;
  // find corresponding callback
  if (callback_id >= 0 && callback_id < MAX_CAN_DEVICES && rx_callbacks_[callback_id])
    rx_callbacks_[callback_id](data, rx_args_[callback_id]);
}

void CAN::ConfigureFilter(bool is_master) {
  CAN_FilterTypeDef CAN_FilterConfigStructure;
  /* Configure Filter Property */
  CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfigStructure.FilterActivation = ENABLE;
  CAN_FilterConfigStructure.SlaveStartFilterBank = 14;  // CAN1 and CAN2 split all 28 filters
  /* Configure each CAN bus */
  if (is_master)
    CAN_FilterConfigStructure.FilterBank = 0;  // Master CAN get filter 0-13
  else
    CAN_FilterConfigStructure.FilterBank = 14;  // Slave CAN get filter 14-27

  RM_EXPECT_HAL_OK(HAL_CAN_ConfigFilter(hcan_, &CAN_FilterConfigStructure),
                   "CAN filter configuration failed.");
}

} /* namespace bsp */
