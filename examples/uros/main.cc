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

#include "main.h"

#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

/* init new task START */
static osThreadId_t uROSTaskHandle;

const osThreadAttr_t uROSTask_attributes = {.name = "uROSTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 4096 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};

extern "C" bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern "C" bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern "C" size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern "C" size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
extern "C" void * microros_allocate(size_t size, void * state);
extern "C" void microros_deallocate(void * pointer, void * state);
extern "C" void * microros_reallocate(void * pointer, size_t size, void * state);
extern "C" void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void uROSTask(void* argument) {
  UNUSED(argument);

  rmw_uros_set_custom_transport(true, nullptr, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

  rcl_allocator_t rtos_allocator = rcutils_get_zero_initialized_allocator();
  rtos_allocator.allocate = microros_allocate;
  rtos_allocator.deallocate = microros_deallocate;
  rtos_allocator.reallocate = microros_reallocate;
  rtos_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&rtos_allocator)) {
    // error handling
  }

  // initialize
  rclc_support_t support;
  rcl_allocator_t allocator;
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, nullptr, &allocator);

  // create node
  rcl_node_t node;
  rclc_node_init_default(&node, "test_node", "", &support);

  // create publisher
  rcl_publisher_t publisher;
  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "test_topic");

  // publish loop
  std_msgs__msg__Int32 msg;
  msg.data = 0;
  while (true) {
    if (rcl_publish(&publisher, &msg, nullptr) != RCL_RET_OK) {
      // error handling
    }
    msg.data++;
    osDelay(10);
  }
}

void RM_RTOS_Threads_Init(void) {
  uROSTaskHandle = osThreadNew(uROSTask, nullptr, &uROSTask_attributes);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    osDelay(500);
  }
}
