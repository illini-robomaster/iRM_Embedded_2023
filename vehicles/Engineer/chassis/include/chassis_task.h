#pragma once

#include "cmsis_os.h"
#include "motor.h"
#include "engineer_steering.h"
#include "dbus.h"
#include "can.h"
#include "rgb.h"

extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();


extern remote::DBUS* dbus;
extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern display::RGB* RGB;


// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);
// speed for chassis rotation (no unit)
constexpr float FOLLOW_SPEED = 40;
constexpr float CHASSIS_DEADZONE = 0.04;
