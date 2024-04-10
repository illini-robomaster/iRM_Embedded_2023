
#pragma once

#include "chassis.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"

extern osThreadId_t shooterTaskHandle;
const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0};


extern remote::DBUS* dbus;
extern bsp::CAN* can1;
extern bsp::CAN* can2;

extern BoolEdgeDetector lob_mode_sw;
extern volatile bool lob_mode;

void shooterTask(void* arg);
void init_shooter();
void kill_shooter();