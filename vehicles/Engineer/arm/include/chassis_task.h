#include "cmsis_os.h"
#include "dbus.h"
#include "utils.h"
#include "can.h"
#include "bsp_can_bridge.h"

static const int CHASSIS_TASK_DELAY = 2;


extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};


void chassisTask(void* arg);
void init_chassis();
void kill_chassis();



extern bsp::CanBridge* send;
extern remote::DBUS* dbus;
extern volatile bool Dead;
