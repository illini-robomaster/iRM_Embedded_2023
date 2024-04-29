#pragma once
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "protocol.h"

#define REFEREE_RX_SIGNAL (1 << 1)

extern osThreadId_t refereeTaskHandle;

class RefereeUART : public bsp::UART {
public:
 using bsp::UART::UART;

protected:
 void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 1024 * 4,
                                            .priority = (osPriority_t)osPriorityAboveNormal,
                                            .tz_module = 0,
                                            .reserved = 0};


void refereeTask(void* args);

extern communication::Referee* referee;
extern RefereeUART* referee_uart;
