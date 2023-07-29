

#include "main.h"
#include "printf.h"

#include "cmsis_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "protocol.h"
#include "user_interface.h"


static const int UI_TASK_DELAY = 20;
static communication::UserInterface* UI = nullptr;

//==================================================================================================
// REFEREE
//==================================================================================================


#define RX_SIGNAL (1 << 0)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static CustomUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}


void RM_RTOS_Init(void){

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  UI = new communication::UserInterface();
}

void RM_RTOS_Threads_Init(void){
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}


void RM_RTOS_Default_Task(const void *argument){
  UNUSED(argument);
  osDelay(3000);

  UI->SetID(referee->game_robot_status.robot_id);

  communication::package_t frame;
  communication::graphic_data_t graphText;
  communication::graphic_data_t graphCrosshair1;
  communication::graphic_data_t graphCrosshair2;
  communication::graphic_data_t graphCrosshair3;
  communication::graphic_data_t graphCrosshair4;
  communication::graphic_data_t graphCrosshair5;
  communication::graphic_data_t graphCrosshair6;
  communication::graphic_data_t graphCrosshair7;


  // Initialize crosshair GUI
  UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                  &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
  UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                  graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                  graphCrosshair7);
  referee->PrepareUIContent(communication::SEVEN_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);


  // Initialize self-diagnosis GUI
  char msgBuffer[30] = "TEST MSG";
  UI->DiagGUIInit(&graphText, 30);
  UI->CharRefresh((uint8_t*)(&referee->graphic_character), graphText, msgBuffer, sizeof msgBuffer);
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  int j = 0;
  while(true){
    snprintf(msgBuffer,30,"TEST MSG%d",j);
    UI->CharRefresh((uint8_t*)(&referee->graphic_character),graphText,msgBuffer,sizeof msgBuffer);
    referee->PrepareUIContent(communication::SINGLE_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data,frame.length);
    j++;
    osDelay(UI_TASK_DELAY);
  }

}