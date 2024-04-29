#include "referee_task.h"





void refereeTask(void* arg) {
 UNUSED(arg);
 uint32_t length;
 uint8_t* data;

 while (true) {
   uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
   if (flags & REFEREE_RX_SIGNAL) {
     length = referee_uart->Read(&data);
     referee->Receive(communication::package_t{data, (int)length});
   }
 }
}

void init_referee(){

}

void kill_referee(){
  
}
