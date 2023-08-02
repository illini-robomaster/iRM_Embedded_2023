#include "chassis_task.h"



void chassisTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

//TODO: IMU???
//   while (!imu->CaliDone()) osDelay(100);

  float vx_keyboard = 0, vy_keyboard = 0;
  float vx_remote, vy_remote;
  float vx_set, vy_set;

  while (true) {

    if (dbus->keyboard.bit.A) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard += 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard -= 61.5;

    if (-35 <= vx_keyboard && vx_keyboard <= 35) vx_keyboard = 0;
    if (-35 <= vy_keyboard && vy_keyboard <= 35) vy_keyboard = 0;

    if (vx_keyboard > 0)
      vx_keyboard -= 60;
    else if (vx_keyboard < 0)
      vx_keyboard += 60;

    if (vy_keyboard > 0)
      vy_keyboard -= 60;
    else if (vy_keyboard < 0)
      vy_keyboard += 60;

    vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
    vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);

    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;

    vx_set = vx_keyboard + vx_remote;
    vy_set = vy_keyboard + vy_remote;

    send->cmd.id = bsp::VX;
    send->cmd.data_float = Dead ? 0 : vx_set;
    send->TransmitOutput();

    send->cmd.id = bsp::VY;
    send->cmd.data_float = Dead ? 0 : vy_set;
    send->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);
  }
}

void init_chassis(){
    
}
void kill_chassis();
