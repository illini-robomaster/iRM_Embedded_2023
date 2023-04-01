#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"
#include "stdio.h"

#define TARGET_SPEED 30
static remote::DBUS* dbus;
bsp::CAN* can = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x201, true);
  fl_motor = new control::Motor3508(can, 0x201);
  fr_motor = new control::Motor3508(can, 0x202);
  bl_motor = new control::Motor3508(can, 0x203);
  br_motor = new control::Motor3508(can, 0x204);
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {fl_motor,fr_motor,bl_motor,br_motor};
  control::PIDController pidfl(20, 15, 30);
  control::PIDController pidfr(20, 15, 30);
  control::PIDController pidbl(20, 15, 30);
  control::PIDController pidbr(20, 15, 30);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
    print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);
    //Wheels speed calculation
    int fl_speed = dbus->ch2+dbus->ch1+dbus->ch0;
    int fr_speed = dbus->ch2-dbus->ch1+dbus->ch0;
    int bl_speed = dbus->ch2+dbus->ch1-dbus->ch0;
    int br_speed = dbus->ch2-dbus->ch1-dbus->ch0;
    //Front left wheel
    float fl_diff = fl_motor->GetOmegaDelta(fl_speed);
    int16_t fl_out = pidfl.ComputeConstrainedOutput(fl_diff);
    fl_motor->SetOutput(fl_out);
    //Front right wheel
    float fr_diff = fr_motor->GetOmegaDelta(fr_speed);
    int16_t fr_out = pidfr.ComputeConstrainedOutput(fr_diff);
    fr_motor->SetOutput(fr_out);
    //Back left wheel
    float bl_diff = bl_motor->GetOmegaDelta(bl_speed);
    int16_t bl_out = pidbl.ComputeConstrainedOutput(bl_diff);
    bl_motor->SetOutput(bl_out);
    //Back right wheel
    float br_diff = br_motor->GetOmegaDelta(br_speed);
    int16_t br_out = pidbr.ComputeConstrainedOutput(br_diff);
    br_motor->SetOutput(br_out);
    //printf("%d%d%d%d",fl_out,fr_out,bl_out, br_out);
    control::MotorCANBase::TransmitOutput(motors, 4);
    //motor->PrintData();
    osDelay(10);
  }
}