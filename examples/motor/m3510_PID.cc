/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
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

// #define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "dbus.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define CONTROLLER

bsp::CAN* can1 = nullptr;                   /*CAN*/
control::MotorCANBase* motor = nullptr;     /* 3510Motor*/

// BoolEdgeDetector key_detector(false);    /* Edge Detector*/


float pid_params[3]{1500,30,300};           /* PID Params */

#ifdef CONTROLLER
  remote::DBUS* dbus = nullptr;             /* Controller */
#endif

void RM_RTOS_Init() {
    print_use_uart(&huart1);                /* Print through *huart*/


    /* Init */
    can1 = new bsp::CAN(&hcan1, true);
    motor =  new control::Motor3510(can1,0x205);

#ifdef CONTROLLER
    dbus = new remote::DBUS(&huart3);
#endif


}

void RM_RTOS_Default_Task(const void* args){
    UNUSED(args);

    /* PID Controller */
    control::ConstrainedPID pid;  
    pid.Reinit(pid_params,3000,30000); 

    /* Motors */
    control::MotorCANBase* motors[] = {motor};
    
#ifdef CONTROLLER
    /* 500 ticks delay waiting for controller init */
    osDelay(500);
#endif

    /* Params Initialization */
    float diff = 0; /* Position Difference */
    float out = 0;  /* PID Controller Output */
    float target = 3.14; /* Target Position */
 


    while (true) {
#ifdef CONTROLLER
        target = clip<float>(float(dbus->ch1) / remote::DBUS::ROCKER_MAX * PI,-PI,PI);
#endif

        //Calculate error
        diff = motor->GetThetaDelta(target);
        out = pid.ComputeOutput(diff);

        //Control Motor 
        motor->SetOutput(out);
        control::MotorCANBase::TransmitOutput(motors, 1);

        //Check output
        print("target: %.4f, diff: %.4f \r\n",target,diff);
        motor->PrintData();
        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
        print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);
        osDelay(100);

    }
}