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


// float pid_params[3]{10000,1500,40000};           /* PID Params */

float pid_params[3]{10000,200,40000};           /* PID Params */

#ifdef CONTROLLER
  remote::DBUS* dbus = nullptr;             /* Controller */
#endif

void RM_RTOS_Init() {
    print_use_uart(&huart1);                /* Print through *huart*/

    /* Init */
    can1 = new bsp::CAN(&hcan1, true);
    motor =  new control::Motor3510(can1,0x207);

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
    float curr_time_s = 0; //current timestamp
    float last_time_s = xTaskGetTickCount() * (1.0/configTICK_RATE_HZ); //last loop's timestamp
    float delta_time_s = 0; //loop delta time
    float target_angular_vel = 0; //target m3510 angular_vel

    int count = 0;

    while (true) {
#ifdef CONTROLLER
        curr_time_s = xTaskGetTickCount() * (1.0/configTICK_RATE_HZ);
        delta_time_s = curr_time_s - last_time_s;
        last_time_s = curr_time_s;
        target_angular_vel = float(dbus->ch1) / remote::DBUS::ROCKER_MAX * PI; //maximum 1 PI/s rate change
        target += target_angular_vel * delta_time_s;
        target = clip<float>(target,0,2*PI); //angle limit for the gimbal
#endif  
        
        //Calculate error
        diff = motor->GetThetaDelta(target);
        out = pid.ComputeOutput(diff);

        //Control Motor 
        motor->SetOutput(out);
        // motor->SetOutput(0);

        control::MotorCANBase::TransmitOutput(motors, 1);

        //Check output
    
        count ++;

        if(count == 10){

            set_cursor(0, 0);
            clear_screen();
            print("target: %.4f, diff in degree: %.4f, target_angular_vel: %.4f \r\n",target,diff/PI*180.0, target_angular_vel);
            print("PID output: %.f \r\n", out);
            motor->PrintData();
            count = 0;
#ifdef CONTROLLER
            print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
            print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);
            print("time: %d s", curr_time_s);
#endif
        }

        osDelay(10);

    }
}