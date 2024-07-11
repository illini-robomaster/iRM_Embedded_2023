/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

#pragma once

#include "motor.h"
#include "power_limit.h"
#include "controller.h"
#include "bsp_os.h"
#include "utils.h"


namespace control{

    typedef struct{
        MotorCANBase* motor;      /* motor instance to be wrapped as a servomotor      */
        float max_speed;          /* desired turning speed of motor shaft, in [rad/s]  */
        float max_acceleration;   /* desired acceleration of motor shaft, in [rad/s^2] */
        float transmission_ratio; /* transmission ratio of motor                       */
        float* omega_pid_param;   /* pid parameter used to control speed of motor      */
        float max_iout;
        float max_out;
        float install_offset;
    }steering6020_t;

    class Steering6020{
    public:

        Steering6020(steering6020_t data);

        ~Steering6020();

        void UpdateData(const uint8_t data[]);

        void CalcOutput();

        servo_status_t SetTarget(float target ,bool override=true);

        void SetMaxSpeed(float max_speed);

        void SetMaxAcceleration(float max_acceleration);

        float GetTarget();
        float GetTheta();

        //TODO: Tried to used in detecting turing behavior, might be useful in future.
        bool inPosition();

        void PrintData();




    private:
        MotorCANBase* motor_;
        float target_angle_;
        float current_angle_;
        float corrected_current_angle_;
        float corrected_target_angle_;

        bool hold_;
        uint32_t start_time_;

        float install_offset_;

        float max_speed_;
        float max_acceleration_;
        float max_iout_;
        float max_out_;
        float transmission_ratio_; /* transmission ratio of motor                       */
        float proximity_in_;
        float proximity_out_;

        ConstrainedPID omega_pid_;

        BoolEdgeDetector* hold_detector_;
    };

}