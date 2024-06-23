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


#include "bsp_uart.h"
#include "utils.h"
#include "usart.h"


namespace distance {

    typedef struct{
        uint8_t command[4];
    }__packed SendPacket_4_t;

    typedef struct{
        uint8_t command[5];
    }__packed SendPacket_5_t;

    typedef struct{
        uint8_t command[6];
    }__packed SendPacket_6_t;

    typedef void (*sen_0366_delay_t)(uint32_t milli);

    void sen_0366_config(UART_HandleTypeDef *huart);

    class SEN_0366_DIST : public bsp::UART {
    public:


        SEN_0366_DIST(UART_HandleTypeDef *huart, uint8_t ADDR, MovingAverageFilter<double> maf);

        static SEN_0366_DIST* init(UART_HandleTypeDef *huart, uint8_t ADDR);

        uint8_t address;
        MovingAverageFilter<double>maf;

        bool begin();
        bool setMeasureRange(int8_t range);
        bool laserOn();
        void readValue();
        bool setResolution(float resolution);
        bool shutdown();

        double distance;

    private:
        sen_0366_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); };
        SendPacket_4_t single_measurement_packet;
        SendPacket_4_t continuous_measurement_packet;
        SendPacket_4_t shutdown_packet;
        SendPacket_5_t laser_on_packet;
        SendPacket_5_t set_measure_distance_packet;
        SendPacket_5_t set_frequency_packet;
        SendPacket_5_t set_resolution_packet;
    };
};
