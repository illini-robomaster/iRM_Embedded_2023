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

#include "sen_0366_dist.h"

namespace distance{

    void sen_0366_config(UART_HandleTypeDef *huart){
      HAL_UART_DeInit(huart);
      huart->Init.BaudRate = 9600;
      huart->Init.StopBits = UART_STOPBITS_1;
      huart->Init.Parity = UART_PARITY_ODD;
      HAL_UART_Init(huart);
    }
    SEN_0366_DIST::SEN_0366_DIST(UART_HandleTypeDef *huart, uint8_t ADDR, MovingAverageFilter<double> maf)
            : bsp::UART(huart), maf(maf) {
      address = ADDR;
      SetupRx(100);
      SetupTx(100);
    }

    SEN_0366_DIST*  SEN_0366_DIST::init(UART_HandleTypeDef *huart, uint8_t ADDR) {
        sen_0366_config(huart);
        size_t window_size = 30;
        SEN_0366_DIST* ret = new SEN_0366_DIST(huart, ADDR, MovingAverageFilter<double>(window_size));
        return ret;
    }
    bool SEN_0366_DIST::begin() {
        // packet for single_measurement
        single_measurement_packet.command[0] = address;
        single_measurement_packet.command[1] = 0x06;
        single_measurement_packet.command[2] = 0x02;
        single_measurement_packet.command[3] = ~(address +
                single_measurement_packet.command[1] + single_measurement_packet.command[2]) + 1;
        // packet for continuous_measurement
        continuous_measurement_packet.command[0] = address;
        continuous_measurement_packet.command[1] = 0x06;
        continuous_measurement_packet.command[2] = 0x03;
        continuous_measurement_packet.command[3] = ~(address +
                continuous_measurement_packet.command[1] + continuous_measurement_packet.command[2]) + 1;
        // shutdown packet
        shutdown_packet.command[0] = address;
        shutdown_packet.command[1] = 0x04;
        shutdown_packet.command[2] = 0x02;
        shutdown_packet.command[3] = ~(address +
                shutdown_packet.command[1] + shutdown_packet.command[2]) + 1;
        // laser on packet
        laser_on_packet.command[0] = address;
        laser_on_packet.command[1] = 0x06;
        laser_on_packet.command[2] = 0x05;
        laser_on_packet.command[3] = 0x01;
        laser_on_packet.command[4] = ~(address +
                laser_on_packet.command[1] + laser_on_packet.command[2] + laser_on_packet.command[3]) + 1;
        // set measure distance
        set_measure_distance_packet.command[0] = 0xFA;
        set_measure_distance_packet.command[1] = 0x04;
        set_measure_distance_packet.command[2] = 0x09;
        // set frequency packet
        set_frequency_packet.command[0] = 0xFA;
        set_frequency_packet.command[1] = 0x04;
        set_frequency_packet.command[2] = 0x0A;
        // set resolution packet
        set_resolution_packet.command[0] = 0xFA;
        set_resolution_packet.command[1] = 0x04;
        set_resolution_packet.command[2] = 0x0C;

        Write((uint8_t*)&continuous_measurement_packet, sizeof(SendPacket_4_t ));



        return laserOn()&true;
    }

    bool SEN_0366_DIST::laserOn() {
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&laser_on_packet, sizeof(SendPacket_5_t));
        Read(&buffer);
        // TODO: ask customer services of how to do CRC check
        return true;
    }

    bool SEN_0366_DIST::shutdown(){
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&shutdown_packet, sizeof(SendPacket_4_t));
        Read(&buffer);
        return true;
    }

    void SEN_0366_DIST::readValue() {
      uint8_t *buffer;
      Read(&buffer);
      double prev_distance = distance;
      distance = (buffer[3] - 0x30) * 100 + (buffer[4] - 0x30) * 10 + (buffer[5] - 0x30) + (buffer[7] - 0x30) * 0.1 + (buffer[8] - 0x30) * 0.01;
      if (distance < 0 || distance > 70){
        distance = prev_distance;
        distance = maf.update(distance);
      }
    }
}

