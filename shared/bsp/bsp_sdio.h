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

#pragma once

#include <string>

#ifdef BOARD_HAS_SD_FATFS
#include "fatfs.h"

namespace bsp {

class SDFileLogger {
 public:
  /**
   * @brief constructor of a sd card file logger
   *
   * @param filename filename of the log file to create
   */
  explicit SDFileLogger(const std::string& filename);

  /**
   * @brief log certain amount of data on the the sd card
   *
   * @param data    pointer to the byte array
   * @param length  length of data
   *
   * @return number of bytes actually written, -1 if error
   */
  int32_t Log(const uint8_t* data, uint32_t length);

 private:
  static bool mounted_;

  const std::string filename_;
  FIL fobj_;
};

} /* namespace bsp */

#endif  // BOARD_HAS_SD_FATFS
