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

#include "bsp_sdio.h"

namespace bsp {

#ifdef BOARD_HAS_SD_FATFS

bool SDFileLogger::mounted_ = false;

SDFileLogger::SDFileLogger(const std::string& filename) : filename_(filename) {
  // lazy initial mounting
  if (!mounted_) {
    osDelay(100);  // wait for default thread to finish initializing FatFS
    f_mount(&SDFatFS, SDPath, 0);
    mounted_ = true;
  }
  // copy filename
  // clear destination file
  f_open(&fobj_, filename.c_str(), FA_CREATE_ALWAYS);
  f_close(&fobj_);
}

int32_t SDFileLogger::Log(const uint8_t* data, uint32_t length) {
  int32_t ret;
  UINT bytes_written;

  if (f_open(&fobj_, filename_.c_str(), FA_OPEN_APPEND | FA_WRITE) != FR_OK) return -1;

  if (f_write(&fobj_, data, length, &bytes_written) != FR_OK)
    ret = -1;
  else
    ret = static_cast<int32_t>(bytes_written);

  f_close(&fobj_);

  return ret;
}

#endif  // BOARD_HAS_SD_FATFS

} /* namespace bsp */
