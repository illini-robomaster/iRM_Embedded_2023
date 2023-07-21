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

#include "bsp_print.h"

#define BSP_DEBUG print("[DEBUG] %s:%d ", __FUNCTION__, __LINE__)
// non-fatal assertions (does not hang)
#define RM_EXPECT_TRUE(cond, msg)                                \
  do {                                                           \
    if (!(cond)) bsp_error_handler(__FUNCTION__, __LINE__, msg); \
  } while (0)
#define RM_EXPECT_FALSE(cond, msg) RM_EXPECT_TRUE(!(cond), msg)
#define RM_EXPECT_EQ(expr, ref, msg) RM_EXPECT_TRUE((expr) == (ref), msg)
#define RM_EXPECT_NE(expr, ref, msg) RM_EXPECT_TRUE((expr) != (ref), msg)
#define RM_EXPECT_GT(expr, ref, msg) RM_EXPECT_TRUE((expr) > (ref), msg)
#define RM_EXPECT_GE(expr, ref, msg) RM_EXPECT_TRUE((expr) >= (ref), msg)
#define RM_EXPECT_LT(expr, ref, msg) RM_EXPECT_TRUE((expr) < (ref), msg)
#define RM_EXPECT_LE(expr, ref, msg) RM_EXPECT_TRUE((expr) <= (ref), msg)
#define RM_EXPECT_HAL_OK(expr, msg) RM_EXPECT_TRUE((expr) == HAL_OK, msg)
// fatal assertions (hangs program)
#define RM_ASSERT_TRUE(cond, msg)                     \
  do {                                                \
    if (!(cond)) {                                    \
      bsp_error_handler(__FUNCTION__, __LINE__, msg); \
      while (1)                                       \
        ;                                             \
    }                                                 \
  } while (0)
#define RM_ASSERT_FALSE(cond, msg) RM_ASSERT_TRUE(!(cond), msg)
#define RM_ASSERT_EQ(expr, ref, msg) RM_ASSERT_TRUE((expr) == (ref), msg)
#define RM_ASSERT_NE(expr, ref, msg) RM_ASSERT_TRUE((expr) != (ref), msg)
#define RM_ASSERT_GT(expr, ref, msg) RM_ASSERT_TRUE((expr) > (ref), msg)
#define RM_ASSERT_GE(expr, ref, msg) RM_ASSERT_TRUE((expr) >= (ref), msg)
#define RM_ASSERT_LT(expr, ref, msg) RM_ASSERT_TRUE((expr) < (ref), msg)
#define RM_ASSERT_LE(expr, ref, msg) RM_ASSERT_TRUE((expr) <= (ref), msg)
#define RM_ASSERT_HAL_OK(expr, msg) RM_ASSERT_TRUE((expr) == HAL_OK, msg)

/**
 * Handle error condition printf etc.
 *
 * @param  file       Which file the error occured
 * @param  line       Which line the error occured
 * @param  msg        Message want to print
 * @author Nickel_Liang
 * @date   2018-04-15
 */
void bsp_error_handler(const char* func, int line, const char* msg);
