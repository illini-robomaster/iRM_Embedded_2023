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

#include "utils.h"

BoolEdgeDetector::BoolEdgeDetector(bool initial) { prev_ = initial; }

void BoolEdgeDetector::input(bool signal) {
  posEdge_ = false;
  negEdge_ = false;
  if (!prev_ && signal)
    posEdge_ = true;
  else if (prev_ && !signal)
    negEdge_ = true;
  prev_ = signal;
}

bool BoolEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool BoolEdgeDetector::posEdge() { return posEdge_; }

bool BoolEdgeDetector::negEdge() { return negEdge_; }

FloatEdgeDetector::FloatEdgeDetector(float initial, float threshold) {
  prev_ = initial;
  threshold_ = threshold;
}

void FloatEdgeDetector::input(float signal) {
  posEdge_ = false;
  negEdge_ = false;
  float diff = signal - prev_;
  if (diff > threshold_)
    posEdge_ = true;
  else if (diff < -threshold_)
    negEdge_ = true;
  prev_ = signal;
}

bool FloatEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool FloatEdgeDetector::posEdge() { return posEdge_; }

bool FloatEdgeDetector::negEdge() { return negEdge_; }

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t) ((x-offset) * ((float)((1<<bits)-1))/span);
}

// WARNING: this does not produce the correct result when bits = 16
// e.g. x_min = -PI and x_max = PI, the result is from -2PI to 0 instead if bits = 16
// probably has to do with int is 16bit and when the bits param is 16

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  int32_t x_int_longer = 0x0000FFFF & x_int; // zero extend 16 bit to 32 bit
  return ((float)x_int_longer) * span / ((float)((1 << bits) - 1)) + offset;
}
