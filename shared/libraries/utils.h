/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

/**
 * @brief clip a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to the clipped
 * @param min   range min
 * @param max   range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T clip(T value, T min, T max) {
  return value < min ? min : (value > max ? max : value);
}

/**
 * @brief absolute value
 *
 * @tparam T type of the value
 * @param val value to be taken absolute value of
 *
 * @return absolute value of the value
 */
// template <typename T>
// T abs(T val) {
//   return val < 0 ? -val : val;
// }

/**
 * @brief wrap around a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped around value that falls in the range [min, max]
 *
 * @note undefined behavior if value is more than one cycle away from min or max
 */
template <typename T>
T wrap(T value, T min, T max) {
  const T range = max - min;
  return value < min ? value + range : (value > max ? value - range : value);
}

/**
 * @brief clip a value to fall into a given range; can wrap around domain
 *                        (designed for angle-based control 2*pi!)
 *
 * @tparam T        type of the value
 * @param value     value to the clipped
 * @param min       clipping range min
 * @param max       clipping range max
 * @param range_min domain range min
 * @param range_max domain range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T wrapping_clip(T value, T min, T max, T range_min, T range_max) {
  value = wrap<T>(value, range_min, range_max);
  // if mim and max are too close, directly return value for omni
  if (abs(max - min) < 1e-4) {
    return value;
  }
  if (max >= min) {
    return value < min ? min : (value > max ? max : value);
  } else {
    // min > max; wrap around
    T middle = (min + max) / 2;
    if (value <= max && value >= range_min) {
      return value;
    } else if (value >= min && value <= range_max) {
      return value;
    } else if (value > max && value < middle) {
      return max;
    } else {
      return min;
    }
  }
}

/**
 * @brief max of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the max of the values
 */
template <typename T>
T max(T value1, T value2) {
  return value1 < value2 ? value2 : value1;
}

/**
 * @brief min of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the min of the values
 */
template <typename T>
T min(T value1, T value2) {
  return value1 < value2 ? value1 : value2;
}

/**
 * @brief get sign of a value
 *
 * @tparam T    type of the value
 * @param value value to be tested
 * @param zero  zero point of that value
 *
 * @return -1 for less than, 1 for greater than, and 0 for equal to
 */
template <typename T>
int sign(T value, T zero) {
  return value < zero ? -1 : (value > zero ? 1 : 0);
}

class BoolEdgeDetector {
 public:
  BoolEdgeDetector(bool initial);
  void input(bool signal);
  bool edge();
  bool posEdge();
  bool negEdge();

 private:
  bool prev_;
  bool posEdge_;
  bool negEdge_;
};

class FloatEdgeDetector {
 public:
  FloatEdgeDetector(float initial, float threshold);
  void input(float signal);
  bool edge();
  bool posEdge();
  bool negEdge();

 private:
  float prev_;
  float threshold_;
  bool posEdge_;
  bool negEdge_;
};
