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

class FilterBase {
 public:
  virtual void register_state(float input, float timestamp) = 0;

  virtual float get_estimation() = 0;
};

class KalmanFilter : public FilterBase {
 public:
  KalmanFilter(float init_x, float init_t);
  void register_state(float input, float timestamp);
  float get_estimation();
  float iter_and_get_estimation();

 private:
  float xhat = 0;       // a posteriori estimate of x
  float xhatminus = 0;  // a priori estimate of x
  float P = 0;          // posteriori error estimate
  float Pminus = 0;     // a priori error estimate

  float Q = 2;  // process noise covariance
  float H = 1;  // measurement function

  float A = 1;  // state transition matrix
  float B = 0;  // control matrix

  float R = 2;  // measurement noise covariance

  float last_x = 0;
  float last_t = 0;
};
