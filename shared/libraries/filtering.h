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
    virtual void register_state(float input) = 0;

protected:
    // The observation array
    float* x_obs_arr;

    // The time interval between observation i - 1 and i
    float* interval_arr;

    bool _initialized=false;
};

class MovingAverageFilter : public FilterBase {
public:
    MovingAverageFilter(int window_size);
    void register_state(float input);
    float get_estimation();

private:
    int _window_size;
    float _sum;
};
