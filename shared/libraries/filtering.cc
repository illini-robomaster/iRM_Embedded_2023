#include "filtering.h"

FilterBase::~FilterBase() {
    if (x_obs_arr) {
        delete[] x_obs_arr;
    }
    if (interval_arr) {
        delete[] interval_arr;
    }
}

MovingAverageFilter::MovingAverageFilter(int window_size) : FilterBase(), _window_size(window_size), _sum(0.0f) {
    x_obs_arr = new float[_window_size];
    interval_arr = new float[_window_size];
}

MovingAverageFilter::~MovingAverageFilter() {
}

void MovingAverageFilter::register_state(float input) {
    if (_initialized) {
        float removed_obs = x_obs_arr[0];
        float removed_interval = interval_arr[0];
        // Update the observation array
        for (int i = 0; i < _window_size - 1; i++) {
            x_obs_arr[i] = x_obs_arr[i + 1];
            interval_arr[i] = interval_arr[i + 1];
        }

        // write value
        x_obs_arr[_window_size - 1] = input;
        // TODO: interval is set to constant 1 for now
        interval_arr[_window_size - 1] = 1.0f;

        // Update the sum of the inputs in the window
        _sum = _sum - removed_obs + input;
    } else {
        // fill the array with the current value to warm-start
        for (int i = 0; i < _window_size; i++) {
            x_obs_arr[i] = input;
            interval_arr[i] = 1.0f;
        }
        _initialized = true;
        _sum = input * _window_size;
    }
}

float MovingAverageFilter::get_estimation() {
    return _sum / _window_size;
}
