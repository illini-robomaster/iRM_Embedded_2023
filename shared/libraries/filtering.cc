#include "filtering.h"

#include "controller.h"
#include "utils.h"

/* Designed for ANGLE!!! */

KalmanFilter::KalmanFilter(float init_x, float init_t) : FilterBase() {
  xhat = init_x;
  xhatminus = init_x;
  last_x = init_x;
  last_t = init_t;
}

void KalmanFilter::register_state(float input, float timestamp) {
  last_x = input;
  last_t = timestamp;

  // Wrap xhat and xhatminus
  if (abs(xhat + 2 * PI - input) < abs(xhat - input)) {
    xhat += 2 * PI;
    xhatminus += 2 * PI;
  } else if (abs(xhat - 2 * PI - input) < abs(xhat - input)) {
    xhat -= 2 * PI;
    xhatminus -= 2 * PI;
  }
}

float KalmanFilter::iter_and_get_estimation() {
  xhatminus = A * xhat + B;
  Pminus = A * P * A + Q;

  float K = Pminus * H / (H * Pminus * H + R);

  // TODO: use better extrapolation (e.g., exponential decay moving average)
  float extrapolation = last_x;
  xhat = xhatminus + K * (extrapolation - H * xhatminus);

  return xhat;
}

float KalmanFilter::get_estimation() {
  return xhat;
}
