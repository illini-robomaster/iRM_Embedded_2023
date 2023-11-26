#pragma once

// temp variable
float demo_height = 0.360795;
float rotation_angle = 0.0;
int8_t turn_count = 0;
float last_raw_yaw = 0.0;
float current_raw_yaw = 0.0;
float current_yaw = 0.0;
float jump_height = demo_height + 0.3;
bool jump_flag = false;

// unit convert parameters
float rpm_rads = 2.0 * PI / 60.0;
float magic_rpm = 469.0 * rpm_rads / 14000.0;

// robot parameters
float wheel_radius = 0.03;

// left front leg, positive -> move up
// left back leg, negative -> move up
// right front leg, negative -> move up
// right back leg, positive -> move up