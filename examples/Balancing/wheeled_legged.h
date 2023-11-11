#pragma once

// temp variable
float left_wheel_omega = 0.0;
float right_wheel_omega = 0.0;
float left_wheel_speed = 0.0;
float right_wheel_speed = 0.0;

// unit convert parameters
float rpm_rads = 2.0 * PI / 60.0;
float degree_rad = PI / 180.0;

// robot parameters
float wheel_radius = 0.03;

// K matrix
float K1 = 0.0183;
float K2 = -2.5897;
float K3 = -9.2569;
float K4 = -1.1094;
float K15 = 2.5820;
float K16 = 0.1954;
float K25 = -K15;
float K26 = -K16;

// state vector in wheel motion
float robot_position = 0.0;
float robot_speed = 0.0;
float pitch = 0.0;
float pitch_omega = 0.0;
float yaw = 0.0;
float yaw_omega = 0.0;
float roll = 0.0;
float roll_omega = 0.0;

// input vector in wheel motion
float torque_left_wheel = 0.0;
float torque_right_wheel = 0.0;

// state vector in leg motion
