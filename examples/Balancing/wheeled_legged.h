#pragma once

// range parameters
float max_torque_wheel = 0.5;
float max_torque_leg = 7.0;

// temp variable
// wheel
// negative velocity for left wheel move forward
// positive velocity for right wheel move forward
float left_wheel_omega = 0.0;
float right_wheel_omega = 0.0;
float left_wheel_speed = 0.0;
float right_wheel_speed = 0.0;
float current_left_wheel = 0.0;
float current_right_wheel = 0.0;

// leg
float left_front_leg_angle = 0.0;
float left_back_leg_angle = 0.0;
float right_front_leg_angle = 0.0;
float right_back_leg_angle = 0.0;
float left_height = 0.0;
float right_height = 0.0;

float move_set= 0.0;
float rotate_set = 0.0;
float elevation_speed = 0.0;

float demo_height = 0.360795;
float rotation_angle = 0.0;
int8_t turn_count = 0;
float last_raw_yaw = 0.0;
float current_raw_yaw = 0.0;
float current_yaw = 0.0;

// unit convert parameters
float rpm_rads = 2.0 * PI / 60.0;
float degree_rad = PI / 180.0;
float torque_constant = 0.34; // Nm/A
float current_mapping_constant_3508 = 16384.0 / 20.0;
float left_leg_force_to_torque = 0.0;
float right_leg_force_to_torque = 0.0;
float magic_rpm = 469.0 * rpm_rads / 14000.0;

// robot parameters
float wheel_radius = 0.03;
float leg_angle_offset = 165/180.0*PI;
float gravity_constant = 9.70;
float l_1 = 0.05;
float l_2 = 0.1;
float l_3 = 0.18;
float distance = 0.3504;
float mass = 3.848;

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
float robot_height = 0.0;
float robot_acceleration_z = 0.0;
float robot_velocity_z = 0.0;
float left_leg_force = 0.0;
float right_leg_force = 0.0;
float left_front_leg_torque = 0.0;
float left_back_leg_torque = 0.0;
float right_front_leg_torque = 0.0;
float right_back_leg_torque = 0.0;

// spring damping constant
float k_1 = 50.0;
float k_2 = 20.0;
float c_1 = 10.0;
float c_2 = 5.0;


// left front leg, positive -> move up
// left back leg, negative -> move up
// right front leg, negative -> move up
// right back leg, positive -> move up