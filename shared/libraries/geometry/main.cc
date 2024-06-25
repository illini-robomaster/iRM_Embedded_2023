#include "vector3d.h"
#include "rotation3d.h"
#include <math.h>
#include <iostream>

#define PI 3.14159265358979323846

typedef struct {
  float base_translate_0;     /* translate 3508 motor                           */
  float base_yaw_rotate_1;   /* rotate the entire arm around an vertical axis  */
  float base_pitch_rotate_2;    /* rotate the entire arm around a horizontal axis */
  float forearm_pitch_3;       /* rotate the forearm on a horizontal axis        */
  float forearm_roll_4;     /* rotate the forearm on its axis                 */
  float wrist_5;       /* rotate the hand around an vertical axis        */
  float end_6;        /* rotate the hand on its axis                    */
} joint_state_t;

const float a = 0.05;
const float b = 0.36; // big arm length
const float c = 0.30; // forearm length
joint_state_t inverse_kinematics(Vector3d position, Rotation3d orientation){
  float l = sqrt(position._x*position._x + position._y*position._y - a*a);
  std::cout << l << std::endl;
  float alpha1 = atan(l/a);
  float theta1 = atan2(position._y, position._x) + alpha1 - PI/2;

  float d = sqrt(l*l+position._z*position._z);
  float alpha2 = atan2(position._z, l);
  float beta1 = acos((b*b+d*d-c*c)/(2*b*d));
  float beta2 = acos((b*b+c*c-d*d)/(2*b*c));
  float theta2 = PI/2 - alpha2 - beta1;
  float theta3 = PI - alpha2 - beta1 - beta2;
  

  // todo last 3 joint inverse kinematics
  return {0, theta1, theta2, theta3, orientation.getYaw(), orientation.getPitch(), orientation.getRoll()};
}

int main(){
    Vector3d position(0.4, 0.0, 0.36);
    Rotation3d orientation(0, 0, 0);

    std::cout << "position: " << position._x << " " << position._y << " " << position._z << std::endl;
    joint_state_t joint = inverse_kinematics(position, orientation);
    std::cout << "theta1: " << joint.base_yaw_rotate_1 << std::endl;
    std::cout << "theta2: " << joint.base_pitch_rotate_2 << std::endl;
    std::cout << "theta3: " << joint.forearm_pitch_3 << std::endl;
    std::cout << "yaw: " << joint.wrist_5 << std::endl;
    std::cout << "pitch: " << joint.end_6 << std::endl;
    std::cout << "roll: " << joint.forearm_roll_4 << std::endl;
}