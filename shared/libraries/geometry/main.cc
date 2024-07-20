#include "geometry.h"
#include <iostream>
#include <math.h>
int main(){
    // test rotate vector3d by rotation3d
    Vector3d v(1, 0, 0);
    Rotation3d orientation(1, -M_PI/6, -M_PI/8); // roll pitch yaw
    Vector3d v_rotated = v.rotateBy(orientation);

    Rotation3d rotationBetween = v.getRotation3d(v_rotated);
    Vector3d v_rotated_2 = v.rotateBy(rotationBetween);
    // v original
    std::cout << "v: " << v._x << " " << v._y << " " << v._z << std::endl;
    std::cout << "v_rotated: " << v_rotated._x << " " << v_rotated._y << " " << v_rotated._z << std::endl;

    Rotation3d difference_rotation = orientation.minus(Rotation3d(0,0,0));
    std::cout << "minus result" << difference_rotation.getRoll() << " " << difference_rotation.getPitch() << " " << difference_rotation.getYaw() << std::endl;
    std::cout << difference_rotation.getAxisAngle().angle.getRadians() << std::endl;
    // std::cout << "v_rotated_2: " << v_rotated_2._x << " " << v_rotated_2._y << " " << v_rotated_2._z << std::endl;

    // std::cout << "angle between two vectors" << v.angleBetween(v_rotated).getDegrees() << std::endl;
    // std::cout << "rotation3d between two vectors" << rotationBetween.getRoll() << " " << rotationBetween.getPitch() << " " << rotationBetween.getYaw() << std::endl;
    // std::cout << "angle between two rotations" << orientation.angleBetween(rotationBetween).getDegrees() << std::endl;


    Vector3d wrist_facing = Vector3d(1,0,0).rotateBy(orientation);
    // assumes forearm is pointing forward (camera is on forearm)
    // TODO: test and test sign
    float theta4 = atan2(wrist_facing._z, wrist_facing._y); // the angle of the wrist pointing direction projected in camera plane
    float theta5 = wrist_facing.angleBetween(Vector3d(1,0,0)).getRadians();  // angle between pointing forward and desired orientation as a vector
   
    // optimize 4310 angles
    // J4 angle should not turn over 90 degrees, and should reverse theta5
    // Angle2d j4_delta(theta4-joint_angles.forearm_roll_4);

    float magical_multipler = 1.0;
    // if(abs(j4_delta.getRadiansNegPItoPI())>M_PI/2){
        theta4 = Angle2d(theta4-M_PI).getRadians();
        theta5 *= -1.0;
        magical_multipler = -1.0;
    // }

    Rotation3d j4_rotation(1,0,0, Angle2d(theta4));
    Rotation3d j5_rotation(0,0,1, Angle2d(theta5));

    Rotation3d j4j5_rotation = j4_rotation * j5_rotation;
    
    // Rotation3d wrist_facing_rot = Vector3d(1,0,0).getRotation3d(wrist_facing);
    float theta6 = j4j5_rotation.minus(orientation).getAxisAngle().angle.getRadians(); // angle between desired orientation and orientation achieved by J4 and J5 only, so the difference is j6 angle
    // however, there two direction of j6, so we need to do forward kinematics to see if it's this theta6 or its opposite.
    
    Rotation3d j6_rotation(1,0,0, Angle2d(theta6));

    Rotation3d j4j5j6_rotation = j4_rotation*j5_rotation*j6_rotation; // rotation by joint 6 then joint 5 then joint 4

    std::cout << "j4j5j6 rotation" << j4j5j6_rotation.getRoll() << " " << j4j5j6_rotation.getPitch() << " " << j4j5j6_rotation.getYaw() << std::endl;
    // compare j4j5j6_rotation to orientation, if difference is orientation is big, then reverse j6 angle;
    float diff = j4j5j6_rotation.minus(orientation).getAxisAngle().angle.getRadians();
    std::cout << "diff" << diff << std::endl;

    if(abs(diff) > 0.01){
        theta6 -= diff * magical_multipler;
    }

    j6_rotation = Rotation3d(1,0,0, Angle2d(theta6));
    j4j5j6_rotation = j4_rotation*j5_rotation*j6_rotation; // rotation by joint 6 then joint 5 then joint 4
    Vector3d j4j5j6_facing = Vector3d(1,0,0).rotateBy(j4j5j6_rotation);
    std::cout << "j4j5j6 facing" << j4j5j6_facing._x << " " << j4j5j6_facing._y << " " << j4j5j6_facing._z << std::endl;
    std::cout << "j4j5j6 rotation corrected: " << j4j5j6_rotation.getRoll() << " " << j4j5j6_rotation.getPitch() << " " << j4j5j6_rotation.getYaw() << std::endl;
    diff = j4j5j6_rotation.minus(orientation).getAxisAngle().angle.getRadians();
    std::cout << "diff corrected" << diff << std::endl;
    std::cout << theta4 << " " << theta5 << " " << theta6 << std::endl;
    //   return {0,theta1, theta2, theta3, theta4, theta5, theta6};
}