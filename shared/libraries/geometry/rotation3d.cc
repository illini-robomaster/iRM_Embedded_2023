//implement class rotation3d in rotation3d.h
#include "rotation3d.h"
#include <math.h>

#define PI 3.14159265358979323846

// TODO: not tested



Rotation3d::Rotation3d()
{
     _x = 0;
     _y = 0;
     _z = 0;
     _w = 0;
}

/**
 * @brief Construct a new Rotation3d object using complex representation
 * @param x i part
 * @param y j part
 * @param z k part
 * @param w w part
*/
Rotation3d::Rotation3d(Quaternion q)
{
     _x = q.x;
     _y = q.y;
     _z = q.z;
     _w = q.w;
}

Rotation3d::Rotation3d(float x, float y, float z, float w)
{
    _x = x;
    _y = y;
    _z = z;
    _w = w;
}


Rotation3d::Rotation3d(float roll, float pitch, float yaw)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    _w = cy * cp * cr + sy * sp * sr;
    _x = cy * cp * sr - sy * sp * cr;
    _y = sy * cp * sr + cy * sp * cr;
    _z = sy * cp * cr - cy * sp * sr;
}

//checked
Rotation3d Rotation3d::operator*(const Rotation3d& r) const
{   
    // v1 dot v2 (imaginary part)
    float dot = _x * r._x + _y * r._y + _z * r._z;

    // v1 cros v2
    float cross_x = _y * r._z - _z * r._y;
    float cross_y = _z * r._x - _x * r._z;
    float cross_z = _x * r._y - _y * r._x;

    float new_w = _w * r._w - dot;
    float new_x = _w * r._x + r._w * _x + cross_x;
    float new_y = _w * r._y + r._w * _y + cross_y;
    float new_z = _w * r._z + r._w * _z + cross_z;

    return Rotation3d(new_x, new_y, new_z, new_w);
}

bool Rotation3d::operator==(const Rotation3d& r) const
{
    return _x == r._x && _y == r._y && _z == r._z && _w == r._w;
}

bool Rotation3d::operator!=(const Rotation3d& r) const
{
    return _x != r._x || _y != r._y || _z != r._z || _w != r._w;
}

float Rotation3d::getX() const
{
    return _x;
}

float Rotation3d::getY() const
{
    return _y;
}

float Rotation3d::getZ() const
{
    return _z;
}

float Rotation3d::getW() const
{
    return _w;
}

float Rotation3d::getYaw() const
{
    float cycz = 1.0 - 2.0 * (_y * _y + _z * _z);
    float cysz = 2.0 * (_x * _y + _z * _w);
    float cy_sq = cycz * cycz + cysz * cysz;
    
    if (cy_sq > 1e-6){
        return atan2(cysz, cycz);
    } else {
        return atan2(2.0 * _w * _z, _w * _w - _z * _z);
    }
}

float Rotation3d::getPitch() const
{
    float ratio = 2.0 * (_w * _y - _z * _x);
    if (abs(ratio) >= 1.0){
        return copysign(PI / 2.0, ratio);
    } else {
        return asin(ratio);
    }
}

float Rotation3d::getRoll() const
{
    float cxcy = 1.0 - 2.0 * (_x * _x+ _y * _y);
    float sxcy = 2.0 * (_z * _y + _x * _w );
    float cy_sq = cxcy * cxcy + sxcy * sxcy;
    if(cy_sq > 1e-6){
        return atan2(sxcy, cxcy);
    } else {
        return 0.0;
    }
    return 0.0;
}

Rotation3d Rotation3d::normalized() const
{
    float length = sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    return Rotation3d(_x / length, _y / length, _z / length, _w / length);
}

Rotation3d Rotation3d::conjugate() const
{
    return Rotation3d(-_x, -_y, -_z, _w);
}

Angle2d Rotation3d::angleBetween(const Rotation3d& r) const
{
    float dot = _x * r._x + _y * r._y + _z * r._z + _w * r._w;
    return Angle2d(2.0 * acos(dot));
}