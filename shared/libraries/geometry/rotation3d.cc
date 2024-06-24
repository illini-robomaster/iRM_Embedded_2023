//implement class rotation3d in rotation3d.h
#include "rotation3d.h"
#include <math.h>

#define PI 3.14159265358979323846

Rotation3d::~Rotation3d()
{
}

Rotation3d::Rotation3d()
{
     _x = 0;
     _y = 0;
     _z = 0;
     _w = 0;
}

Rotation3d::Rotation3d( float x,  float y,  float z,  float w)
{
     _x = x;
     _y = y;
     _z = z;
     _w = w;
}

Rotation3d::Rotation3d(const Rotation3d& r)
{
    _x = r._x;
    _y = r._y;
    _z = r._z;
    _w = r._w;
}

Rotation3d Rotation3d::operator*(const Rotation3d& r) const
{
    return Rotation3d(
        _w * r._x + _x * r._w + _y * r._z - _z * r._y,
        _w * r._y - _x * r._z + _y * r._w + _z * r._x,
        _w * r._z + _x * r._y - _y * r._x + _z * r._w,
        _w * r._w - _x * r._x - _y * r._y - _z * r._z
    );
}

Rotation3d& Rotation3d::operator*=(const Rotation3d& r)
{
    float x = _w * r._x + _x * r._w + _y * r._z - _z * r._y;
    float y = _w * r._y - _x * r._z + _y * r._w + _z * r._x;
    float z = _w * r._z + _x * r._y - _y * r._x + _z * r._w;
    float w = _w * r._w - _x * r._x - _y * r._y - _z * r._z;
    _x = x;
    _y = y;
    _z = z;
    _w = w;
    return *this;
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

Rotation3d Rotation3d::normalized() const
{
    float length = sqrt(_x * _x + _y * _y + _z * _z + _w * _w);
    return Rotation3d(_x / length, _y / length, _z / length, _w / length);
}

void Rotation3d::toEuler()
{
    float sinr_cosp = 2 * (_w * _x + _y * _z);
    float cosr_cosp = 1 - 2 * (_x * _x + _y * _y);
    _roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (_w * _y - _z * _x);
    if (fabs(sinp) >= 1)
    {
        _pitch = copysign(PI / 2, sinp);
    }
    else
    {
        _pitch = asin(sinp);
    }

    float siny_cosp = 2 * (_w * _z + _x * _y);
    float cosy_cosp = 1 - 2 * (_y * _y + _z * _z);
    _yaw = atan2(siny_cosp, cosy_cosp);
}

void Rotation3d::fromEuler(float roll, float pitch, float yaw)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    _w = cr * cp * cy + sr * sp * sy;
    _x = sr * cp * cy - cr * sp * sy;
    _y = cr * sp * cy + sr * cp * sy;
    _z = cr * cp * sy - sr * sp * cy;
}

void Rotation3d::toAxisAngle(Vector3d& axis, float& angle) const
{
    float scale = sqrt(_x * _x + _y * _y + _z * _z);
    if (scale < 1e-9)
    {
        axis = Vector3d(1, 0, 0);
        angle = 0;
    }
    else
    {
        axis = Vector3d(_x / scale, _y / scale, _z / scale);
        angle = 2 * acos(_w);
    }
}

void Rotation3d::fromAxisAngle(const Vector3d& axis, float angle)
{
    float half_angle = angle * 0.5;
    float s = sin(half_angle);
    _x = axis.x * s;
    _y = axis.y * s;
    _z = axis.z * s;
    _w = cos(half_angle);
}

void Rotation3d::toMatrix(float matrix[3][3]) const
{
    float xx = _x * _x;
    float xy = _x * _y;
    float xz = _x * _z;
    float xw = _x * _w;
    float yy = _y * _y;
    float yz = _y * _z;
    float yw = _y * _w;
    float zz = _z * _z;
    float zw = _z * _w;

    matrix[0][0] = 1 - 2 * (yy + zz);
    matrix[0][1] = 2 * (xy - zw);
    matrix[0][2] = 2 * (xz + yw);
    matrix[1][0] = 2 * (xy + zw);
    matrix[1][1] = 1 - 2 * (xx + zz);
    matrix[1][2] = 2 * (yz - xw);
    matrix[2][0] = 2 * (xz - yw);
    matrix[2][1] = 2 * (yz + xw);
    matrix[2][2] = 1 - 2 * (xx + yy);
}

void Rotation3d::fromMatrix(const float matrix[3][3])
{
    float trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0)
    {
        float s = 0.5 / sqrt(trace + 1.0);
        _w = 0.25 / s;
        _x = (matrix[2][1] - matrix[1][2]) * s;
        _y = (matrix[0][2] - matrix[2][0]) * s;
        _z = (matrix[1][0] - matrix[0][1]) * s;
    }
    else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2])
    {
        float s = 2.0 * sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
        _w = (matrix[2][1] - matrix[1][2]) / s;
        _x = 0.25 * s;
        _y = (matrix[0][1] + matrix[1][0]) / s;
        _z = (matrix[0][2] + matrix[2][0]) / s;
    }
    else if (matrix[1][1] > matrix[2][2])
    {
        float s = 2.0 * sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
        _w = (matrix[0][2] - matrix[2][0]) / s;
        _x = (matrix[0][1] + matrix[1][0]) / s;
        _y = 0.25 * s;
        _z = (matrix[1][2] + matrix[2][1]) / s;
    }
    else
    {
        float s = 2.0 * sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
        _w = (matrix[1][0] - matrix[0][1]) / s;
        _x = (matrix[0][2] + matrix[2][0]) / s;
        _y = (matrix[1][2] + matrix[2][1]) / s;
    }
}
