#include "vector3d.h"
#include <math.h>

// implement Vector3d class


Vector3d::~Vector3d()
{
}

Vector3d::Vector3d()
{
     _x = 0;
     _y = 0;
     _z = 0;
}

Vector3d::Vector3d( float x,  float y,  float z)
{
     _x = x;
     _y = y;
     _z = z;
}

Vector3d::Vector3d(const Vector3d& v)
{
    _x = v._x;
    _y = v._y;
    _z = v._z;
}

Vector3d Vector3d::operator+(const Vector3d& v) const
{
    return Vector3d(_x + v._x, _y + v._y, _z + v._z);
}

Vector3d Vector3d::operator-(const Vector3d& v) const
{
    return Vector3d(_x - v._x, _y - v._y, _z - v._z);
}

Vector3d Vector3d::operator*( float scalar) const
{
    return Vector3d(_x * scalar, _y * scalar, _z * scalar);
}

Vector3d Vector3d::operator/( float scalar) const
{
    return Vector3d(_x / scalar, _y / scalar, _z / scalar);
}

Vector3d& Vector3d::operator=(const Vector3d& v)
{
    _x = v._x;
    _y = v._y;
    _z = v._z;
    return *this;
}

Vector3d& Vector3d::operator+=(const Vector3d& v)
{
    _x += v._x;
    _y += v._y;
    _z += v._z;
    return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d& v)
{
    _x -= v._x;
    _y -= v._y;
    _z -= v._z;
    return *this;
}

Vector3d& Vector3d::operator*=( float scalar)
{
    _x *= scalar;
    _y *= scalar;
    _z *= scalar;
    return *this;
}

Vector3d& Vector3d::operator/=( float scalar)
{
    _x /= scalar;
    _y /= scalar;
    _z /= scalar;
    return *this;
}

bool Vector3d::operator==(const Vector3d& v) const
{
    return _x == v._x && _y == v._y && _z == v._z;
}

bool Vector3d::operator!=(const Vector3d& v) const
{
    return !(*this == v);
}

float Vector3d::dot(const Vector3d& v) const
{
    return _x * v._x + _y * v._y + _z * v._z;
}

Vector3d Vector3d::cross(const Vector3d& v) const
{
    return Vector3d(
        _y * v._z - _z * v._y,
        _z * v._x - _x * v._z,
        _x * v._y - _y * v._x
    );
}

float Vector3d::length() const
{
    return sqrt(_x * _x + _y * _y + _z * _z);
}

float Vector3d::lengthSquared() const
{
    return _x * _x + _y * _y + _z * _z;
}

Vector3d Vector3d::normalized() const
{
    float len = length();
    if (len < 1e-9)
    {
        return Vector3d(0, 0, 0);
    }
    return *this / len;
}

void Vector3d::normalize()
{
    float len = length();
    if (len < 1e-9)
    {
        _x = 0;
        _y = 0;
        _z = 0;
    }
    else
    {
        _x /= len;
        _y /= len;
        _z /= len;
    }
}

