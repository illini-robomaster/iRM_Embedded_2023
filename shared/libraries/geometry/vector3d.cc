#include "vector3d.h"
#include <math.h>

// implement Vector3d class


Vector3d::~Vector3d()
{
}

Vector3d::Vector3d()
{
     x = 0;
     y = 0;
     z = 0;
}

Vector3d::Vector3d( float x,  float y,  float z)
{
     x = x;
     y = y;
     z = z;
}

Vector3d::Vector3d(const Vector3d& v)
{
    x = v.x;
    y = v.y;
    z = v.z;
}

Vector3d Vector3d::operator+(const Vector3d& v) const
{
    return Vector3d(x + v.x, y + v.y, z + v.z);
}

Vector3d Vector3d::operator-(const Vector3d& v) const
{
    return Vector3d(x - v.x, y - v.y, z - v.z);
}

Vector3d Vector3d::operator*( float scalar) const
{
    return Vector3d(x * scalar, y * scalar, z * scalar);
}

Vector3d Vector3d::operator/( float scalar) const
{
    return Vector3d(x / scalar, y / scalar, z / scalar);
}

Vector3d& Vector3d::operator=(const Vector3d& v)
{
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
}

Vector3d& Vector3d::operator+=(const Vector3d& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vector3d& Vector3d::operator*=( float scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3d& Vector3d::operator/=( float scalar)
{
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

bool Vector3d::operator==(const Vector3d& v) const
{
    return x == v.x && y == v.y && z == v.z;
}

bool Vector3d::operator!=(const Vector3d& v) const
{
    return !(*this == v);
}

float Vector3d::dot(const Vector3d& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

Vector3d Vector3d::cross(const Vector3d& v) const
{
    return Vector3d(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}

float Vector3d::length() const
{
    return sqrt(x * x + y * y + z * z);
}

float Vector3d::lengthSquared() const
{
    return x * x + y * y + z * z;
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
        x = 0;
        y = 0;
        z = 0;
    }
    else
    {
        x /= len;
        y /= len;
        z /= len;
    }
}

