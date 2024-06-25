#pragma once


class Vector3d{
public:
    Vector3d();
    Vector3d(float x, float y, float z);
    Vector3d(const Vector3d& v);
    ~Vector3d();

    float _x;
    float _y;
    float _z;

    Vector3d operator+(const Vector3d& v) const;
    Vector3d operator-(const Vector3d& v) const;
    Vector3d operator*(float scalar) const;
    Vector3d operator/(float scalar) const;
    Vector3d& operator=(const Vector3d& v);
    Vector3d& operator+=(const Vector3d& v);
    Vector3d& operator-=(const Vector3d& v);
    Vector3d& operator*=(float scalar);
    Vector3d& operator/=(float scalar);
    bool operator==(const Vector3d& v) const;
    bool operator!=(const Vector3d& v) const;

    float dot(const Vector3d& v) const;
    Vector3d cross(const Vector3d& v) const;
    float length() const;
    float lengthSquared() const;
    Vector3d normalized() const;
    void normalize();
};