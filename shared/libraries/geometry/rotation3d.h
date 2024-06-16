#include "vector3d.h"
class Rotation3d {
// use quaternion to represent rotation
private:
    float _x;
    float _y;
    float _z;
    float _w;

public:
    Rotation3d();
    Rotation3d(float x, float y, float z, float w);
    Rotation3d(const Rotation3d& r);
    ~Rotation3d();

    Rotation3d operator*(const Rotation3d& r) const;
    Rotation3d& operator*=(const Rotation3d& r);
    bool operator==(const Rotation3d& r) const;
    bool operator!=(const Rotation3d& r) const;

    float getX() const;
    float getY() const;
    float getZ() const;
    float getW() const;

    Rotation3d normalized() const;

    void toEuler(float& roll, float& pitch, float& yaw) const;
    void fromEuler(float roll, float pitch, float yaw);

    void toAxisAngle(Vector3d& axis, float& angle) const;
    void fromAxisAngle(const Vector3d& axis, float angle);

    void toMatrix(float matrix[3][3]) const;
    void fromMatrix(const float matrix[3][3]); 
};