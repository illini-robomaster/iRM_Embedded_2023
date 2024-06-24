#include "vector3d.h"
class Rotation3d {
// use quaternion to represent rotation
private:

    // quaternion representation
    float _x; // i
    float _y; // j 
    float _z; // k
    float _w; // real part
    
public:
    Rotation3d();
    Rotation3d(float x, float y, float z, float w); // quaternion
    Rotation3d(float yaw, float pitch, float roll); // euler angles (yaw, pitch, roll
    // TODO: axis-angle and rotation matrix

    Rotation3d operator*(const Rotation3d& r) const;
    bool operator==(const Rotation3d& r) const;
    bool operator!=(const Rotation3d& r) const;

    float getX() const;
    float getY() const;
    float getZ() const;
    float getW() const;

    float getYaw() const;
    float getPitch() const;
    float getRoll() const;

    Rotation3d normalized() const;
};