/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include <cmath>
#include <string>
#include "geometry.h"

Angle2d::~Angle2d()
{
}

Angle2d::Angle2d( float radians)
{
     _radians = radians;
     _cos_value = cos(radians);
     _sin_value = sin(radians);

}

Angle2d::Angle2d()
{
     _radians = 0;
     _cos_value = 1;
     _sin_value = 0;
}

Angle2d Angle2d::createFromDegrees( float degrees)
{
    return Angle2d(degrees/180.0*PI);
}

Angle2d Angle2d::createFromRotations( float rotations)
{
    return Angle2d(rotations*2*PI);
}

Angle2d::Angle2d( float x,  float y)
{
     float magnitude = sqrt(x*x+y*y);

    //to prevent chaos near 0
    if(magnitude > 1e-6){
         _sin_value = y / magnitude;
         _cos_value = x / magnitude;
    }else{
         _sin_value = 0;
         _cos_value = 1;
    }
     _radians = atan2(_sin_value, _cos_value);
}

Angle2d Angle2d::plus(Angle2d other)
{
    return Angle2d( _radians + other.getRadians());
}

Angle2d Angle2d::unaryMinus()
{
    return Angle2d( _radians*-1);
}

Angle2d Angle2d::minus(Angle2d other)
{
    return  plus(other.unaryMinus());
}

Angle2d Angle2d::oppositeAngle()
{
    return Angle2d( _radians+PI); //current radian + PI
}

Angle2d Angle2d::times( float scalar)
{
    return Angle2d(scalar *  _radians);
}

Angle2d Angle2d::getAngleIn1Rotation()
{
    return Angle2d(getRadians0to2PI());
}

Angle2d Angle2d::getAngleIn1RotationSymmetricAbout0()
{
    return Angle2d(getRadiansNegPItoPI());
}

 float Angle2d::getRadians0to2PI()
{
     float temp_angle = atan2(_sin_value, _cos_value); // [-PI, PI)
    return temp_angle < 0?temp_angle+2*PI:temp_angle;
}

 float Angle2d::getRadiansNegPItoPI()
{
    return atan2(_sin_value, _cos_value);
}

 float Angle2d::getDegrees0to360()
{
    return  getRadians0to2PI()/PI*180;
}

 float Angle2d::getDegreesN180to180()
{
    return getRadiansNegPItoPI()/PI*180;
}

 float Angle2d::getRadians()
{
    return  _radians;
}

 float Angle2d::getDegrees()
{
    return  _radians / PI * 180;
}

 float Angle2d::getRotations()
{
    return  _radians / 2 / PI;
}

 float Angle2d::getCos()
{
    return  _cos_value;
}

 float Angle2d::getSin()
{
    return  _sin_value;
}

std::string Angle2d::toString()
{
    return std::string("Angle2d with " + std::to_string(_radians) + " radians");
}

bool Angle2d::equalsTo(Angle2d other)
{
    return hypot(other._cos_value -  _cos_value, other._sin_value -  _sin_value) < 1e-9;
}

// ==========================================================================================
// Vector 2d
// =========================================================================================

Vector2d::~Vector2d()
{
}

Vector2d::Vector2d()
{
     _x = 0;
     _y = 0;
}

Vector2d::Vector2d( float x,  float y)
{
     _x = x;
     _y = y;
}

Vector2d::Vector2d( float r, Angle2d theta)
{
     _x = r * theta.getCos();
     _y = r * theta.getSin();
}

 float Vector2d::getX()
{
    return  _x;
}

 float Vector2d::getY()
{
    return  _y;
}

 float Vector2d::getMagnitude()
{
    return hypot(_x,  _y);
}

Angle2d Vector2d::getDirection()
{
    return Angle2d( _x,  _y);
}

Vector2d Vector2d::rotateBy(Angle2d other)
{
    return Vector2d(
        _x * other.getCos() - _y * other.getSin(),
        _x * other.getSin() + _y * other.getCos()
    );
}

Vector2d Vector2d::plus(Vector2d other)
{
    return Vector2d(_x + other.getX(), _y + other.getY());
}

Vector2d Vector2d::minus(Vector2d other)
{
    return plus(other.inverse());
}

Vector2d Vector2d::inverse()
{
    return Vector2d( _x * -1,  _y * -1);
}

 float Vector2d::dot(Vector2d other)
{
    return  _x * other.getX() +  _y * other.getY();
}

Vector2d Vector2d::times( float scalar)
{
    return Vector2d( _x * scalar,  _y * scalar);
}

std::string Vector2d::toString()
{
    return std::string("<" + std::to_string( _x) + "," + std::to_string( _y) + ">");
}

Vector2d Vector2d::normalize()
{
    if(getMagnitude() < 1e-9){
        return Vector2d(0, 0);
    }
    return times(1.0/getMagnitude());
}

bool Vector2d::equalsTo(Vector2d other)
{
    return  minus(other).getMagnitude() < 1e-9;
}