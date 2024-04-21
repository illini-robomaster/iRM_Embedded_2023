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

#pragma once
#include <string>
#include <cmath>
#include "controller.h"


/**
 * Create an instance that contains an angle. This angle's unit can be taken to degree, radian, or rotation.
 * Its range is also all real numebrs meaning that an angle of 361 degree is not equivalent with an angle of 1 degree.
 * However, it's possible to converted to range [0, 2PI) or [-PI, PI).
 */
class Angle2d
{
private:
    float  _sin_value;
    float  _cos_value;
    float  _radians;

public:
    ~Angle2d();

    /**
     * Create an Angle2d instance from radians. If want to create an instance from degrees, then use the createFromDegrees() method
     *
     * @param radians the radian of the angle
     */
    Angle2d(float  radians);

    /**
     * Default constructor that generate an instance with radian of 0.0
     */
    Angle2d();

    /**
     * Create an Angle2d instance whose angle is the angle from the x-axis to the vector <x,y>
     *
     * @param x the x coordinate
     * @param y the y coordinate
     *
     *
     *
     */
    Angle2d(float  x, float  y);

    /**
     * Create an Angle2d instance from degrees
     *
     * @param degrees the degree of the angle
     * @return the instance created
     */
    static Angle2d createFromDegrees(float  degrees);

    /**
     * Create an Angle2d instance from degrees
     *
     * @param rotations the number of full rotation of the angle (1 rotation = 360 degrees)
     * @return the instance created
     */
    static Angle2d createFromRotations(float  rotations);

    /**
     * returns a new instance of Angle2d with this angle plus another angle
     * @param other the other angle in the addition operation
     */
    Angle2d plus(Angle2d other);

    /**
     * returns a new instance of Angle2d with 0 minus this angle
     * e.g. 60 degrees's inverse is -60 degrees
     *
     *
     */
    Angle2d unaryMinus();

    /** returns a new instance of Angle2d with the opposite angle (+180 degrees)*
     * e.g. 30 degree's inverse is 210 degree
     *
     */
    Angle2d oppositeAngle();

    /**
     * returns a new instance of Angle2d with this angle minus another angle
     */
    Angle2d minus(Angle2d other);

    /**
     * returns a new instance of Angle2d with this angle times a scalar
     */
    Angle2d times(float  scalar);

    /**
     * returns a new Angle2d instance which is this angle converting into a corresponding angle in range [0, 2PI) radians.
     */
    Angle2d getAngleIn1Rotation();

    /**
     * returns a new Angle2d instance which is this angle converting into a corresponding angle in range [-PI, PI) radians.
     */
    Angle2d getAngleIn1RotationSymmetricAbout0();

    /**
     * returns this angle in radians but in the range of [0, 2PI).
     */
    float  getRadians0to2PI();

    /**
     * return this angle in radians but in the range of [-PI, PI).
     */
    float  getRadiansNegPItoPI();

    /**
     * returns this angle in degrees in the range [0, 360).
     */
    float  getDegrees0to360();

    /**
     * returns this angle in degrees in the range [-180, 180). 
    */
    float  getDegreesN180to180();

    /**
     * returns this angle in radians
     */
    float  getRadians();

    /**
     * returns this angle in degrees
     */
    float  getDegrees();

    /**
     * returns this angle in the number of rotations
     */
    float  getRotations();

    /** returns the cosine of this angle
     *
     */
    float  getCos();

    /** returns the sine of this angle
     *
     */
    float  getSin();

    /**
     * returns a string containing info of this instance
     */
    std::string toString();

    /**
     * checks if two angles are equal (or very close)
     */
    bool equalsTo(Angle2d other);
};

class Vector2d {
    private:
        float  _x, _y;

    public:

        ~Vector2d();

        Vector2d();

        /**
         * Create an 2d vector with rectangular/Cartesian coordinates.
         * 
         * @param x the x coordinate of the vector.
         * @param y the y coordinate of the vector.
        */
        Vector2d(float  x, float  y);


        /**
         * Create an 2d vector with polar coordinates.
         * 
         * @param r the magnitude of the vector.
         * @param theta the angle/direction of the vector.
        */
        Vector2d(float  r, Angle2d theta);


        /**
         * @return The x coordinate of the vector.
        */
        float  getX();

        /**
         * @return The y coordinate of the vector.
        */
        float  getY();


        /**
         * @return The magnitude (r) of the vector.
        */
        float  getMagnitude();


        /** 
         * @return The angle(theta) of the vector as a Angle2d instance.
        */
        Angle2d getDirection();


        /**
         * @return A new instance of Vector2d which is this current vector rotated by the angle represented by "other".
         * 
         * For example, the vector <1,0> rotated by 90 degrees will be <0,1>;
         * 
         * the vector <1,0> rotated by 45 degrees will be <(sqrt2)/2, (sqrt2)/2>
         * 
        */
        Vector2d rotateBy(Angle2d other);

        /**
         * @returns This vector plus another vector.
         * For example, <2,1> plus <1,0> will be <3,1>.
         * 
        */
        Vector2d plus(Vector2d other);


        /**
         * @returns This vector minus another vector.
         * For example, <2,1> minus <1,0> will be <1,1>.
        */
        Vector2d minus(Vector2d other);


        /**
         * @returns The inverse of this vector.
         * For example, the inverse of <1,1> is <-1,-1>
        */
        Vector2d inverse();

        /**
         * @return The dot product of this and the other vector.
         * 
        */
        float  dot(Vector2d other);


        /**
         * @param scalar the scalar that you want to times the vector by.
         * @returns The scalar product of scalar times the vector.
        */
        Vector2d times(float  scalar);

        /**
         * @returns A string format of the vector
        */
        std::string toString();

        /**
         * @returns The normalized vector (magnitude 1).
        */
        Vector2d normalize();

        /**
         * @returns Whether the two vectors are equal.
        */
        bool equalsTo(Vector2d other);
};