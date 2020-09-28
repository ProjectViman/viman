#ifndef QUATERNION_H
#define QUATERNION_H

#ifndef IMU_MATH_H
	#include <cmath>
	#define RAD2DEG 57.2957795131
	#define DEG2RAD 0.01745329251
#endif

#include "cardan.h"

namespace imu{

class Cardan;

class Quaternion{

private:
    double _w, _x, _y, _z;

public:
    Quaternion();

    Quaternion(double w, double x, double y, double z);

    double& w();
    double& x();
    double& y();
    double& z();

    double w() const;
    double x() const;
    double y() const;
    double z() const;

    double magnitude() const;

    void normalize();

    Quaternion conjugate() const;

    Cardan toCardan() const;

    Quaternion operator*(const Quaternion& q) const;

    Quaternion operator+(const Quaternion& q) const;

    Quaternion operator-(const Quaternion& q) const;

    Quaternion operator/(double scalar) const;

    Quaternion operator*(double scalar) const;

    Quaternion scale(double scalar) const;
};

}
#endif
