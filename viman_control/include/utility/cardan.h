#ifndef CARDAN_H
#define CARDAN_H

#ifndef IMU_MATH_H
	#include <cmath>
	#define RAD2DEG 57.2957795131
	#define DEG2RAD 0.01745329251
#endif

namespace imu{

class Quaternion;

class Cardan{

private:
    double _roll, _pitch, _yaw;

public:
    Cardan();

    Cardan(double r, double p, double y);

    double& roll();
    double& pitch();
    double& yaw();
    double roll() const;
    double pitch() const;
    double yaw() const;
	
	Quaternion toQuat() const;
};

}
#endif
