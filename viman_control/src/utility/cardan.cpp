#include "quaternion.h"
#include "cardan.h"

namespace imu{


    Cardan::Cardan(){
		_roll = 0.0;
		_pitch = 0.0;
		_yaw = 0.0;
	}
	
	Cardan::Cardan(double r, double p, double y){
		_roll = r*DEG2RAD;
		_pitch = p*DEG2RAD;
		_yaw = y*DEG2RAD;
	}

    double& Cardan::roll(){
        return _roll;
    }
    double& Cardan::pitch(){
        return _pitch;
    }
    double& Cardan::yaw(){
        return _yaw;
    }
    double Cardan::roll() const{
        return _roll*RAD2DEG;
    }
    double Cardan::pitch() const{
        return _pitch*RAD2DEG;
    }
    double Cardan::yaw() const{
        return _yaw*RAD2DEG;
    }
	
	Quaternion Cardan::toQuat() const{
		double cy = cos(_yaw * 0.5);
		double sy = sin(_yaw * 0.5);
		double cp = cos(_pitch * 0.5);
		double sp = sin(_pitch * 0.5);
		double cr = cos(_roll * 0.5);
		double sr = sin(_roll * 0.5);

		Quaternion q;
		q.w() = cr * cp * cy + sr * sp * sy;
		q.x() = sr * cp * cy - cr * sp * sy;
		q.y() = cr * sp * cy + sr * cp * sy;
		q.z() = cr * cp * sy - sr * sp * cy;

		return q;
	}
	
}
