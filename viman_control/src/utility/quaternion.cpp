#include "quaternion.h"
#include "cardan.h"

namespace imu{

    Quaternion::Quaternion(){
		_w = 1.0;
		_x = 0.0;
		_y = 0.0;
		_z = 0.0;
	}
	
	Quaternion::Quaternion(double w, double x, double y, double z){
		_w = w;
		_x = x;
		_y = y;
		_z = z;
	}

    double& Quaternion::w(){
        return _w;
    }
    
    double& Quaternion::x(){
        return _x;
    }
    double& Quaternion::y(){
        return _y;
    }
    double& Quaternion::z(){
        return _z;
    }

    double Quaternion::w() const{
        return _w;
    }
    double Quaternion::x() const{
        return _x;
    }
    double Quaternion::y() const{
        return _y;
    }
    double Quaternion::z() const{
        return _z;
    }

    double Quaternion::magnitude() const{
        return sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
    }

    void Quaternion::normalize(){
        double mag = magnitude();
        *this = this->scale(1/mag);
    }

    Quaternion Quaternion::conjugate() const{
        return Quaternion(_w, -_x, -_y, -_z);
    }

    Cardan Quaternion::toCardan() const{
        Cardan angles;
        double sqw = _w*_w;
        double sqx = _x*_x;
        double sqy = _y*_y;
        double sqz = _z*_z;
		
		angles.roll() = atan2(2.0*(_y*_z+_x*_w),(-sqx-sqy+sqz+sqw)) * RAD2DEG;
		angles.pitch() = asin(2.0*(_y*_w-_x*_z)/(sqx+sqy+sqz+sqw)) * RAD2DEG;
		angles.yaw() = atan2(2.0*(_x*_y+_z*_w),(sqx-sqy-sqz+sqw)) * RAD2DEG;
        
        return angles;
    }

    Quaternion Quaternion::operator*(const Quaternion& q) const{
        return Quaternion(
            _w*q._w - _x*q._x - _y*q._y - _z*q._z,
            _w*q._x + _x*q._w + _y*q._z - _z*q._y,
            _w*q._y - _x*q._z + _y*q._w + _z*q._x,
            _w*q._z + _x*q._y - _y*q._x + _z*q._w
        );
    }

    Quaternion Quaternion::operator+(const Quaternion& q) const{
        return Quaternion(_w + q._w, _x + q._x, _y + q._y, _z + q._z);
    }

    Quaternion Quaternion::operator-(const Quaternion& q) const{
        return Quaternion(_w - q._w, _x - q._x, _y - q._y, _z - q._z);
    }

    Quaternion Quaternion::operator/(double scalar) const{
        return Quaternion(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
    }

    Quaternion Quaternion::operator*(double scalar) const{
        return scale(scalar);
    }

    Quaternion Quaternion::scale(double scalar) const{
        return Quaternion(_w * scalar, _x * scalar, _y * scalar, _z * scalar);
    }

}
