#include "quaternion.h"

Quaternion::Quaternion(double qw, double qx, double qy, double qz) {
    this->qw = qw;
    this->qx = qx;
    this->qy = qy;
    this->qz = qz;
}

Quaternion::~Quaternion() {}

double Quaternion::QuatToRoll(double qw, double qx, double qy, double qz) const{
    double rotateXa0 = 2.0*(qy*qz + qw*qx);
    double rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
    double rotateX = 0.0;
    if (rotateXa0 != 0.0 && rotateXa1 != 0.0) 
        rotateX = atan2(rotateXa0, rotateXa1);

    return rotateX;
}

double Quaternion::QuatToPitch(double qw, double qx, double qy, double qz) const{
    double rotateYa0 = -2.0*(qx*qz - qw*qy);
    double rotateY = 0.0;
    if( rotateYa0 >= 1.0 )
        rotateY = M_PI/2.0;
    else if( rotateYa0 <= -1.0 )
        rotateY = -M_PI/2.0;
    else rotateY = asin(rotateYa0);

    return rotateY;
}

double Quaternion::QuatToYaw(double qw, double qx, double qy, double qz) const{
    double rotateZa0 = 2.0*(qx*qy + qw*qz);
    double rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
    double rotateZ = 0.0;
    if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
        rotateZ = atan2(rotateZa0, rotateZa1);

    return rotateZ;
}

double Quaternion::QuatToRoll() const{
    double rotateXa0 = 2.0*(qy*qz + qw*qx);
    double rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
    double rotateX = 0.0;
    if (rotateXa0 != 0.0 && rotateXa1 != 0.0) 
        rotateX = atan2(rotateXa0, rotateXa1);

    return rotateX;
}

double Quaternion::QuatToPitch() const{
    double rotateYa0 = -2.0*(qx*qz - qw*qy);
    double rotateY = 0.0;
    if( rotateYa0 >= 1.0 )
        rotateY = M_PI/2.0;
    else if( rotateYa0 <= -1.0 )
        rotateY = -M_PI/2.0;
    else rotateY = asin(rotateYa0);

    return rotateY;
}

double Quaternion::QuatToYaw() const{
    double rotateZa0 = 2.0*(qx*qy + qw*qz);
    double rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
    double rotateZ = 0.0;
    if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
        rotateZ = atan2(rotateZa0, rotateZa1);

    return rotateZ;
}
