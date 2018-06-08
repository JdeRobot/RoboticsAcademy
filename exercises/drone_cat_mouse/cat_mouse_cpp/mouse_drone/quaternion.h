#ifndef QUATERNION_H
#define QUATERNION_H

#define _USE_MATH_DEFINES
#include <math.h>

class Quaternion {
public:
    Quaternion(double, double, double, double);
    ~Quaternion();

    double QuatToRoll(double, double, double, double) const;
    double QuatToPitch(double, double, double, double) const;
    double QuatToYaw(double, double, double, double) const;
    double QuatToRoll() const;
    double QuatToPitch() const;
    double QuatToYaw() const;

    double qw, qx, qy, qz;
};

#endif //QUATERNION_H
