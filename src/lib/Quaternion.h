#ifndef QUATERNION_H
#define QUATERNION_H

#include "EulerAngles.h"

class Quaternion
{
public:
    float w, x, y, z;

    Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f);

    Quaternion normalize() const;

    Quaternion operator*(const Quaternion &q) const;

    Quaternion &operator*=(const Quaternion &q);

    Quaternion operator/(const Quaternion &q) const;

    Quaternion &operator/=(const Quaternion &q);

    EulerAngles toEulerAngles() const;

private:
    Quaternion inverse() const;
};

#endif // QUATERNION_H
