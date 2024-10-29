#include "Quaternion.h"
#include "FastMath.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Quaternion::Quaternion(float w, float x, float y, float z)
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::normalize() const
{
    float norm = FastMath::fastInverseSqrt(w * w + x * x + y * y + z * z);
    return Quaternion(w * norm, x * norm, y * norm, z * norm);
}

Quaternion Quaternion::operator*(const Quaternion &q) const
{
    return Quaternion(
               w * q.w - x * q.x - y * q.y - z * q.z,
               w * q.x + x * q.w + y * q.z - z * q.y,
               w * q.y - x * q.z + y * q.w + z * q.x,
               w * q.z + x * q.y - y * q.x + z * q.w)
        .normalize();
}

Quaternion &Quaternion::operator*=(const Quaternion &q)
{
    float newW = w * q.w - x * q.x - y * q.y - z * q.z;
    float newX = w * q.x + x * q.w + y * q.z - z * q.y;
    float newY = w * q.y - x * q.z + y * q.w + z * q.x;
    float newZ = w * q.z + x * q.y - y * q.x + z * q.w;

    w = newW;
    x = newX;
    y = newY;
    z = newZ;

    *this = normalize();
    return *this;
}

Quaternion Quaternion::inverse() const
{
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::operator/(const Quaternion &q) const
{
    return *this * q.inverse();
}

Quaternion &Quaternion::operator/=(const Quaternion &q)
{
    *this *= q.inverse();
    return *this;
}

EulerAngles Quaternion::toEulerAngles() const {
    EulerAngles angles;

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f)
        angles.pitch = std::copysign(M_PI / 2.0f, sinp);
    else
        angles.pitch = std::asin(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
