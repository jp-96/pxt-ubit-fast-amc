#include "AccelMagiQLibQuaternionEstimator.h"
#include <cmath>

using namespace accelmagiqlib;

QuaternionEstimator::QuaternionEstimator(const double newAlpha)
    : filterAx(newAlpha), filterAy(newAlpha), filterAz(newAlpha),
      filterMx(newAlpha), filterMy(newAlpha), filterMz(newAlpha),
      currentMethod(0) {}

double QuaternionEstimator::getQw() const
{
    return qw;
}

double QuaternionEstimator::getQx() const
{
    return qx;
}

double QuaternionEstimator::getQy() const
{
    return qy;
}

double QuaternionEstimator::getQz() const
{
    return qz;
}

void QuaternionEstimator::accelerometerUpdate(const double x, const double y, const double z)
{
    // Update and normalize accelerometer data
    ax = filterAx.filter(x);
    ay = filterAy.filter(y);
    az = filterAz.filter(z);
    double normA = std::sqrt(ax * ax + ay * ay + az * az);
    if (normA > 0.0)
    {
        ax /= normA;
        ay /= normA;
        az /= normA;
    }
}

void QuaternionEstimator::magnetometerUpdate(const double x, const double y, const double z)
{
    // Update and normalize magnetometer data
    mx = filterMx.filter(x);
    my = filterMy.filter(y);
    mz = filterMz.filter(z);
    double normM = std::sqrt(mx * mx + my * my + mz * mz);
    if (normM > 0.0)
    {
        mx /= normM;
        my /= normM;
        mz /= normM;
    }
}

void QuaternionEstimator::setEstimateMethod(const int method)
{
    currentMethod = method;
}

void QuaternionEstimator::estimate()
{
    if (currentMethod == 0)
    {
        estimateFamc();
    }
    else
    {
        estimateSimple();
    }
}

void QuaternionEstimator::estimateFamc()
{
    // ---------------------------------------------------------------------------------------------
    // A Simplified Analytic Attitude Determination Algorithm Using Accelerometer and Magnetometer
    // Fast Accelerometer-Magnetometer Combination (FAMC) algorithm by Zhuohua Liu and Jin Wu
    // ---------------------------------------------------------------------------------------------
    // https://github.com/zarathustr/Analytic-AMC/blob/master/FAMC.m

    // Dynamic magnetometer reference vector
    double m_D = ax * mx + ay * my + az * mz;
    double m_N = sqrt(1.0 - m_D * m_D);

    // Parameters
    double B11 = (m_N * mx) / 2.0;
    double B13 = ax / 2.0 + (m_D * mx) / 2.0;
    double B21 = (m_N * my) / 2.0;
    double B23 = ay / 2.0 + (m_D * my) / 2.0;
    double B31 = (m_N * mz) / 2.0;
    double B33 = az / 2.0 + (m_D * mz) / 2.0;

    double tau = B13 + B31;

    // First Row
    double p1 = B33 - B11 + 1.0;
    double A11 = -1.0 / p1;
    double A12 = B21 / p1;
    double A13 = tau / p1;

    // Second Row
    double p2 = -B21 * B21 / p1 + B11 + B33 + 1.0;
    double A21 = -B21 / (p1 * p2);
    double A22 = -1.0 / p2;
    double A23 = (B23 + B21 * tau / p1) / p2;

    // Third Row
    double p3 = p1 - 2.0 + tau * tau / p1 + A23 * A23 * p2;
    double A31 = (tau / p1 + B21 * A23 / p1) / p3;
    double A32 = A23 / p3;
    double A33 = 1.0 / p3;

    // Quaternion Elements
    double a1 = B23 * (A11 + A12 * (A21 + A23 * A31) + A13 * A31);
    double a2 = (B13 - B31) * (A21 + A23 * A31);
    double a3 = A31 * B21;
    double b1 = B23 * (A12 * (A22 + A23 * A32) + A13 * A32);
    double b2 = (B13 - B31) * (A22 + A23 * A32);
    double b3 = A32 * B21;
    double c1 = B23 * (A13 * A33 + A12 * A23 * A33);
    double c2 = A33 * B21;
    double c3 = A23 * A33 * (B13 - B31);

    // Quaternion
    double w = -1.0;
    double x = a1 - a2 - a3;
    double y = b1 - b2 - b3;
    double z = c1 - c2 - c3;

    // normalize
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (0 < norm)
    {
        norm = 1 / norm;
        qw = w * norm;
        qx = x * norm;
        qy = y * norm;
        qz = z * norm;
    }
}

void QuaternionEstimator::estimateSimple()
{
    // Accelerration Only
    double w = std::sqrt((az + 1.0) / 2.0);
    double x = ay / (2.0 * w);
    double y = -ax / (2.0 * w);
    double z = 0.0;

    // normalize
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (0 < norm)
    {
        norm = 1 / norm;
        qw = w * norm;
        qx = x * norm;
        qy = y * norm;
        qz = z * norm;
    }
}
