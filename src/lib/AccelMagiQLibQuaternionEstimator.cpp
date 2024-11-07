#include "AccelMagiQLibQuaternionEstimator.h"
#include <cmath>

using namespace accelmagiqlib;

/**
 * Get the W component of the quaternion
 *
 * @return The W component of the quaternion
 */
double QuaternionEstimator::getW() const
{
    return qw;
}

/**
 * Get the X component of the quaternion
 *
 * @return The X component of the quaternion
 */
double QuaternionEstimator::getX() const
{
    return qx;
}

/**
 * Get the Y component of the quaternion
 *
 * @return The Y component of the quaternion
 */
double QuaternionEstimator::getY() const
{
    return qy;
}

/**
 * Get the Z component of the quaternion
 *
 * @return The Z component of the quaternion
 */
double QuaternionEstimator::getZ() const
{
    return qz;
}

/**
 * Set the alpha value for the low pass filters
 *
 * @param alpha The new alpha value. Should be in the range of 0.0 to 1.0.
 */
void QuaternionEstimator::setLowPassFilterAlpha(const double alpha)
{
    filterAccel.setAlpha(alpha);
    filterMagne.setAlpha(alpha);
}

/**
 * Update the accelerometer data
 *
 * This function updates and normalizes the accelerometer data.
 *
 * @param x The X component of the accelerometer data
 * @param y The Y component of the accelerometer data
 * @param z The Z component of the accelerometer data
 */
void QuaternionEstimator::accelerometerUpdate(const double x, const double y, const double z)
{
    // Update and normalize accelerometer data
    filterAccel.update(x, y, z);
}

/**
 * Update the magnetometer data
 *
 * This function updates and normalizes the magnetometer data.
 *
 * @param x The X component of the magnetometer data
 * @param y The Y component of the magnetometer data
 * @param z The Z component of the magnetometer data
 */
void QuaternionEstimator::magnetometerUpdate(const double x, const double y, const double z)
{
    // Update and normalize magnetometer data
    filterMagne.update(x, y, z);
}

/**
 * Set the method used for quaternion estimation
 *
 * @param method The method identifier to use for estimation
 */
void QuaternionEstimator::setEstimateMethod(const int method)
{
    currentMethod = method;
}
void QuaternionEstimator::setCoordinateSystem(const int system)
{
    filterAccel.setCoordinateSystem(system);
    filterMagne.setCoordinateSystem(system);
}
/**
 * Perform the quaternion estimation
 *
 * This function calculates the quaternion based on the current sensor data and the selected estimation method.
 */
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

/**
 * Estimate the quaternion using the Fast Accelerometer-Magnetometer Combination (FAMC) algorithm
 */
void QuaternionEstimator::estimateFamc()
{
    const double ax = filterAccel.getX();
    const double ay = filterAccel.getY();
    const double az = filterAccel.getZ();
    const double mx = filterMagne.getX();
    const double my = filterMagne.getY();
    const double mz = filterMagne.getZ();

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

    // Normalize the quaternion
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

/**
 * Estimate the quaternion using a simple method
 */
void QuaternionEstimator::estimateSimple()
{
    const double ax = filterAccel.getX();
    const double ay = filterAccel.getY();
    const double az = filterAccel.getZ();

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
