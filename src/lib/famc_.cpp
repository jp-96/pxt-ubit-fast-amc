#include "pxt.h"
#include "LowPassFilter.h"

namespace accelmagicfamc_
{

    static const double DEFAULT_ALPHA = 0.8;

    // Accelerration filter
    LowPassFilter filterAx(DEFAULT_ALPHA);
    LowPassFilter filterAy(DEFAULT_ALPHA);
    LowPassFilter filterAz(DEFAULT_ALPHA);

    // Magnetic force filter
    LowPassFilter filterMx(DEFAULT_ALPHA);
    LowPassFilter filterMy(DEFAULT_ALPHA);
    LowPassFilter filterMz(DEFAULT_ALPHA);

    // Accelerration (normalized)
    double ax = 0.0;
    double ay = 0.0;
    double az = 1.0;

    // Magnetic force (normalized)
    double mx = 0.0;
    double my = 0.0;
    double mz = 1.0;

    // Quaternion (normalized)
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    //%
    void setLowPassFilterAlpha(TNumber alpha)
    {
        const double newAlpha = toDouble(alpha);
        filterAx.setAlpha(newAlpha);
        filterAy.setAlpha(newAlpha);
        filterAz.setAlpha(newAlpha);
        filterMx.setAlpha(newAlpha);
        filterMy.setAlpha(newAlpha);
        filterMz.setAlpha(newAlpha);
    }

    //%
    void updateAcceleration(TNumber accX, TNumber accY, TNumber accZ)
    {
        const double x = filterAx.filter(toDouble(accX));
        const double y = filterAy.filter(toDouble(accY));
        const double z = filterAz.filter(toDouble(accZ));
        double norm = sqrt(x * x + y * y + z * z);
        if (0 < norm)
        {
            norm = 1 / norm;
            ax = x * norm;
            ay = y * norm;
            az = z * norm;
        }
    }

    //%
    void updateMagneticForce(TNumber magX, TNumber magY, TNumber magZ)
    {
        const double x = filterMx.filter(toDouble(magX));
        const double y = filterMy.filter(toDouble(magY));
        const double z = filterMz.filter(toDouble(magZ));
        double norm = sqrt(x * x + y * y + z * z);
        if (0 < norm)
        {
            norm = 1 / norm;
            mx = x * norm;
            my = y * norm;
            mz = z * norm;
        }
    }

    //%
    void estimate()
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

    //%
    TNumber getW()
    {
        return fromDouble(qw);
    }

    //%
    TNumber getX()
    {
        return fromDouble(qx);
    }

    //%
    TNumber getY()
    {
        return fromDouble(qy);
    }

    //%
    TNumber getZ()
    {
        return fromDouble(qz);
    }

}
