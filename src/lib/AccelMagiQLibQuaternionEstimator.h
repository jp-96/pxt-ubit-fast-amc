#ifndef ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
#define ACCELMAGIQLIB_QUATERNIONESTIMATOR_H

#include "AccelMagiQLibLowPassFilter.h"

namespace accelmagiqlib
{

class QuaternionEstimator {
public:

    QuaternionEstimator(const double newAlpha = LowPassFilter::DEFAULT_ALPHA);

    // Getters for quaternion components
    double getQw() const;
    double getQx() const;
    double getQy() const;
    double getQz() const;

    // Update functions
    void accelerometerUpdate(const double x, const double y, const double z);
    void magnetometerUpdate(const double x, const double y, const double z);

    // Setter for EstimateMethod
    void setEstimateMethod(const int method);

    // Estimate function
    void estimate();

private:
    void estimateFamc();
    void estimateSimple();

    int currentMethod;

    // Acceleration filter
    LowPassFilter filterAx;
    LowPassFilter filterAy;
    LowPassFilter filterAz;

    // Magnetic force filter
    LowPassFilter filterMx;
    LowPassFilter filterMy;
    LowPassFilter filterMz;

    // Acceleration (normalized)
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
};

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
