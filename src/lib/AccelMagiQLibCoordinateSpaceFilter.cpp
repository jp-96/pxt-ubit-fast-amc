#include "AccelMagiQLibCoordinateSpaceFilter.h"

namespace accelmagiqlib
{

    void CoordinateSpaceFilter::setCoordinateSystem(int system)
    {
        currentSystem = system;
    }

    double CoordinateSpaceFilter::getX() const
    {
        if (currentSystem == COORDINATE_SYSTEM_BASIC)
            return rawY;
        if (currentSystem == COORDINATE_SYSTEM_TILT)
            return rawZ;
        return rawX;
    }

    double CoordinateSpaceFilter::getY() const
    {
        if (currentSystem == COORDINATE_SYSTEM_BASIC)
            return rawX;
        if (currentSystem == COORDINATE_SYSTEM_TILT)
            return rawX;
        return rawY;
    }

    double CoordinateSpaceFilter::getZ() const
    {
        if (currentSystem == COORDINATE_SYSTEM_BASIC)
            return -rawZ;
        if (currentSystem == COORDINATE_SYSTEM_TILT)
            return rawY;
        return rawZ;
    }

    void CoordinateSpaceFilter::update(double newX, double newY, double newZ)
    {
        // Use the inline low-pass filter to update the raw values
        prevX = lowPassFilter(newX, prevX, alphaX, oneMinusAlphaX);
        prevY = lowPassFilter(newY, prevY, alphaY, oneMinusAlphaY);
        prevZ = lowPassFilter(newZ, prevZ, alphaZ, oneMinusAlphaZ);
        double norm = std::sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);
        if (0.0 < norm)
        {
            rawX = prevX / norm;
            rawY = prevY / norm;
            rawZ = prevZ / norm;
        }
    }

    void CoordinateSpaceFilter::setAlpha(double alpha)
    {
        alphaX = alpha;
        oneMinusAlphaX = 1.0 - alpha;
        alphaY = alpha;
        oneMinusAlphaY = 1.0 - alpha;
        alphaZ = alpha;
        oneMinusAlphaZ = 1.0 - alpha;
    }

    inline double CoordinateSpaceFilter::lowPassFilter(double newValue, double &oldValue, double alpha, double oneMinusAlpha)
    {
        oldValue = alpha * newValue + oneMinusAlpha * oldValue;
        return oldValue;
    }

} // namespace accelmagiqlib
