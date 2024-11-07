#include "AccelMagiQLibCoordinateSpaceFilter.h"

namespace accelmagiqlib
{

    void CoordinateSpaceFilter::setCoordinateSystem(const int system)
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

    void CoordinateSpaceFilter::update(const double x, const double y, const double z)
    {
        // Use the inline low-pass filter to update the raw values
        prevX = lowPassFilter(x, prevX, alphaX, oneMinusAlphaX);
        prevY = lowPassFilter(y, prevY, alphaY, oneMinusAlphaY);
        prevZ = lowPassFilter(z, prevZ, alphaZ, oneMinusAlphaZ);
        double norm = std::sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);
        if (0.0 < norm)
        {
            rawX = prevX / norm;
            rawY = prevY / norm;
            rawZ = prevZ / norm;
        }
    }

    void CoordinateSpaceFilter::setAlpha(const double alpha)
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
