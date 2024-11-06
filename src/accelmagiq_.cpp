#include "pxt.h"
#include "lib/AccelMagiQLibQuaternionEstimator.h"

using namespace accelmagiqlib;

namespace
{
    static QuaternionEstimator *_pQuaternionEstimator = nullptr;
    QuaternionEstimator &instance()
    {
        if (nullptr == _pQuaternionEstimator)
            _pQuaternionEstimator = new QuaternionEstimator();
        return *_pQuaternionEstimator;
    }
}

namespace accelmagiq_
{

    //%
    void setLowPassFilterAlpha(TNumber alpha)
    {
        instance().setLowPassFilterAlpha(toDouble(alpha));
    }

    //%
    void updateAcceleration(TNumber accX, TNumber accY, TNumber accZ)
    {
        instance().accelerometerUpdate(toDouble(accX), toDouble(accY), toDouble(accZ));
    }

    //%
    void updateMagneticForce(TNumber magX, TNumber magY, TNumber magZ)
    {
        instance().magnetometerUpdate(toDouble(magX), toDouble(magY), toDouble(magZ));
    }

    //%
    void estimate()
    {
        instance().estimate();
    }

    //%
    TNumber getW()
    {
        return fromDouble(instance().getW());
    }

    //%
    TNumber getX()
    {
        return fromDouble(instance().getX());
    }

    //%
    TNumber getY()
    {
        return fromDouble(instance().getY());
    }

    //%
    TNumber getZ()
    {
        return fromDouble(instance().getZ());
    }

}
