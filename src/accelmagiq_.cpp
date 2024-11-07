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
    void setCoordinateSystem(int system)
    {
        instance().setCoordinateSystem(system);
    }

    //%
    void setLowPassFilterAlpha(TNumber alpha)
    {
        instance().setLowPassFilterAlpha(toDouble(alpha));
    }

    //%
    void setEstimateMethod(int method)
    {
        instance().setEstimateMethod(method);
    }

    //%
    void startSampling()
    {
        instance().resumeSampling();
    }

    //%
    void stopSampling()
    {
        instance().pauseSampling();
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
