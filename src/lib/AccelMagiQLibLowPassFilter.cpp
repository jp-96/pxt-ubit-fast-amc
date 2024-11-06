#include "AccelMagiQLibLowPassFilter.h"

using namespace accelmagiqlib;

LowPassFilter::LowPassFilter(const double newAlpha) : alpha(DEFAULT_ALPHA), prev(0.0)
{
    setAlpha(newAlpha);
}

void LowPassFilter::setAlpha(const double newAlpha)
{
    if ((newAlpha < 0.0) || (newAlpha > 1.0))
    {
        alpha = 1.0;
    }
    alpha = newAlpha;
    oneMinusAlpha = 1.0 - alpha;
}

double LowPassFilter::filter(const double input)
{
    prev = alpha * input + oneMinusAlpha * prev;
    return prev;
}
