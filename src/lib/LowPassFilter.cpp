#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(double newAlpha) : alpha(0.8), prev(0.0)
{
    setAlpha(newAlpha);
}

void LowPassFilter::setAlpha(double newAlpha)
{
    if ((0.0 > newAlpha) || (1.0 < newAlpha))
    {
        return;
    }
    alpha = newAlpha;
}

double LowPassFilter::filter(double input)
{
    prev = alpha * input + (1.0 - alpha) * prev;
    return prev;
}
