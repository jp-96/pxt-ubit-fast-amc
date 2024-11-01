#include "LowPassFilter.h"

LowPassFilter::LowPassFilter() : alpha(1.0), prev(0.0) {}

void LowPassFilter::setAlpha(double newAlpha)
{
    if (newAlpha < 0.0)
    {
        newAlpha = 0.0;
    }
    else if (newAlpha > 1.0)
    {
        newAlpha = 1.0;
    }
    alpha = newAlpha;
}

double LowPassFilter::filter(double input)
{
    prev = alpha * input + (1.0 - alpha) * prev;
    return prev;
}
