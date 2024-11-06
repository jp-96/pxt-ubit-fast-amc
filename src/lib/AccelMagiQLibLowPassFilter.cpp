#include "AccelMagiQLibLowPassFilter.h"

using namespace accelmagiqlib;

/**
 * Constructor
 *
 * Initializes the low pass filter with a specified alpha value.
 *
 * @param alpha The alpha value. Default is 0.8.
 */
LowPassFilter::LowPassFilter(const double alpha) : alpha(DEFAULT_ALPHA), prev(0.0)
{
    setAlpha(alpha);
}

/**
 * Set the alpha value
 *
 * This method sets the alpha value used in the filter. The alpha value determines
 * the weight of the current input versus the previous filtered value.
 *
 * @param newAlpha The new alpha value. Should be in the range of 0.0 to 1.0.
 */
void LowPassFilter::setAlpha(const double newAlpha)
{
    if ((0.0 <= newAlpha) && (1.0 >= newAlpha))
    {
        alpha = newAlpha;
        oneMinusAlpha = 1.0 - newAlpha;
    }
}

/**
 * Apply the filter
 *
 * This method applies the low pass filter to the given input value. The filter
 * smooths the input data by considering both the current input and the
 * previous filtered value.
 *
 * @param input The input data
 * @return Filtered data
 */
double LowPassFilter::filter(const double input)
{
    prev = alpha * input + oneMinusAlpha * prev;
    return prev;
}
