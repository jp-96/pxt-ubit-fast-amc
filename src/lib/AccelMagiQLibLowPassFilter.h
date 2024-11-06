#ifndef ACCELMAGIQLIB_LOWPASSFILTER_H
#define ACCELMAGIQLIB_LOWPASSFILTER_H

namespace accelmagiqlib
{

/**
 * Low Pass Filter class
 * 
 * This class applies a low pass filter to input data using the specified alpha value.
 * It is useful for smoothing out noise from sensor data.
 */
class LowPassFilter
{
public:
    /**
     * Default alpha value
     */
    static constexpr double DEFAULT_ALPHA = 0.8;

    /**
     * Constructor
     * 
     * Initializes the low pass filter with a specified alpha value.
     * 
     * @param alpha The alpha value. Default is 0.8.
     */
    LowPassFilter(const double alpha = DEFAULT_ALPHA);

    /**
     * Set the alpha value
     * 
     * This method sets the alpha value used in the filter. The alpha value determines
     * the weight of the current input versus the previous filtered value.
     * 
     * @param alpha The new alpha value. Should be in the range of 0.0 to 1.0.
     */
    void setAlpha(const double alpha);

    /**
     * Apply the filter
     * 
     * This method applies the low pass filter to the given input value.
     * 
     * @param input The input data
     * @return Filtered data
     */
    double filter(const double input);

private:
    double alpha;          /**< Filter coefficient, representing the weight of the current input */
    double oneMinusAlpha;  /**< 1.0 - alpha value, representing the weight of the previous filtered value */
    double prev;           /**< Previous filtered value */
};

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_LOWPASSFILTER_H
