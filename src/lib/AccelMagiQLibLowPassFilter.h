#ifndef ACCELMAGIQLIB_LOWPASSFILTER_H
#define ACCELMAGIQLIB_LOWPASSFILTER_H

namespace accelmagiqlib
{

class LowPassFilter
{
public:
    static constexpr double DEFAULT_ALPHA = 0.8;

    LowPassFilter(const double newAlpha = DEFAULT_ALPHA);
    void setAlpha(const double alpha);
    double filter(const double input);

private:
    double alpha;
    double oneMinusAlpha;
    double prev;
};

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_LOWPASSFILTER_H
