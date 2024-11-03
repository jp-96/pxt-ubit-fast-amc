#ifndef ACCELMAGIQ_LOWPASSFILTER_H
#define ACCELMAGIQ_LOWPASSFILTER_H

class LowPassFilter
{
public:
    LowPassFilter(double newAlpha = 0.8);
    void setAlpha(double alpha);
    double filter(double input);

private:
    double alpha;
    double prev;
};

#endif // ACCELMAGIQ_LOWPASSFILTER_H
