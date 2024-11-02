#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

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

#endif // LOWPASSFILTER_H
