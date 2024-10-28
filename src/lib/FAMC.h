#ifndef FAMC_H
#define FAMC_H

#include <vector>

class FAMC {
public:
    float w, x, y, z;
    void estimate(const std::vector<float>& acc, const std::vector<float>& mag);
};

#endif // FAMC_H
