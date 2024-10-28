#include "FAMC.h"
#include "FastMath.h"
#include <cmath>
#include <stdexcept>

void FAMC::estimate(const std::vector<float>& acc, const std::vector<float>& mag) {
    // Normalize measurements
    float a_norm = FastMath::fastInverseSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    float m_norm = FastMath::fastInverseSqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);

    float ax = acc[0] * a_norm;
    float ay = acc[1] * a_norm;
    float az = acc[2] * a_norm;

    float mx = mag[0] * m_norm;
    float my = mag[1] * m_norm;
    float mz = mag[2] * m_norm;

    // Dynamic magnetometer reference vector
    float m_D = ax * mx + ay * my + az * mz;
    float m_N = std::sqrt(1.0f - m_D * m_D);

    // Parameters
    std::vector<std::vector<float>> B(3, std::vector<float>(3, 0.0f));
    B[0] = {m_N * mx, m_N * my, m_N * mz};
    B[2] = {m_D * mx + ax, m_D * my + ay, m_D * mz + az};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            B[i][j] *= 0.5f;
        }
    }

    float tau = B[0][2] + B[2][0];
    std::vector<float> alpha(3, 0.0f);
    std::vector<std::vector<float>> Y(3, std::vector<float>(3, 0.0f));

    // First Row
    alpha[0] = B[2][2] - B[0][0] + 1.0f;
    Y[0][0] = -1.0f / alpha[0];
    Y[0][1] = B[1][0] / alpha[0];
    Y[0][2] = tau / alpha[0];

    // Second Row
    alpha[1] = -B[1][0] * B[1][0] / alpha[0] + B[0][0] + B[2][2] + 1.0f;
    Y[1][0] = -B[1][0] / (alpha[0] * alpha[1]);
    Y[1][1] = -1.0f / alpha[1];
    Y[1][2] = (B[1][2] + B[1][0] * tau / alpha[0]) / alpha[1];

    // Third row
    alpha[2] = alpha[0] - 2.0f + tau * tau / alpha[0] + Y[1][2] * Y[1][2] * alpha[1];
    Y[2][0] = (tau / alpha[0] + B[1][0] * Y[1][2] / alpha[0]) / alpha[2];
    Y[2][1] = Y[1][2] / alpha[2];
    Y[2][2] = 1.0f / alpha[2];

    // Quaternion Elements
    float a = B[1][2] * (Y[0][0] + Y[0][1] * (Y[1][2] * Y[2][0] + Y[1][0]) + Y[0][2] * Y[2][0]) -
              (B[0][2] - B[2][0]) * (Y[1][2] * Y[2][0] + Y[1][0]) - Y[2][0] * B[1][0];
    float b = B[1][2] * (Y[0][1] * (Y[1][2] * Y[2][1] + Y[1][1]) + Y[0][2] * Y[2][1]) -
              (B[0][2] - B[2][0]) * (Y[1][2] * Y[2][1] + Y[1][1]) - Y[2][1] * B[1][0];
    float c = B[1][2] * (Y[0][1] * Y[1][2] * Y[2][2] + Y[0][2] * Y[2][2]) -
              (B[0][2] - B[2][0]) * (Y[1][2] * Y[2][2]) - Y[2][2] * B[1][0];

    w = -1.0f;
    x = a;
    y = b;
    z = c;
}
