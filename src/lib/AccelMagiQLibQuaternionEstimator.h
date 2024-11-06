#ifndef ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
#define ACCELMAGIQLIB_QUATERNIONESTIMATOR_H

#include "AccelMagiQLibLowPassFilter.h"

namespace accelmagiqlib
{

    /**
     * QuaternionEstimator class
     *
     * This class estimates quaternion orientation based on accelerometer and magnetometer data.
     */
    class QuaternionEstimator
    {
    public:
        /**
         * Constructor
         *
         * Initializes the quaternion estimator with a specified alpha value for the low pass filters.
         *
         * @param alpha The alpha value for the low pass filters. Default is 0.8.
         */
        QuaternionEstimator(const double alpha = LowPassFilter::DEFAULT_ALPHA);

        // Getters for quaternion components
        /**
         * @brief Get the W component of the quaternion
         *
         * @return The W component of the quaternion
         */
        double getW() const;

        /**
         * @brief Get the X component of the quaternion
         *
         * @return The X component of the quaternion
         */
        double getX() const;

        /**
         * @brief Get the Y component of the quaternion
         *
         * @return The Y component of the quaternion
         */
        double getY() const;

        /**
         * @brief Get the Z component of the quaternion
         *
         * @return The Z component of the quaternion
         */
        double getZ() const;

        /**
         * @brief Set the alpha value for the low pass filters
         *
         * @param alpha The new alpha value. Should be in the range of 0.0 to 1.0.
         */
        void setLowPassFilterAlpha(const double alpha);

        // Update functions
        /**
         * @brief Update the accelerometer data
         *
         * This function updates the internal state of the accelerometer filters with new data.
         *
         * @param x The X component of the accelerometer data
         * @param y The Y component of the accelerometer data
         * @param z The Z component of the accelerometer data
         */
        void accelerometerUpdate(const double x, const double y, const double z);

        /**
         * @brief Update the magnetometer data
         *
         * This function updates the internal state of the magnetometer filters with new data.
         *
         * @param x The X component of the magnetometer data
         * @param y The Y component of the magnetometer data
         * @param z The Z component of the magnetometer data
         */
        void magnetometerUpdate(const double x, const double y, const double z);

        /**
         * @brief Set the method used for quaternion estimation
         *
         * @param method The method identifier to use for estimation: 0-FAMC, 1-SIMPLE
         */
        void setEstimateMethod(const int method);

        /**
         * @brief Perform the quaternion estimation
         *
         * This function calculates the quaternion based on the current sensor data and the selected estimation method.
         */
        void estimate();

    private:
        /**
         * @brief Estimate the quaternion using the Fast Accelerometer-Magnetometer Combination (FAMC) algorithm
         */
        void estimateFamc();

        /**
         * @brief Estimate the quaternion using a simple method
         */
        void estimateSimple();

        // Estimation method
        int currentMethod; /**< The currently selected method identifier to use for estimation: 0-FAMC, 1-SIMPLE*/

        // Acceleration filter
        LowPassFilter filterAx; /**< Low pass filter for the X component of the accelerometer */
        LowPassFilter filterAy; /**< Low pass filter for the Y component of the accelerometer */
        LowPassFilter filterAz; /**< Low pass filter for the Z component of the accelerometer */

        // Magnetic force filter
        LowPassFilter filterMx; /**< Low pass filter for the X component of the magnetometer */
        LowPassFilter filterMy; /**< Low pass filter for the Y component of the magnetometer */
        LowPassFilter filterMz; /**< Low pass filter for the Z component of the magnetometer */

        // Acceleration (normalized)
        double ax = 0.0; /**< Normalized X component of the acceleration */
        double ay = 0.0; /**< Normalized Y component of the acceleration */
        double az = 1.0; /**< Normalized Z component of the acceleration */

        // Magnetic force (normalized)
        double mx = 0.0; /**< Normalized X component of the magnetic force */
        double my = 0.0; /**< Normalized Y component of the magnetic force */
        double mz = 1.0; /**< Normalized Z component of the magnetic force */

        // Quaternion (normalized)
        double qw = 1.0; /**< W component of the quaternion */
        double qx = 0.0; /**< X component of the quaternion */
        double qy = 0.0; /**< Y component of the quaternion */
        double qz = 0.0; /**< Z component of the quaternion */
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
