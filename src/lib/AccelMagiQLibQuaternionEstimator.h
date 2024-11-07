#ifndef ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
#define ACCELMAGIQLIB_QUATERNIONESTIMATOR_H

#include "pxt.h"
#include "AccelMagiQLibCoordinateSpaceFilter.h"

namespace accelmagiqlib
{

    // The method identifier to use for estimation: 0-FAMC, 1-SIMPLE
    const int ESTIMATION_METHOD_FAMC = 0;   /**< FAMC method */
    const int ESTIMATION_METHOD_SIMPLE = 1; /**< Simple method */

    /**
     * @class QuaternionEstimator
     * @brief This class estimates quaternion orientation based on accelerometer and magnetometer data.
     */
    class QuaternionEstimator
    {
    public:
        /**
         * @brief Constructor to initialize the QuaternionEstimator.
         */
        QuaternionEstimator()
            : currentMethod(ESTIMATION_METHOD_FAMC),
              isSampling(false),
              filterAccel(), filterMagne(),
              qw(1.0), qx(0.0), qy(0.0), qz(0.0) {}

        // Getters for quaternion components
        /**
         * @brief Get the W component of the quaternion.
         * @return The W component of the quaternion.
         */
        double getW() const;

        /**
         * @brief Get the X component of the quaternion.
         * @return The X component of the quaternion.
         */
        double getX() const;

        /**
         * @brief Get the Y component of the quaternion.
         * @return The Y component of the quaternion.
         */
        double getY() const;

        /**
         * @brief Get the Z component of the quaternion.
         * @return The Z component of the quaternion.
         */
        double getZ() const;

        /**
         * @brief Set the alpha value for the low-pass filters.
         * @param alpha The new alpha value. Should be in the range of 0.0 to 1.0.
         */
        void setLowPassFilterAlpha(const double alpha);

        /**
         * @brief Resume sampling sensor data.
         */
        void resumeSampling();

        /**
         * @brief Pause sampling sensor data.
         */
        void pauseSampling();

    private:
        bool isSampling; /**< Indicates whether the sampling of sensor data is active */

        /**
         * @brief Callback for accelerometer updates.
         * @param e The MicroBitEvent triggered by the accelerometer.
         */
        void accelerometerUpdateHandler(MicroBitEvent e);

        /**
         * @brief Callback for magnetometer updates.
         * @param e The MicroBitEvent triggered by the magnetometer.
         */
        void magnetometerUpdateHandler(MicroBitEvent e);

    public:
        /**
         * @brief Set the method used for quaternion estimation.
         * @param method The method identifier to use for estimation: 0-FAMC, 1-SIMPLE.
         */
        void setEstimateMethod(const int method);

        /**
         * @brief Sets the coordinate system for the filter.
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_BASIC: 0
         *               - COORDINATE_SYSTEM_TILT: 1
         *               - COORDINATE_SYSTEM_RAW: 2
         */
        void setCoordinateSystem(const int system);

        /**
         * @brief Perform the quaternion estimation.
         *
         * This function calculates the quaternion based on the current sensor data and the selected estimation method.
         */
        void estimate();

    private:
        /**
         * @brief Estimate the quaternion using the Fast Accelerometer-Magnetometer Combination (FAMC) algorithm.
         */
        void estimateFamc();

        /**
         * @brief Estimate the quaternion using a simple method.
         */
        void estimateSimple();

        // Estimation method
        int currentMethod; /**< The currently selected method identifier to use for estimation: 0-FAMC, 1-SIMPLE */

        // Acceleration filter
        CoordinateSpaceFilter filterAccel; /**< Filter for accelerometer data */

        // Magnetic force filter
        CoordinateSpaceFilter filterMagne; /**< Filter for magnetometer data */

        // Quaternion (normalized)
        double qw; /**< W component of the quaternion */
        double qx; /**< X component of the quaternion */
        double qy; /**< Y component of the quaternion */
        double qz; /**< Z component of the quaternion */
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
