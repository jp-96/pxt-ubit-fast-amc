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
     * QuaternionEstimator class
     *
     * This class estimates quaternion orientation based on accelerometer and magnetometer data.
     */
    class QuaternionEstimator
    {
    public:
        /**
         * Constructor
         */
        QuaternionEstimator()
            : currentMethod(ESTIMATION_METHOD_FAMC), isSampling(false)
        {
            resumeSampling();
        }

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

        void resumeSampling();

        void pauseSampling();

    private:
        bool isSampling;

        /**
         * Accelerometer update callback
         */
        void accelerometerUpdateHandler(MicroBitEvent e);

        /**
         * Magnetometer update callback
         */
        void magnetometerUpdateHandler(MicroBitEvent e);

    public:

        /**
         * @brief Set the method used for quaternion estimation
         *
         * @param method The method identifier to use for estimation: 0-FAMC, 1-SIMPLE
         */
        void setEstimateMethod(const int method);

        /**
         * @brief Sets the coordinate system for the filter.
         *
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_BASIC: 0
         *               - COORDINATE_SYSTEM_TILT: 1
         *               - COORDINATE_SYSTEM_RAW: 2
         */
        void setCoordinateSystem(const int system);

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
        CoordinateSpaceFilter filterAccel;

        // Magnetic force filter
        CoordinateSpaceFilter filterMagne;

        // Quaternion (normalized)
        double qw = 1.0; /**< W component of the quaternion */
        double qx = 0.0; /**< X component of the quaternion */
        double qy = 0.0; /**< Y component of the quaternion */
        double qz = 0.0; /**< Z component of the quaternion */
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
