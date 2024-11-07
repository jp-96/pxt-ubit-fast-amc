#ifndef ACCELMAGIQLIB_COORDINATESPACEFILTER_H
#define ACCELMAGIQLIB_COORDINATESPACEFILTER_H

#include <cmath>

namespace accelmagiqlib
{

    // Define coordinate system constants
    const int COORDINATE_SYSTEM_BASIC = 0; /**< BASIC: a non-tilt compensated bearing of the device (North: logo mark) */
    const int COORDINATE_SYSTEM_TILT = 1;  /**< TILT: a tilt compensated bearing of the device (North: back side) */
    const int COORDINATE_SYSTEM_RAW = 2;   /**< RAW: a raw coordinate system (North: A-button, upside-down) */

    /**
     * @class CoordinateSpaceFilter
     * @brief A class to filter and manage coordinates with low-pass filtering and different coordinate systems.
     */
    class CoordinateSpaceFilter
    {
    private:
        int currentSystem; /**< The current coordinate system being used */

        // Low-pass filter parameters
        double alphaX;          /**< Alpha value for low-pass filter on X-axis */
        double oneMinusAlphaX;  /**< Precomputed (1.0 - alphaX) value */
        double alphaY;          /**< Alpha value for low-pass filter on Y-axis */
        double oneMinusAlphaY;  /**< Precomputed (1.0 - alphaY) value */
        double alphaZ;          /**< Alpha value for low-pass filter on Z-axis */
        double oneMinusAlphaZ;  /**< Precomputed (1.0 - alphaZ) value */
        double prevX;           /**< Previous filtered value for X-axis */
        double prevY;           /**< Previous filtered value for Y-axis */
        double prevZ;           /**< Previous filtered value for Z-axis */

        // Normalized coordinates
        double rawX; /**< Normalized X coordinate */
        double rawY; /**< Normalized Y coordinate */
        double rawZ; /**< Normalized Z coordinate */

    public:
        static constexpr double DEFAULT_ALPHA = 0.8; /**< Default alpha value for the low-pass filter */

        /**
         * Constructor to initialize the CoordinateSpaceFilter with initial coordinates
         * and an optional alpha value for the low-pass filter.
         *
         * @param x Initial X coordinate (default is 0.0).
         * @param y Initial Y coordinate (default is 0.0).
         * @param z Initial Z coordinate (default is 0.0).
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_BASIC: 0 (default)
         *               - COORDINATE_SYSTEM_TILT: 1
         *               - COORDINATE_SYSTEM_RAW: 2
         * @param alpha Alpha value for the low-pass filter (default is 0.8).
         */
        CoordinateSpaceFilter(const double x = 0.0, const double y = 0.0, const double z = 0.0,
                              const int system = COORDINATE_SYSTEM_BASIC, const double alpha = DEFAULT_ALPHA)
            : currentSystem(system),
              alphaX(alpha), oneMinusAlphaX(1.0 - alpha),
              alphaY(alpha), oneMinusAlphaY(1.0 - alpha),
              alphaZ(alpha), oneMinusAlphaZ(1.0 - alpha),
              prevX(x), prevY(y), prevZ(z),
              rawX(0.0), rawY(0.0), rawZ(0.0)
        {
            update(x, y, z);
        }

        /**
         * Sets the coordinate system for the filter.
         *
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_BASIC: 0
         *               - COORDINATE_SYSTEM_TILT: 1
         *               - COORDINATE_SYSTEM_RAW: 2
         */
        void setCoordinateSystem(const int system);

        /**
         * Gets the X coordinate based on the current coordinate system.
         *
         * @return The X coordinate value.
         */
        double getCoordX() const;

        /**
         * Gets the Y coordinate based on the current coordinate system.
         *
         * @return The Y coordinate value.
         */
        double getCoordY() const;

        /**
         * Gets the Z coordinate based on the current coordinate system.
         *
         * @return The Z coordinate value.
         */
        double getCoordZ() const;

        /**
         * Updates the coordinate values with new data.
         *
         * @param x The new X value.
         * @param y The new Y value.
         * @param z The new Z value.
         */
        void update(const double x, const double y, const double z);

        /**
         * Sets the alpha value for the low-pass filter.
         *
         * @param alpha The new alpha value.
         */
        void setAlpha(const double alpha);

    private:
        /**
         * Applies a low-pass filter to the given value.
         *
         * @param newValue The new value to filter.
         * @param oldValue The previous filtered value.
         * @param alpha The alpha value for the filter.
         * @param oneMinusAlpha Precomputed (1.0 - alpha) value.
         * @return The filtered value.
         */
        inline double lowPassFilter(double newValue, double &oldValue, double alpha, double oneMinusAlpha);
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_COORDINATESPACEFILTER_H
