#ifndef ACCELMAGIQLIB_COORDINATESPACEFILTER_H
#define ACCELMAGIQLIB_COORDINATESPACEFILTER_H

#include <cmath>

namespace accelmagiqlib
{

    // Define coordinate system constants
    const int COORDINATE_SYSTEM_BASIC = 0; /**< BASIC: a non-tilt compensated bearing of the device (North: logo mark) */
    const int COORDINATE_SYSTEM_TILT = 1;  /**< TILT: a tilt compensated bearing of the device (North: back side) */
    const int COORDINATE_SYSTEM_RAW = 2;   /**< RAW (North: A-button, upside-down) */

    class CoordinateSpaceFilter
    {
    private:
        int currentSystem;

        // low-pass filter
        double alphaX;
        double oneMinusAlphaX;
        double alphaY;
        double oneMinusAlphaY;
        double alphaZ;
        double oneMinusAlphaZ;
        double prevX;
        double prevY;
        double prevZ;

        // normalized
        double rawX;
        double rawY;
        double rawZ;

    public:
        static constexpr double DEFAULT_ALPHA = 0.8;

        /**
         * Constructor to initialize the CoordinateSpaceFilter with initial coordinates
         * and an optional alpha value for the low-pass filter.
         *
         * @param x Initial X coordinate (default is 0.0).
         * @param y Initial Y coordinate (default is 0.0).
         * @param z Initial Z coordinate (default is 0.0).
         * @param system The coordinate system to use:
         *               - COORD_SYSTEM_BASIC: 0 (default)
         *               - COORD_SYSTEM_TILT: 1
         *               - COORD_SYSTEM_RAW: 2
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
         *               - COORD_SYSTEM_BASIC: 0
         *               - COORD_SYSTEM_TILT: 1
         *               - COORD_SYSTEM_RAW: 2
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
