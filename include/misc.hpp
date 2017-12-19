/* Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef _utility_Misc_hpp_
#define _utility_Misc_hpp_

// SYSTEM INCLUDES
#include <cmath>

namespace utility {

/**
 * Implement miscellaneous functions.
 */
class Misc {
public:

// LIFECYCLE

    /**
     * Delete constructor/destructor as this class uses only static operations.
     */
    Misc() = delete;
    ~Misc() = delete;

// OPERATIONS

    /**
     * Translate a value from one range to the value in another.
     *
     * @param value - the position to translate
     * @param left_min - minimum value of input range
     * @param left_max - maximum value of input range
     * @param right_min - minimum value of output range
     * @param right_max - maximum value of output range
     * @return the translated value
     */
    template<typename T>
    static T translate(
            double value,
            double left_min,
            double left_max,
            double right_min,
            double right_max);

    /**
     * Get a sign of the provided value.
     *
     * @param val - the value to get the sign of
     * @return - 1 if the value is positive, -1 otherwise
     */
	template <typename T>
    static int8_t sign(T val);

    /**
     * Convert angle in degrees to radians.
     *
     * @param degrees - the angle in degrees
     */
    static double toRadians(uint8_t degrees);

    /**
     * Convert angle in radians to degrees.
     *
     * @param radians - the angle in radians
     */
    static double toDegrees(double radians);

    /**
     * Calculate angle delta squared.
     *
     * @param angle1
     * @param angle2
     */
    static double delta(double angle1, double angle2);

// ATTRIBUTES

    constexpr static const double PI        = 3.14159;
    constexpr static const double TWO_PI    = PI * 2;
    constexpr static const double PI_HALF   = PI / 2;

}; // class Misc

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== OPERATIONS ===============================

template<typename T>
inline T Misc::translate(
    double value,
    double left_min,
    double left_max,
    double right_min,
    double right_max) {

    double left_range = left_max - left_min;
    double right_range = right_max - right_min;
    double value_scaled = (value - left_min) / left_range;

    return static_cast<T>(right_min + (value_scaled * right_range));
}

template <typename T>
inline int8_t Misc::sign(T val) {
    return (T(0) < val) - (val < T(0));
}

inline double Misc::toRadians(uint8_t angle) {
    return (angle * PI / 180);
}

inline double Misc::toDegrees(double rad) {
    return (rad * 180 / PI);
}

inline double Misc::delta(double angle1, double angle2) {
    return std::pow((angle1 - angle2), 2);
}

} // namespace utility

#endif  // _utility_Misc_hpp_
