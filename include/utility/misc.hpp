/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_Misc_hpp_
#define _btr_Misc_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>
#include <cmath>
#include <cstring>

// PROJECT INCLUDES

namespace btr
{

/**
 * Implement miscellaneous functions.
 */
class Misc
{
public:

// ATTRIBUTES

  constexpr static const double PI        = 3.14159;
  constexpr static const double PI_HALF   = PI / 2;
  constexpr static const double PI_TWO    = PI * 2;

// LIFECYCLE

  /**
   * This class uses only static operations.
   */
  Misc() = delete;
  ~Misc() = delete;

// OPERATIONS

  /**
   * Translate a value from one range to the value in another.
   *
   * @param val - the position to translate
   * @param left_min - minimum value of input range
   * @param left_max - maximum value of input range
   * @param right_min - minimum value of output range
   * @param right_max - maximum value of output range
   * @param output - the translated value
   */
  template<typename T>
  static void translate(
    double value, double left_min, double left_max, double right_min, double right_max, T* output);

  /**
   * Get a sign of the provided value.
   *
   * @param val - the value to get the sign of
   * @return - 1 if the value is positive, -1 otherwise
   */
  template <typename T>
  static int8_t sign(T val);

  /**
   * Calculate angle delta squared.
   *
   * @param angle1
   * @param angle2
   */
  static double delta(double angle1, double angle2);

  /**
   * Modulo operator to handle negative numbers. The % operator in C is not the
   * modulo but the remainder operator.
   *
   * @param a - left parameter
   * @param b - right parameter
   */
  template<typename T, typename U>
  static T modulo(T a, U b);

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
   * Represent the content of buffer in hex.
   *
   * @param data - raw data
   * @param size - the size of raw data
   * @param dst_str - the destination buffer
   * @param dst_size - the size of dst_str
   * @return 0 if converted ok, -1 if strlen(dst_str) is less than size * 3
   */
  static int toHex(const uint8_t* data, uint32_t size, char* dst_str, uint32_t dst_size);

  /**
   * Shift the decimal point in a floating-point number by the specified number of places
   * to the right.
   *
   * @param input - input variable
   * @param decimal_places - the number of places to shift by
   * @return the resulting integer
   */
  template<typename InType, typename OutType>
  static void shiftfint(InType input, OutType* output, uint8_t decimal_places);

  /**
   * Break input value into integer and fractional parts. Multiply the fractional
   * part by the supplied number. After, cast the resulting values into target types.
   *
   * @param input - input value
   * @param ipart - integer part of the result
   * @param fpart - fractional part of the result
   * @param decimal_places - the number of decimal places in fractional part
   */
  template<typename InType, typename OutType>
  static void modfint(InType input, OutType* ipart, OutType* fpart, uint8_t decimal_places);

}; // class Misc

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
inline void Misc::translate(
    double value,
    double left_min,
    double left_max,
    double right_min,
    double right_max,
    T* output)
{
  double left_range = left_max - left_min;
  double right_range = right_max - right_min;
  double value_scaled = (value - left_min) / left_range;
  *output = static_cast<T>(right_min + (value_scaled * right_range));
}

template <typename T>
inline int8_t Misc::sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

inline double Misc::delta(double angle1, double angle2)
{
  return std::pow((angle1 - angle2), 2);
}

template<typename T, typename U>
inline T Misc::modulo(T a, U b)
{
  T r = a % b;
  return (r < 0 ? r + b : r);
}

inline double Misc::toRadians(uint8_t angle)
{
  return (angle * PI / 180);
}

inline double Misc::toDegrees(double rad)
{
  return (rad * 180 / PI);
}

inline int Misc::toHex(const uint8_t* data, uint32_t size, char* dst_str, uint32_t dst_size)
{
  if (size == 0 || dst_size < (size * 3)) {
    return -1;
  }

  static const char lut[] = "0123456789ABCDEF";

  for (uint32_t i = 0, j = 0; i < size; i++, j += 3) {
    const uint8_t c = data[i];
    dst_str[j] = lut[c >> 4];
    dst_str[j + 1] = lut[c & 15];
    dst_str[j + 2] = ':';
  }

  dst_str[dst_size - 1] = '\0';
  return 0;
}

#if 0
template<typename T, uint32_t N>
inline std::string Misc::toString(T (&vals)[N])
{
  std::stringstream ss;
  ss << vals[0];

  for (uint32_t i = 1; i < N; i++) {
    ss << "," << vals[i];
  }
  return ss.str();
}
#endif

template<typename InType, typename OutType>
inline void Misc::shiftfint(InType input, OutType* output, uint8_t dec_places)
{
  *output = round(input * pow(10, dec_places));
}

template<typename InType, typename OutType>
inline void Misc::modfint(InType input, OutType* ipart, OutType* fpart, uint8_t dec_places)
{
  double ipart_tmp = 0;
  double fpart_tmp = modf(input, &ipart_tmp);

  *ipart = static_cast<OutType>(ipart_tmp);
  shiftfint(fpart_tmp, fpart, dec_places);
}

} // namespace btr

#endif  // _btr_Misc_hpp_
