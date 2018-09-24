/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
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

// SYSTEM INCLUDES
#include <cmath>
#include <cstring>

// PROJECT INCLUDES
#include "utility/misc.hpp"  // class implemented

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

double Misc::translate(
  double value, double left_min, double left_max, double right_min, double right_max)
{
  return translateImpl<double>(value, left_min, left_max, right_min, right_max);
}

int8_t Misc::sign(int16_t val)
{
  return signImpl(val);
}

double Misc::toRadians(uint8_t angle)
{
  return (angle * PI / 180);
}

double Misc::toDegrees(double rad)
{
  return (rad * 180 / PI);
}

double Misc::delta(double angle1, double angle2)
{
  return std::pow((angle1 - angle2), 2);
}

int16_t Misc::modulo(int16_t a, int16_t b)
{
  return moduloImpl(a, b);
}

int Misc::toHex(const uint8_t* data, uint32_t size, char* dst_str, uint32_t dst_size)
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

void Misc::shiftfint(double input, uint16_t* output, uint8_t dec_places)
{
  shiftfintImpl(input, output, dec_places);
}

void Misc::modfint(double input, uint8_t* ipart, uint8_t* fpart, uint8_t decimal_places)
{
  modfintImpl(input, ipart, fpart, decimal_places);
}

void Misc::modfint(double input, uint16_t* ipart, uint16_t* fpart, uint8_t decimal_places)
{
  modfintImpl(input, ipart, fpart, decimal_places);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
T Misc::translateImpl(
    double value,
    double left_min,
    double left_max,
    double right_min,
    double right_max)
{
  double left_range = left_max - left_min;
  double right_range = right_max - right_min;
  double value_scaled = (value - left_min) / left_range;
  return static_cast<T>(right_min + (value_scaled * right_range));
}

template <typename T>
int8_t Misc::signImpl(T val)
{
  return (T(0) < val) - (val < T(0));
}

template<typename T, typename U>
T Misc::moduloImpl(T a, U b)
{
  T r = a % b;
  return (r < 0 ? r + b : r);
}

template<typename InType, typename OutType>
void Misc::shiftfintImpl(InType input, OutType* output, uint8_t dec_places)
{
  *output = round(input * pow(10, dec_places));
}

template<typename InType, typename OutType>
void Misc::modfintImpl(InType input, OutType* ipart, OutType* fpart, uint8_t dec_places)
{
  double ipart_tmp = 0;
  double fpart_tmp = modf(input, &ipart_tmp);

  *ipart = static_cast<OutType>(ipart_tmp);
  shiftfintImpl(fpart_tmp, fpart, dec_places);
}

} // namespace btr
