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

#ifndef _btr_TestHelpers_hpp_
#define _btr_TestHelpers_hpp_

// SYSTEM INCLUDES
#include <cmath>
#include <string>

namespace testing
{
  namespace internal
  {
    enum GTestColor {
      COLOR_DEFAULT,
      COLOR_RED,
      COLOR_GREEN,
      COLOR_YELLOW
    };

    extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
  }
}

#define PRINTF(...)  do { \
  testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); \
  testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); \
} while(0)

namespace btr
{

/**
 * Implement commonly-used functions for unit-testing.
 */
class TestHelpers : public std::stringstream
{
public:

  // LIFECYCLE

  /**
   * Output a string accumulated via stringstream super class.
   */
  ~TestHelpers();

  // OPERATIONS

  /**
   * Represent the content of buffer in hex.
   *
   * @param buff - the buffer to display
   * @return string representation
   */
  static std::string toHex(const Buff& buff);

  /**
   * Output an array of values as a comma-delimieted string.
   *
   * @param vals - the array
   * @return string representation
   */
  template<typename T, uint32_t N>
  static std::string toString(T (&vals)[N]);

}; // class TestHelpers

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

TestHelpers::~TestHelpers()
{
  PRINTF("%s", str().c_str());
}

//=================================== OPERATIONS ===============================

inline std::string TestHelpers::toHex(const Buff& buff)
{
  static const char lut[] = "0123456789ABCDEF";

  std::string hex_buff("Hex: ");
  uint32_t size = buff.size();

  for (uint32_t i = 0; i < size; i++) {
    const uint8_t c = buff.data()[i];
    hex_buff.push_back(lut[c >> 4]);
    hex_buff.push_back(lut[c & 15]);
    hex_buff.push_back(' ');
  }
  return hex_buff;
}

template<typename T, uint32_t N>
inline std::string TestHelpers::toString(T (&vals)[N])
{
  std::stringstream ss;
  ss << vals[0];

  for (uint32_t i = 1; i < N; i++) {
    ss << "," << vals[i];
  }
  return ss.str();
}

#define TEST_MSG TestHelpers()

} // namespace btr

#endif  // _btr_TestHelpers_hpp_
