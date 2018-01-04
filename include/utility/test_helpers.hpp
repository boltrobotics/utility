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

// PROJECT INCLUDES
#include "utility/misc.hpp"

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
  virtual ~TestHelpers();

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

inline TestHelpers::~TestHelpers()
{
  PRINTF("%s\n", str().c_str());
}

//=================================== OPERATIONS ===============================

inline std::string TestHelpers::toHex(const Buff& buff)
{
  if (buff.available() > 0) {
    std::vector<char> tmp(buff.available() * 3);
    Misc::toHex(buff.read_ptr(), buff.available(), &tmp[0], tmp.size());
    return std::string(&tmp[0]);
  } else {
    return std::string();
  }
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
