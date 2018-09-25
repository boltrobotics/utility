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
#include <sstream>
#include <cmath>
#include <vector>
#include <iostream>

// PROJECT INCLUDES
#include "utility/misc.hpp"
#include "utility/buff.hpp"

namespace btr
{

class Buff;

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
   * Represent the content of buffer in hex.
   *
   * @param buff - the buffer to display
   * @param size - the buffer size
   * @return string representation
   */
  static std::string toHex(const uint8_t* buff, uint32_t size);

  /**
   * Output an array of values as a comma-delimieted string.
   *
   * @param vals - the array
   * @return string representation
   */
  static std::string toString(const uint8_t* buff, uint32_t size);

}; // class TestHelpers

#define TEST_MSG TestHelpers()

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline TestHelpers::~TestHelpers()
{
  std::cout << "\033[;32m[          ]\033[0m "
    << "\033[;33m" << str().c_str() << "\033[0m" << std::endl;
}

//============================================= OPERATIONS =========================================

inline std::string TestHelpers::toHex(const Buff& buff)
{
  return toHex(buff.read_ptr(), buff.available());
}

inline std::string TestHelpers::toHex(const uint8_t* buff, uint32_t size)
{
  if (size > 0) {
    std::vector<char> tmp(size * 3);
    Misc::toHex(buff, size, &tmp[0], tmp.size());
    return std::string(&tmp[0]);
  } else {
    return std::string();
  }
}

inline std::string TestHelpers::toString(const uint8_t* buff, uint32_t size)
{
  std::stringstream ss;
  ss << buff[0];

  for (uint32_t i = 1; i < size; i++) {
    ss << "," << buff[i];
  }
  return ss.str();
}

} // namespace btr

#endif  // _btr_TestHelpers_hpp_
