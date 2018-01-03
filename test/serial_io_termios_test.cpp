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

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

// PROJECT INCLUDES
#include "utility/serial_io_termios.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"
#include "utility/pseudo_tty.hpp"

using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200

//------------------------------------------------------------------------------

class SerialIOTermiosTest : public testing::Test
{
public:

  // LIFECYCLE

  SerialIOTermiosTest()
  : tty_(),
    act_serial_(TTY_SIM_0, BAUD, 100),
    sim_serial_(TTY_SIM_1, BAUD, 100),
    wbuff_(),
    rbuff_(),
    success_()
  {
    resetBuffers();
  }

  void resetBuffers()
  {
    uint8_t h[] =  { 'h','e','l','l','o' };
    wbuff_.resize(sizeof(h));
    wbuff_.writeChunk(h);
    // Don't expect to receive endline character(s)
    rbuff_.resize(wbuff_.size());
  }

protected:

  // ATTRIBUTES

  PseudoTTY tty_;
  SerialIOTermios act_serial_;
  SerialIOTermios sim_serial_;
  Buff wbuff_;
  Buff rbuff_;
  std::error_code success_;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(SerialIOTermiosTest, ReadWriteOK)
{
  std::error_code e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  std::this_thread::sleep_for(10ms);
  e = act_serial_.recv(&rbuff_);

  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, Flush)
{
  std::error_code e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  e = sim_serial_.flush();
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  std::this_thread::sleep_for(10ms);
  e = act_serial_.recv(&rbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, rbuff_.available());

  resetBuffers();

  e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  std::this_thread::sleep_for(10ms);
  e = act_serial_.recv(&rbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, ReadTimeout)
{
  std::error_code e = act_serial_.recv(&rbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, rbuff_.available());
}

TEST_F(SerialIOTermiosTest, DISABLED_WriteTimeout)
{
#if 0
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  std::error_code e = sim_serial_.send(&large_buff);
  ASSERT_EQ(success_, e) << " Message: " << e.message();
#endif
}

// } Tests

} // namespace btr
