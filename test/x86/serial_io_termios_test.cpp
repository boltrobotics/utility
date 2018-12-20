// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL v3

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

// PROJECT INCLUDES
#include "utility/x86/serial_io_termios.hpp"
#include "utility/x86/pseudo_tty.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200
#define DATA_BITS 8
#define TIMEOUT 100

//------------------------------------------------------------------------------

class SerialIOTermiosTest : public testing::Test
{
public:

  // LIFECYCLE

  SerialIOTermiosTest()
    :
      tty_(),
      act_serial_(TTY_SIM_0, BAUD, DATA_BITS, SerialIOTermios::PARITY_NONE, TIMEOUT),
      sim_serial_(TTY_SIM_1, BAUD, DATA_BITS, SerialIOTermios::PARITY_NONE, TIMEOUT),
      wbuff_(),
      rbuff_()
  {
    resetBuffers();
  }

  void resetBuffers()
  {
    uint8_t h[] =  { 'h','e','l','l','o' };
    wbuff_.resize(sizeof(h));
    wbuff_.write(h, sizeof(h)/sizeof(uint8_t));
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
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(SerialIOTermiosTest, ReadWriteOK)
{
  errno = 0;
  int e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(5, e) << " Message: " << strerror(errno);

  errno = 0;
  std::this_thread::sleep_for(10ms);
  e = act_serial_.recv(&rbuff_, rbuff_.remaining());

  ASSERT_EQ(5, e) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size()));
  TEST_MSG << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, Flush)
{
  errno = 0;
  int e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(5, e) << " Message: " << strerror(errno);

  e = sim_serial_.flush(SerialIOTermios::FlashType::FLUSH_INOUT);
  ASSERT_EQ(0, e) << " Message: " << strerror(errno);

  std::this_thread::sleep_for(10ms);

  e = act_serial_.recv(&rbuff_, rbuff_.remaining());
  ASSERT_EQ(0, e) << " Message: " << strerror(errno);

  ASSERT_EQ(0, rbuff_.available());

  resetBuffers();

  e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(5, e) << " Message: " << strerror(errno);

  std::this_thread::sleep_for(10ms);
  e = act_serial_.recv(&rbuff_, rbuff_.remaining());
  ASSERT_EQ(5, e) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, setTimeout)
{
  FAIL();
}

TEST_F(SerialIOTermiosTest, ReadTimeout)
{
  int e = act_serial_.recv(&rbuff_, rbuff_.remaining());
  // Timed out
  ASSERT_EQ(0, e) << " Message: " << strerror(errno);
  ASSERT_EQ(0, rbuff_.available());
}

TEST_F(SerialIOTermiosTest, DISABLED_WriteTimeout)
{
#if 0
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  int e = sim_serial_.send(&large_buff);
  ASSERT_EQ(0, e) << " Message: " << strerror(errno);
#endif
}

TEST_F(SerialIOTermiosTest, NoBufferSpace)
{
  rbuff_.write_ptr() += rbuff_.remaining();
  ASSERT_EQ(rbuff_.size(), rbuff_.available());
  ASSERT_EQ(0, rbuff_.remaining());

  int e = act_serial_.recv(&rbuff_, rbuff_.remaining() + 1);
  ASSERT_EQ(-1, e) << " Message: " << strerror(errno);
  ASSERT_EQ(ENOBUFS, errno) << " Message: " << strerror(errno);
}

// } Tests

} // namespace btr
