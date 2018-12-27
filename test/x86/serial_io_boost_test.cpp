// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL v3

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <chrono>

// PROJECT INCLUDES
#include "utility/x86/serial_io_boost.hpp"
#include "utility/x86/pseudo_tty.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200
#define DATA_BITS 8
#define TIMEOUT 100

//------------------------------------------------------------------------------

class SerialIOBoostTest : public testing::Test
{
public:

  // LIFECYCLE

  SerialIOBoostTest()
    :
      tty_(),
      reader_(),
      sender_(),
      wbuff_(),
      rbuff_(),
      success_(),
      timeout_(std::make_error_code(std::errc::operation_canceled))
  {
    reader_.open(TTY_SIM_0, BAUD, DATA_BITS, SerialIOBoost::PARITY_NONE, TIMEOUT);
    sender_.open(TTY_SIM_1, BAUD, DATA_BITS, SerialIOBoost::PARITY_NONE, TIMEOUT);
    resetBuffers();
  }

  void resetBuffers()
  {
    uint8_t h[] =  { 'h','e','l','l','o' };
    wbuff_.resize(sizeof(h));
    wbuff_.write(h, sizeof(h) / sizeof(uint8_t));
    // Don't expect to receive endline character(s)
    rbuff_.resize(wbuff_.size());
  }

protected:

  // ATTRIBUTES

  PseudoTTY tty_;
  SerialIOBoost reader_;
  SerialIOBoost sender_;
  Buff wbuff_;
  Buff rbuff_;
  std::error_code success_;
  std::error_code timeout_;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(SerialIOBoostTest, ReadWriteOK)
{
  std::error_code e = sender_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  e = reader_.recv(&rbuff_);

  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOBoostTest, DISABLED_Flush)
{
  std::error_code e = sender_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  e = sender_.flush();
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  // FIXME: Flush test: serial_.recv generates an error
  e = reader_.recv(&rbuff_);
  ASSERT_EQ(timeout_, e) << " Message: " << e.message();

  resetBuffers();

  e = sender_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  e = reader_.recv(&rbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOBoostTest, ReadTimeout)
{
  std::error_code e = reader_.recv(&rbuff_);
  ASSERT_EQ(timeout_, e) << " Message: " << e.message();
}

TEST_F(SerialIOBoostTest, setTimeout)
{
  uint32_t timeout = 2;
  reader_.setTimeout(timeout);
  high_resolution_clock::time_point start = high_resolution_clock::now();

  std::error_code e = reader_.recv(&rbuff_);

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE((timeout - 2), duration);
  ASSERT_GT((timeout + 2), duration);

  ASSERT_EQ(timeout_, e) << " Message: " << e.message();
  ASSERT_EQ(0, rbuff_.available());
}

TEST_F(SerialIOBoostTest, DISABLED_WriteTimeout)
{
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  std::error_code e = sender_.send(&large_buff);
  ASSERT_EQ(timeout_, e) << " Message: " << e.message();
}

// } Tests

} // namespace btr
