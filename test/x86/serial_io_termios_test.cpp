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

using namespace std::chrono;
using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200
#define DATA_BITS 8
#define TIMEOUT 200

//------------------------------------------------------------------------------

class SerialIOTermiosTest : public testing::Test
{
public:

  // LIFECYCLE

  SerialIOTermiosTest()
    :
      tty_(),
      reader_(),
      sender_(),
      wbuff_(),
      rbuff_()
  {
    reader_.open(TTY_SIM_0, BAUD, DATA_BITS, SerialIOTermios::PARITY_NONE, TIMEOUT);
    sender_.open(TTY_SIM_1, BAUD, DATA_BITS, SerialIOTermios::PARITY_NONE, TIMEOUT);
    resetBuffers();
  }

  void resetBuffers()
  {
    uint8_t h[] =  { 'h','e','l','l','o' };

    wbuff_.reset();
    wbuff_.resize(sizeof(h));
    wbuff_.write(h, sizeof(h)/sizeof(uint8_t));

    // Don't expect to receive endline character(s)
    rbuff_.reset();
    rbuff_.resize(wbuff_.size());
  }

protected:

  // ATTRIBUTES

  PseudoTTY tty_;
  SerialIOTermios reader_;
  SerialIOTermios sender_;
  Buff wbuff_;
  Buff rbuff_;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(SerialIOTermiosTest, readWriteOK)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size()));
  TEST_MSG << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, flush)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available(), true);
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  std::this_thread::sleep_for(20ms);

  rc = reader_.available();
  ASSERT_EQ(5, rc);
  rc = reader_.flush(SerialIOTermios::FlashType::FLUSH_IN);
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
  rc = reader_.available();
  ASSERT_EQ(0, rc);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);

  resetBuffers();

  rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTermiosTest, readTimeout)
{
  high_resolution_clock::time_point start = high_resolution_clock::now();

  ssize_t rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE(TIMEOUT, duration);
  ASSERT_GT(TIMEOUT + 20, duration);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(SerialIOTermiosTest, setTimeout)
{
  uint32_t timeout = 200;
  reader_.setTimeout(timeout);
  high_resolution_clock::time_point start = high_resolution_clock::now();

  ssize_t rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE((timeout - 10), duration);
  ASSERT_GT((timeout + 10), duration);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(SerialIOTermiosTest, DISABLED_sendBreak)
{
  ssize_t rc = reader_.sendBreak(0);

  std::this_thread::sleep_for(20ms);

  rc = reader_.available();
  ASSERT_EQ(1, rc);

  uint32_t timeout = 200;
  reader_.setTimeout(timeout);
  high_resolution_clock::time_point start = high_resolution_clock::now();

  rc = reader_.recv((char*)rbuff_.write_ptr(), 1);

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE(duration, 10);
  ASSERT_GT(10, duration);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(SerialIOTermiosTest, DISABLED_WriteTimeout)
{
#if 0
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  int e = sender_.send(&large_buff);
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
#endif
}

// } Tests

} // namespace btr
