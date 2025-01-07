// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

// PROJECT INCLUDES
#include "utility/common/defines.hpp"
#include "utility/common/buff.hpp"
#include "utility/common/test_helpers.hpp"
#include "utility/x86/usart_termios.hpp"
#include "utility/x86/pseudo_tty.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace btr
{

#define BAUD 115200
#define DATA_BITS 8

//------------------------------------------------------------------------------

class UsartTermiosTest : public testing::Test
{
public:

  // LIFECYCLE

  UsartTermiosTest()
    :
      tty_(),
      reader_(),
      sender_(),
      wbuff_(),
      rbuff_()
  {
    // On occasion, readWriteOK test would get bad file descriptor. One reason could be because
    // of unfinished port set up in tty_ constructor. Add a bit of sleep for now.
    std::this_thread::sleep_for(20ms);

    reader_.configure(TTY_SIM_0, BAUD, DATA_BITS, ParityType::NONE, BTR_USART_IO_TIMEOUT_MS);
    sender_.configure(TTY_SIM_1, BAUD, DATA_BITS, ParityType::NONE, BTR_USART_IO_TIMEOUT_MS);
    reader_.open();
    sender_.open();
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
  UsartTermios reader_;
  UsartTermios sender_;
  Buff wbuff_;
  Buff rbuff_;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(UsartTermiosTest, readWriteOK)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available());
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size()));
  TEST_MSG << TestHelpers::toHex(rbuff_);
}

TEST_F(UsartTermiosTest, flush)
{
  ssize_t rc = sender_.send((char*)wbuff_.read_ptr(), wbuff_.available(), true);
  ASSERT_EQ(5, rc) << " Message: " << strerror(errno);

  std::this_thread::sleep_for(20ms);

  rc = reader_.available();
  ASSERT_EQ(5, rc);
  rc = reader_.flush(DirectionType::IN);
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

TEST_F(UsartTermiosTest, readTimeout)
{
  high_resolution_clock::time_point start = high_resolution_clock::now();

  ssize_t rc = reader_.recv((char*)rbuff_.write_ptr(), rbuff_.remaining());

  high_resolution_clock::time_point now = high_resolution_clock::now();
  auto elapsed = duration_cast<milliseconds>(now - start).count();

  ASSERT_LE(BTR_USART_IO_TIMEOUT_MS, elapsed);
  ASSERT_GT(BTR_USART_IO_TIMEOUT_MS + 20, elapsed);

  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}

TEST_F(UsartTermiosTest, setTimeout)
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

#if 0
TEST_F(UsartTermiosTest, DISABLED_sendBreak)
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
#endif

#if 0
TEST_F(UsartTermiosTest, DISABLED_WriteTimeout)
{
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  int e = sender_.send(&large_buff);
  ASSERT_EQ(0, rc) << " Message: " << strerror(errno);
}
#endif

// } Tests

} // namespace btr
