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
#include <iostream>
#include <sstream>
#include <limits>

// PROJECT INCLUDES
#include "buff.hpp"

namespace btr
{

//------------------------------------------------------------------------------

class BuffTest : public testing::Test
{
public:

  // LIFECYCLE

  BuffTest()
  : buff_(1)
  {
  }

  // ATTRIBUTES

  btr::Buff buff_;
};

//------------------------------------------------------------------------------

TEST_F(BuffTest, resize)
{
  ASSERT_EQ(uint32_t(1), buff_.size());
  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());

  bool success = buff_.resize(5);
  buff_.write_ptr() += 3;

  ASSERT_EQ(true, success);
  ASSERT_EQ(uint32_t(5), buff_.size());
  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(2), buff_.remaining());

  const uint8_t* data = buff_.data();
  success = buff_.resize(4097);

  ASSERT_EQ(true, success);
  ASSERT_TRUE(buff_.data() != NULL);
  ASSERT_TRUE(buff_.data() != data);
  ASSERT_EQ(uint32_t(4097), buff_.size());
  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(4097 - 3), buff_.remaining());

  data = buff_.data();
  success = buff_.resize(4);

  ASSERT_EQ(true, success);
  ASSERT_TRUE(buff_.data() != NULL);
  ASSERT_TRUE(buff_.data() == data);
  ASSERT_EQ(uint32_t(4), buff_.size());
  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(4 - 3), buff_.remaining());
}

TEST_F(BuffTest, extend)
{
  ASSERT_EQ(uint32_t(1), buff_.size());
  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());

  // Extend so as to have 3 bytes remaining for writing to.
  buff_.extend(3);

  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(4), buff_.remaining());
  ASSERT_EQ(uint32_t(4), buff_.size());

  bool success = buff_.extend(10000);
  buff_.write_ptr() += 4;

  ASSERT_EQ(true, success);
  ASSERT_EQ(uint32_t(4), buff_.available());
  ASSERT_EQ(uint32_t(10000), buff_.remaining());
  ASSERT_EQ(uint32_t(10000 + 4), buff_.size());

  buff_.reset();
  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(10004), buff_.remaining());
  ASSERT_EQ(uint32_t(10004), buff_.size());
}

TEST_F(BuffTest, readWriteSingle)
{
  buff_.write(uint8_t(1));
  buff_.write(uint8_t(2));
  buff_.write(uint8_t(3));
  ASSERT_EQ(uint32_t(3), buff_.size());
  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(0), buff_.remaining());

  uint8_t output[3] = { 0 };
  ASSERT_EQ(true, buff_.read(&output[0]));
  ASSERT_EQ(true, buff_.read(&output[1]));
  ASSERT_EQ(true, buff_.read(&output[2]));
  ASSERT_EQ(1, output[0]);
  ASSERT_EQ(2, output[1]);
  ASSERT_EQ(3, output[2]);
  ASSERT_EQ(uint32_t(3), buff_.size());
  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(0), buff_.remaining());
}

TEST_F(BuffTest, readWriteChunk)
{
  uint8_t input1[] = { 1, 2, 3 };
  buff_.writeChunk(input1);
  uint8_t input2[] = { 4, 5, 6 };
  buff_.writeChunk(input2);

  ASSERT_EQ(uint32_t(6), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());
  ASSERT_EQ(uint32_t(7), buff_.size());

  uint8_t output[3] = { 0 };
  ASSERT_EQ(true, buff_.readChunk(output));

  for (uint8_t i = 0; i < sizeof(output); i++) {
    ASSERT_EQ(input1[i], output[i]);
  }

  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());
  ASSERT_EQ(uint32_t(7), buff_.size());

  ASSERT_EQ(true, buff_.readChunk(output));

  for (uint8_t i = 0; i < sizeof(output); i++) {
    ASSERT_EQ(input2[i], output[i]);
  }

  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());
  ASSERT_EQ(uint32_t(7), buff_.size());
}

TEST_F(BuffTest, shiftOnWrite)
{
  const uint8_t chunk[] = { '2', '2' };
  bool success = buff_.writeChunk(chunk, false);

  // It wasn't successful because it couldn't extend the buffer
  ASSERT_NE(true, success);
  ASSERT_EQ(uint32_t(1), buff_.remaining());

  success = buff_.extend(3);
  success &= buff_.writeChunk(chunk, false);

  ASSERT_EQ(true, success);
  ASSERT_EQ(uint32_t(2), buff_.remaining());

  buff_.read_ptr() += 2;

  ASSERT_EQ(uint32_t(2), buff_.consumed());
  ASSERT_EQ(uint32_t(0), buff_.available());
  ASSERT_EQ(uint32_t(2), buff_.remaining());
  ASSERT_EQ(uint32_t(4), buff_.size());

  // Store the pointer to later verify that the memory wasn't moved
  const uint8_t* data = buff_.data();
  const uint8_t chunk3[] = { '3', '3', '3' };
  // During this call, already consumed data will be shifted (discarded)
  success = buff_.writeChunk(chunk3, false);

  ASSERT_EQ(true, success);
  ASSERT_EQ(uint32_t(0), buff_.consumed());
  ASSERT_EQ(uint32_t(3), buff_.available());
  ASSERT_EQ(uint32_t(1), buff_.remaining());
  ASSERT_EQ(uint32_t(4), buff_.size());
  ASSERT_TRUE(data == buff_.data());

  buff_.read_ptr() += 2;
  ASSERT_EQ(uint32_t(2), buff_.consumed());
  ASSERT_EQ(uint32_t(1), buff_.available());

  data = buff_.data();
  uint8_t chunk4[] = { '4', '4', '4', '4' };
  success = buff_.writeChunk(chunk4, true);

  ASSERT_EQ(true, success);
  ASSERT_EQ(uint32_t(0), buff_.consumed());
  ASSERT_EQ(uint32_t(5), buff_.available()); // '3' (1 item) + '4' (4 items)
  ASSERT_EQ(uint32_t(3), buff_.remaining());
  ASSERT_EQ(uint32_t(8), buff_.size());
  ASSERT_TRUE(data == buff_.data());
}

} // namespace btr
