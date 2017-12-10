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

namespace utility {

static const char lut[] = "0123456789ABCDEF";

//------------------------------------------------------------------------------

class BuffTest : public testing::Test {
public:

// LIFECYCLE

    BuffTest() :
        buff_(1) {
    }

// OPERATIONS

    std::string toHex() {
        std::string hex_buff("     Hex: ");
        uint32_t size = buff_.size();

        for (uint32_t i = 0; i < size; i++) {
            const uint8_t c = buff_.data()[i];
            hex_buff.push_back(lut[c >> 4]);
            hex_buff.push_back(lut[c & 15]);
            hex_buff.push_back(' ');
        }
        return hex_buff;
    }

    template<typename T, uint32_t N>
    std::string toString(T (&vals)[N]) {
        std::stringstream ss;

        for (uint32_t i = 0; i < N; i++) {
            ss << vals[i] << ' ';
        }

        return ss.str();
    }

// ATTRIBUTES

    utility::Buff buff_;
};

//------------------------------------------------------------------------------

TEST_F(BuffTest, resize) {

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

TEST_F(BuffTest, extend) {

    ASSERT_EQ(uint32_t(1), buff_.size());
    ASSERT_EQ(uint32_t(0), buff_.available());
    ASSERT_EQ(uint32_t(1), buff_.remaining());

    // Extend so as to have 3 bytes remaining for writing to.
    buff_.extend(3);

    ASSERT_EQ(uint32_t(3), buff_.size());
    ASSERT_EQ(uint32_t(0), buff_.available());
    ASSERT_EQ(uint32_t(3), buff_.remaining());

    bool success = buff_.extend(10000);
    buff_.write_ptr() += 3;

    ASSERT_EQ(true, success);
    ASSERT_EQ(uint32_t(10000), buff_.size());
    ASSERT_EQ(uint32_t(3), buff_.available());
    ASSERT_EQ(uint32_t(10000 - 3), buff_.remaining());

    buff_.reset();
    ASSERT_EQ(uint32_t(10000), buff_.size());
    ASSERT_EQ(uint32_t(0), buff_.available());
    ASSERT_EQ(uint32_t(10000), buff_.remaining());
}

TEST_F(BuffTest, readWriteSingle) {
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

TEST_F(BuffTest, readWriteChunk) {
    uint8_t input1[] = { 1, 2, 3 };
    buff_.writeChunk(input1);
    uint8_t input2[] = { 4, 5, 6 };
    buff_.writeChunk(input2);

    ASSERT_EQ(uint32_t(6), buff_.size());
    ASSERT_EQ(uint32_t(6), buff_.available());
    ASSERT_EQ(uint32_t(0), buff_.remaining());

    uint8_t output[3] = { 0 };
    ASSERT_EQ(true, buff_.readChunk(output));

    for (uint8_t i = 0; i < sizeof(output); i++) {
        ASSERT_EQ(input1[i], output[i]);
    }
    ASSERT_EQ(uint32_t(6), buff_.size());
    ASSERT_EQ(uint32_t(3), buff_.available());
    ASSERT_EQ(uint32_t(0), buff_.remaining());

    ASSERT_EQ(true, buff_.readChunk(output));

    for (uint8_t i = 0; i < sizeof(output); i++) {
        ASSERT_EQ(input2[i], output[i]);
    }
    ASSERT_EQ(uint32_t(6), buff_.size());
    ASSERT_EQ(uint32_t(0), buff_.available());
    ASSERT_EQ(uint32_t(0), buff_.remaining());
}

TEST_F(BuffTest, shiftOnWrite) {

    uint8_t chunk[] = { '\n', '\n' };
    bool success = buff_.writeChunk(chunk, false);

    ASSERT_NE(true, success);
    ASSERT_EQ(uint32_t(1), buff_.remaining());

    success = buff_.extend(3);
    success &= buff_.writeChunk(chunk, false);

    ASSERT_EQ(true, success);
    ASSERT_EQ(uint32_t(1), buff_.remaining());

    buff_.read_ptr() += 2;

    ASSERT_EQ(uint32_t(2), buff_.consumed());
    ASSERT_EQ(uint32_t(0), buff_.available());
    ASSERT_EQ(uint32_t(1), buff_.remaining());
    ASSERT_EQ(uint32_t(3), buff_.size());

    const uint8_t* data = buff_.data();
    success = buff_.writeChunk(chunk, false);

    ASSERT_EQ(true, success);
    ASSERT_EQ(uint32_t(0), buff_.consumed());
    ASSERT_EQ(uint32_t(2), buff_.available());
    ASSERT_EQ(uint32_t(1), buff_.remaining());
    ASSERT_EQ(uint32_t(3), buff_.size());
    ASSERT_TRUE(data == buff_.data());

    buff_.read_ptr() += 2;

    data = buff_.data();
    uint8_t chunk2[] = { '\n', '\n', '\n', '\n' };
    success = buff_.writeChunk(chunk2, true);

    ASSERT_EQ(true, success);
    ASSERT_EQ(uint32_t(0), buff_.consumed());
    ASSERT_EQ(uint32_t(4), buff_.available());
    ASSERT_EQ(uint32_t(0), buff_.remaining());
    ASSERT_EQ(uint32_t(4), buff_.size());
    ASSERT_TRUE(data == buff_.data());
}

} // namespace utility
