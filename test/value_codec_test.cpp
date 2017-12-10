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

// PROJECT INCLUDES
#include "value_codec.hpp"
#include "buff.hpp"

namespace utility {

//================================ TEST FIXTURES ===============================

class ValueCodecTest : public testing::Test {
public:

// LIFECYCLE

    ValueCodecTest() :
        buff_(32) {
    }

// ATTRIBUTES

    Buff buff_;
};

//=================================== TESTS ====================================

TEST_F(ValueCodecTest, testGetIntMsbOK) {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

    buff_.writeChunk(data);
    int64_t val64 = 0;
    int success = ValueCodec::getInt(&buff_, &val64, sizeof(int64_t), true);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-1), val64);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(data);
    int32_t val32 = 0;
    success = ValueCodec::getInt(&buff_, &val32, sizeof(int32_t), true);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-1), val32);
    ASSERT_EQ(uint32_t(4), buff_.available());

    uint8_t uval8 = 0;
    success = ValueCodec::getInt(&buff_, &uval8, sizeof(uint8_t), true);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint8_t(255), uval8);
    ASSERT_EQ(uint32_t(3), buff_.available());

    uint16_t uval16 = 0;
    success = ValueCodec::getInt(&buff_, &uval16, sizeof(uint16_t), true);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint16_t(65535), uval16);
    ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetIntLsbOK) {
    uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8};

    buff_.writeChunk(data);
    int64_t val64 = 0;
    int success = ValueCodec::getInt(&buff_, &val64, sizeof(int64_t), false);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-506097522914230529), val64);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(data);
    int32_t val32 = 0;
    success = ValueCodec::getInt(&buff_, &val32, sizeof(int32_t), false);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-50462977), val32);
    ASSERT_EQ(uint32_t(4), buff_.available());

    uint8_t uval8 = 0;
    success = ValueCodec::getInt(&buff_, &uval8, sizeof(uint8_t), false);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint8_t(251), uval8);
    ASSERT_EQ(uint32_t(3), buff_.available());

    uint16_t uval16 = 0;
    success = ValueCodec::getInt(&buff_, &uval16, sizeof(uint16_t), false);

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint16_t(63994), uval16);
    ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetIntBad) {
    uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC};

    buff_.writeChunk(data);
    int64_t val64 = 0;
    int success = ValueCodec::getInt(&buff_, &val64, sizeof(int64_t), false);
    ASSERT_EQ(ValueCodec::SMALL_BUFF, success);
    ASSERT_EQ(uint32_t(4), buff_.available());

    int16_t val16 = 0;
    success = ValueCodec::getInt(&buff_, &val16, sizeof(val16) + 1, true);
    ASSERT_EQ(ValueCodec::SMALL_VALUE, success);
    ASSERT_EQ(uint32_t(4), buff_.available());
}

TEST_F(ValueCodecTest, testLittleEndian) {
    int16_t number = 0x0001;
    char* ptr = (char*) &number;
    bool expected = (ptr[0] == 1);

    ASSERT_EQ(expected, ValueCodec::isLittleEndian());
}

TEST_F(ValueCodecTest, testModulo) {
    ASSERT_EQ(0, ValueCodec::modulo(0, 3));
    ASSERT_EQ(1, ValueCodec::modulo(1, 3));
    ASSERT_EQ(2, ValueCodec::modulo(2, 3));
    ASSERT_EQ(0, ValueCodec::modulo(3, 3));
    ASSERT_EQ(1, ValueCodec::modulo(4, 3));
    ASSERT_EQ(2, ValueCodec::modulo(5, 3));
    ASSERT_EQ(0, ValueCodec::modulo(6, 3));
}

TEST_F(ValueCodecTest, testSwap) {
    uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC};

    uint32_t val = *reinterpret_cast<uint32_t*>(data);
    ValueCodec::swap(&val);

    uint8_t* data2 = reinterpret_cast<uint8_t*>(&val);
    
    for (int i = 0; i < 4; i++)  {
        ASSERT_EQ(data[i], data2[3 - i]);
    }
}

TEST_F(ValueCodecTest, testGetIntByBits) {
    // Bits are all 1s
    //
    buff_.writeChunk(ARRAY(0xFF));
    uint8_t v1 = ValueCodec::getInt<uint8_t>(&buff_, ARRAY(0x86));
    ASSERT_EQ(uint8_t(0b111111), v1);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0xFF));
    uint8_t v2 = ValueCodec::getInt<uint8_t>(&buff_, ARRAY(0x2));
    ASSERT_EQ(uint8_t(0b11), v2);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0x1F, 0xFF));
    uint16_t v3 = ValueCodec::getInt<uint16_t>(&buff_, ARRAY(0x05, 0x88));
    ASSERT_EQ(uint16_t(0b1111111111111), v3);
    ASSERT_EQ(uint32_t(0), buff_.available());

    // Mixed bits
    //
    buff_.writeChunk(ARRAY(0xAC));
    v1 = ValueCodec::getInt<uint8_t>(&buff_, ARRAY(0x86));
    ASSERT_EQ(uint8_t(0b101011), v1);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0xAE));
    v2 = ValueCodec::getInt<uint8_t>(&buff_, ARRAY(0x2));
    ASSERT_EQ(uint8_t(0b10), v2);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0x17, 0x19));
    v3 = ValueCodec::getInt<uint16_t>(&buff_, ARRAY(0x05, 0x88));
    ASSERT_EQ(uint16_t(0b1011100011001), v3);
    ASSERT_EQ(uint32_t(0), buff_.available());

    // Second by has high bit clear. Result should be identical to the previous
    // because all 8 bits are requested.
    //
    buff_.writeChunk(ARRAY(0x17, 0x19));
    v3 = ValueCodec::getInt<uint16_t>(&buff_, ARRAY(0x05, 0x08));
    ASSERT_EQ(uint16_t(0b1011100011001), v3);
    ASSERT_EQ(uint32_t(0), buff_.available());
}

TEST_F(ValueCodecTest, testGetVarInt) {
    buff_.writeChunk(ARRAY(0x0F));
    uint8_t v1 = 0;
    int success = ValueCodec::getVarInt(&buff_, &v1);
    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint8_t(0xF), v1);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0x8F, 0x7F));
    uint16_t v2 = 0;
    success = ValueCodec::getVarInt(&buff_, &v2);
    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint16_t(0x7FF), v2);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(ARRAY(0x82, 0x88, 0x1, 0x3));
    uint16_t v3 = 0;
    success = ValueCodec::getVarInt(&buff_, &v3);
    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint16_t(0x421), v3);
    ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetVarIntBad) {
    // Buffer is small and high bit is set.
    buff_.writeChunk(ARRAY(0x82));
    uint16_t v1 = 0;
    int success = ValueCodec::getVarInt(&buff_, &v1);
    ASSERT_EQ(ValueCodec::SMALL_BUFF, success);
    ASSERT_EQ(uint32_t(0), buff_.available());

    // High bit is not set.
    buff_.writeChunk(ARRAY(0x82, 0x88, 0x81, 0x3));
    uint16_t v2 = 0;
    success = ValueCodec::getVarInt(&buff_, &v2);
    ASSERT_EQ(ValueCodec::SMALL_VALUE, success);
    ASSERT_EQ(uint32_t(1), buff_.available());
}

} // namespace utility
