/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

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
    int success = ValueCodec::getInt(&buff_, &val64, sizeof(int64_t));

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-1), val64);
    ASSERT_EQ(uint32_t(0), buff_.available());

    buff_.writeChunk(data);
    int32_t val32 = 0;
    success = ValueCodec::getInt(&buff_, &val32, sizeof(int32_t));

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(int64_t(-1), val32);
    ASSERT_EQ(uint32_t(4), buff_.available());

    uint8_t uval8 = 0;
    success = ValueCodec::getInt(&buff_, &uval8, sizeof(uint8_t));

    ASSERT_EQ(ValueCodec::SUCCESS, success);
    ASSERT_EQ(uint8_t(255), uval8);
    ASSERT_EQ(uint32_t(3), buff_.available());

    uint16_t uval16 = 0;
    success = ValueCodec::getInt(&buff_, &uval16, sizeof(uint16_t));

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

} // namespace utility
