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
#include "utility/value_codec.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

namespace btr
{

//================================ TEST FIXTURES ===============================

class ValueCodecTest : public testing::Test
{
public:

  // LIFECYCLE

  ValueCodecTest()
  : buff_(32)
  {
  }

  // ATTRIBUTES

  Buff buff_;
};

//=================================== TESTS ====================================

TEST_F(ValueCodecTest, testGetFixedIntMsbOK)
{
  uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

  buff_.write(data, sizeof(data));
  int64_t val64 = 0;
  int success = ValueCodec::fixedInt(&buff_, &val64, sizeof(val64), true);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int64_t(-1), val64);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(data, sizeof(data));
  int32_t val32 = 0;
  success = ValueCodec::fixedInt(&buff_, &val32, sizeof(val32), true);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int64_t(-1), val32);
  ASSERT_EQ(uint32_t(4), buff_.available());

  uint8_t uval8 = 0;
  success = ValueCodec::fixedInt(&buff_, &uval8, sizeof(uval8), true);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint8_t(255), uval8);
  ASSERT_EQ(uint32_t(3), buff_.available());

  uint16_t uval16 = 0;
  success = ValueCodec::fixedInt(&buff_, &uval16, sizeof(uval16), true);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(65535), uval16);
  ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetFixedIntLsbOK)
{
  uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8};

  buff_.write(data, sizeof(data));
  int64_t val64 = 0;
  int success = ValueCodec::fixedInt(&buff_, &val64, sizeof(int64_t), false);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int64_t(-506097522914230529), val64);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(data, sizeof(data));
  int32_t val32 = 0;
  success = ValueCodec::fixedInt(&buff_, &val32, sizeof(int32_t), false);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int64_t(-50462977), val32);
  ASSERT_EQ(uint32_t(4), buff_.available());

  uint8_t uval8 = 0;
  success = ValueCodec::fixedInt(&buff_, &uval8, sizeof(uint8_t), false);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint8_t(251), uval8);
  ASSERT_EQ(uint32_t(3), buff_.available());

  uint16_t uval16 = 0;
  success = ValueCodec::fixedInt(&buff_, &uval16, sizeof(uint16_t), false);

  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(63994), uval16);
  ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetFixedIntBad)
{
  uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC};

  buff_.write(data, sizeof(data));
  int64_t val64 = 0;
  int success = ValueCodec::fixedInt(&buff_, &val64, sizeof(int64_t), false);
  ASSERT_EQ(ValueCodec::SMALL_BUFF, success);
  ASSERT_EQ(uint32_t(4), buff_.available());

  int16_t val16 = 0;
  success = ValueCodec::fixedInt(&buff_, &val16, sizeof(val16) + 1, true);
  ASSERT_EQ(ValueCodec::SMALL_VALUE, success);
  ASSERT_EQ(uint32_t(4), buff_.available());
}

TEST_F(ValueCodecTest, testLittleEndian)
{
  int16_t number = 0x0001;
  char* ptr = (char*) &number;
  bool expected = (ptr[0] == 1);

  ASSERT_EQ(expected, ValueCodec::isLittleEndian());
}

TEST_F(ValueCodecTest, testSwap)
{
  uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC};

  char* p = reinterpret_cast<char*>(data);
  uint32_t val = *reinterpret_cast<uint32_t*>(p);
  ValueCodec::swap(&val);

  uint8_t* data2 = reinterpret_cast<uint8_t*>(&val);

  for (int i = 0; i < 4; i++)  {
    ASSERT_EQ(data[i], data2[3 - i]) << "Index: " << i <<
      ", data2: " << TestHelpers::toHex(data2, sizeof(val));
  }
}

TEST_F(ValueCodecTest, testGetNBitVarInt)
{
  // Bits are all 1s
  //
  buff_.write(ARRAY(0xFF), 1);
  uint8_t v1 = 0;
  ValueCodec::varIntNBits(&buff_, &v1, ARRAY(0x86), 1);
  ASSERT_EQ(uint8_t(0b111111), v1);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0xFF), 1);
  uint8_t v2 = 0;
  ValueCodec::varIntNBits(&buff_, &v2, ARRAY(0x2), 1);
  ASSERT_EQ(uint8_t(0b11), v2);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0x1F, 0xFF), 2);
  uint16_t v3 = 0;
  ValueCodec::varIntNBits(&buff_, &v3, ARRAY(0x05, 0x88), 2);
  ASSERT_EQ(uint16_t(0b1111111111111), v3);
  ASSERT_EQ(uint32_t(0), buff_.available());

  // Mixed bits
  //
  buff_.write(ARRAY(0xAC), 1);
  ValueCodec::varIntNBits(&buff_, &v1, ARRAY(0x86), 1);
  ASSERT_EQ(uint8_t(0b101011), v1);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0xAE), 1);
  ValueCodec::varIntNBits(&buff_, &v2, ARRAY(0x2), 1);
  ASSERT_EQ(uint8_t(0b10), v2);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0x17, 0x19), 2);
  ValueCodec::varIntNBits(&buff_, &v3, ARRAY(0x05, 0x88), 2);
  ASSERT_EQ(uint16_t(0b1011100011001), v3);
  ASSERT_EQ(uint32_t(0), buff_.available());

  // Second by has high bit clear. Result should be identical to the previous
  // because all 8 bits are requested.
  //
  buff_.write(ARRAY(0x17, 0x19), 2);
  ValueCodec::varIntNBits(&buff_, &v3, ARRAY(0x05, 0x08), 2);
  ASSERT_EQ(uint16_t(0b1011100011001), v3);
  ASSERT_EQ(uint32_t(0), buff_.available());
}

TEST_F(ValueCodecTest, testGet7BitVarInt)
{
  buff_.write(ARRAY(0x0F), 1);
  uint8_t v1 = 0;
  int success = ValueCodec::varInt7Bits(&buff_, &v1);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint8_t(0xF), v1);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0x8F, 0x7F), 2);
  uint16_t v2 = 0;
  success = ValueCodec::varInt7Bits(&buff_, &v2);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(0x7FF), v2);
  ASSERT_EQ(uint32_t(0), buff_.available());

  buff_.write(ARRAY(0x82, 0x88, 0x1, 0x3), 4);
  uint16_t v3 = 0;
  success = ValueCodec::varInt7Bits(&buff_, &v3);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(0x421), v3);
  ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testGetVarIntBad)
{
  // Buffer is small and high bit is set.
  buff_.write(ARRAY(0x82), 1);
  uint16_t v1 = 0;
  int success = ValueCodec::varInt7Bits(&buff_, &v1);
  ASSERT_EQ(ValueCodec::SMALL_BUFF, success);
  ASSERT_EQ(uint32_t(0), buff_.available());

  // High bit is not set.
  buff_.write(ARRAY(0x82, 0x88, 0x81, 0x3), 4);
  uint16_t v2 = 0;
  success = ValueCodec::varInt7Bits(&buff_, &v2);
  ASSERT_EQ(ValueCodec::SMALL_VALUE, success);
  ASSERT_EQ(uint32_t(1), buff_.available());
}

TEST_F(ValueCodecTest, testSetFixedIntPositive)
{
  const uint16_t num = 0x0402;
  uint16_t val = 0;

  // If this host is little-endian, the number is encoded in lSB, 0x0204
  buff_.write(num);
  // Decode in MSB
  int success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(0x0204), val);

  // If this host is little-endian, the number is encoded in LSB, 0x0204
  buff_.write(num);
  // Decode in LSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), false);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);

  // If this host is little-endian, the number is encoded in MSB, 0x0402
  ValueCodec::fixedInt(&buff_, num, true);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);

  // If this host is little-endian, the number is encoded in LSB, 0x0204
  ValueCodec::fixedInt(&buff_, num, false);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(uint16_t(0x0204), val);

  // If this host is little-endian, the number is encoded in LSB, 0x0204
  ValueCodec::fixedInt(&buff_, num, false);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), false);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);
}

TEST_F(ValueCodecTest, testSetFixedIntNegative)
{
  const int16_t num = -0x0402;
  int16_t val = 0;

  // If this host is little-endian, the number is encoded in lSB, -0x0204
  buff_.write(num);
  // Decode in MSB
  int success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int16_t(-0x0105), val);

  // If this host is little-endian, the number is encoded in LSB, -0x0204
  buff_.write(num);
  // Decode in LSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), false);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);

  // If this host is little-endian, the number is encoded in MSB, -0x0402
  ValueCodec::fixedInt(&buff_, num, true);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);

  // If this host is little-endian, the number is encoded in LSB, -0x0204
  ValueCodec::fixedInt(&buff_, num, false);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(int16_t(-0x0105), val);

  // If this host is little-endian, the number is encoded in LSB, -0x0204
  ValueCodec::fixedInt(&buff_, num, false);
  // Decode in MSB
  success = ValueCodec::fixedInt(&buff_, &val, sizeof(val), false);
  ASSERT_EQ(ValueCodec::SUCCESS, success);
  ASSERT_EQ(num, val);
}

template<typename OutType>
void decodeModf(Buff* buff, OutType ipart_expected, OutType fpart_expected, int line)
{
  // Decode integer part
  OutType ipart_actual = 0;
  int success = ValueCodec::fixedInt(buff, &ipart_actual, sizeof(OutType), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success) << "Line: " << line;
  ASSERT_EQ(ipart_expected, ipart_actual) << "Line: " << line;

  // Decode fractional part
  OutType fpart_actual = 0;
  success = ValueCodec::fixedInt(buff, &fpart_actual, sizeof(OutType), true);
  ASSERT_EQ(ValueCodec::SUCCESS, success) << "Line: " << line;
  ASSERT_EQ(fpart_expected, fpart_actual) << "Line: " << line;
}

TEST_F(ValueCodecTest, testEncodeModf)
{
  double val = Misc::PI;

  ValueCodec::encodeModf<double, uint8_t>(&buff_, val, 2, true);
  decodeModf<uint8_t>(&buff_, 3, 14, __LINE__);

  ValueCodec::encodeModf<double, uint8_t>(&buff_, val, 1, true);
  decodeModf<uint8_t>(&buff_, 3, 1, __LINE__);

  ValueCodec::encodeModf<double, uint8_t>(&buff_, val, 0, true);
  decodeModf<uint8_t>(&buff_, 3, 0, __LINE__);

  val = 65535.65536;
  ValueCodec::encodeModf<double, uint8_t>(&buff_, val, 5, true);
  decodeModf<uint8_t>(&buff_, 255, 0, __LINE__);

  val = 65536.65535;
  ValueCodec::encodeModf<double, uint16_t>(&buff_, val, 5, true);
  decodeModf<uint16_t>(&buff_, 0, 65535, __LINE__);
}

} // namespace btr
