// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "utility/misc.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

namespace btr
{

TEST(MiscTest, testTranslatePosition)
{
  int16_t v = 0;
  Misc::translate(-1.5, -1.5, 1.5, 0, 180, &v);
  EXPECT_EQ(0, v);
  Misc::translate(-1.0, -1.5, 1.5, 0, 180, &v);
  EXPECT_EQ(30, v);
  Misc::translate(0.0, -1.5, 1.5, 0, 180, &v);
  EXPECT_EQ(90, v);
  Misc::translate(1.0, -1.5, 1.5, 0, 180, &v);
  EXPECT_EQ(150, v);
  Misc::translate(1.5, -1.5, 1.5, 0, 180, &v);
  EXPECT_EQ(180, v);

  double d = 0.0;
  Misc::translate(0, 0, 180, -1.5, 1.5, &d);
  EXPECT_DOUBLE_EQ(-1.5, d);
  Misc::translate(30, 0, 180, -1.5, 1.5, &d);
  EXPECT_DOUBLE_EQ(-1.0, d);
  Misc::translate(90, 0, 180, -1.5, 1.5, &d);
  EXPECT_DOUBLE_EQ(0.0, d);
  Misc::translate(150, 0, 180, -1.5, 1.5, &d);
  EXPECT_DOUBLE_EQ(1.0, d);
  Misc::translate(180, 0, 180, -1.5, 1.5, &d);
  EXPECT_DOUBLE_EQ(1.5, d);
}

TEST(MiscTest, testTranslatePwm)
{
  int16_t v;
  Misc::translate(-1.0, -1.0, 0, -255, -65, &v);
  EXPECT_DOUBLE_EQ(-255, v);
  Misc::translate(-.012, -1.0, 0, -255, -65, &v);
  EXPECT_DOUBLE_EQ(-67, v);
  Misc::translate(0.012, 0, 1.0, 65, 255, &v);
  EXPECT_DOUBLE_EQ(67, v);
  Misc::translate(1.0, 0, 1.0, 65, 255, &v);
  EXPECT_DOUBLE_EQ(255, v);
}

TEST(MiscTest, testModulo)
{
  ASSERT_EQ(0, Misc::modulo(0, 3));
  ASSERT_EQ(1, Misc::modulo(1, 3));
  ASSERT_EQ(2, Misc::modulo(2, 3));
  ASSERT_EQ(0, Misc::modulo(3, 3));
  ASSERT_EQ(1, Misc::modulo(4, 3));
  ASSERT_EQ(2, Misc::modulo(5, 3));
  ASSERT_EQ(0, Misc::modulo(6, 3));
}

TEST(MiscTest, toHex)
{
  Buff buff(5);
  uint8_t input[] = {'0','1','a','b','c'};
  buff.write(input, 5);
  std::string output = TestHelpers::toHex(buff);

  ASSERT_EQ("30:31:61:62:63", output);
}

TEST(MiscTest, modBits)
{
  uint8_t v[] = { 0, 0 };

  Misc::setBit(v, 1, 0);
  ASSERT_EQ(0x2, v[0]);
  ASSERT_EQ(1, Misc::checkBit(v, 1, 0));
  Misc::clearBit(v, 1, 0);
  ASSERT_EQ(0, Misc::checkBit(v, 1, 0));

  Misc::setBit(v, 7, 0);
  ASSERT_EQ(0x80, v[0]);
  ASSERT_EQ(1, Misc::checkBit(v, 7, 0));
  Misc::clearBit(v, 7, 0);
  ASSERT_EQ(0, Misc::checkBit(v, 7, 0));

  Misc::setBit(v, 10, 0);
  ASSERT_EQ(0x4, v[1]);
  ASSERT_EQ(1, Misc::checkBit(v, 10, 0));
  Misc::clearBit(v, 10, 0);
  ASSERT_EQ(0, Misc::checkBit(v, 10, 0));

  Misc::setBit(v, 3, 1);
  Misc::setBit(v, 9, 0);
  ASSERT_EQ(0xA, v[1]);
  Misc::toggleBit(v, 3, 1);
  ASSERT_EQ(0x2, v[1]);
  Misc::toggleBit(v, 3, 1);
  ASSERT_EQ(0xA, v[1]);
}

} // namespace btr
