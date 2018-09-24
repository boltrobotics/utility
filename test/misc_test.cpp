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
#include "utility/misc.hpp"
#include "utility/buff.hpp"
#include "utility/test_helpers.hpp"

namespace btr
{

TEST(MiscTest, testTranslatePosition)
{
  EXPECT_EQ(uint8_t(0), Misc::translate<int16_t>(-1.5, -1.5, 1.5, 0, 180));
  EXPECT_EQ(uint8_t(30), Misc::translate<int16_t>(-1.0, -1.5, 1.5, 0, 180));
  EXPECT_EQ(uint8_t(90), Misc::translate<int16_t>(0.0, -1.5, 1.5, 0, 180));
  EXPECT_EQ(uint8_t(150), Misc::translate<int16_t>(1.0, -1.5, 1.5, 0, 180));
  EXPECT_EQ(uint8_t(180), Misc::translate<int16_t>(1.5, -1.5, 1.5, 0, 180));

  EXPECT_DOUBLE_EQ(-1.5, Misc::translate<double>(0, 0, 180, -1.5, 1.5));
  EXPECT_DOUBLE_EQ(-1.0, Misc::translate<double>(30, 0, 180, -1.5, 1.5));
  EXPECT_DOUBLE_EQ(0.0, Misc::translate<double>(90, 0, 180, -1.5, 1.5));
  EXPECT_DOUBLE_EQ(1.0, Misc::translate<double>(150, 0, 180, -1.5, 1.5));
  EXPECT_DOUBLE_EQ(1.5, Misc::translate<double>(180, 0, 180, -1.5, 1.5));
}

TEST(MiscTest, testTranslatePwm)
{
  EXPECT_DOUBLE_EQ(-255, Misc::translate<int16_t>(-1.0, -1.0, 0, -255, -65));
  EXPECT_DOUBLE_EQ(-67, Misc::translate<int16_t>(-.012, -1.0, 0, -255, -65));
  EXPECT_DOUBLE_EQ(67, Misc::translate<int16_t>(0.012, 0, 1.0, 65, 255));
  EXPECT_DOUBLE_EQ(255, Misc::translate<int16_t>(1.0, 0, 1.0, 65, 255));
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

TEST(MiscTest, testModfint)
{
  double input = Misc::PI;
  uint8_t int_part = 0;
  uint8_t fract_part = Misc::modfint<uint8_t>(input, &int_part, 2);
  ASSERT_EQ(3, int_part);
  ASSERT_EQ(14, fract_part);

  fract_part = Misc::modfint<uint8_t>(Misc::PI, &int_part, 1);
  ASSERT_EQ(3, int_part);
  ASSERT_EQ(1, fract_part);

  fract_part = Misc::modfint<uint8_t>(Misc::PI, &int_part, 0);
  ASSERT_EQ(3, int_part);
  ASSERT_EQ(0, fract_part);

  input = 65535.65536;
  fract_part = Misc::modfint<uint8_t>(input, &int_part, 5);
  ASSERT_EQ(255, int_part);
  ASSERT_EQ(0, fract_part);

  input = 65536.65535;
  uint16_t int_part16 = 0;
  uint16_t fract_part16 = Misc::modfint<uint16_t>(input, &int_part16, 5);
  ASSERT_EQ(0, int_part16);
  ASSERT_EQ(65535, fract_part16);
}

} // namespace btr
