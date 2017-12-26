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
#include "misc.hpp"

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

} // namespace btr
