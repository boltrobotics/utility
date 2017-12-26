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
#include "wheel_encoder.hpp"

namespace btr
{

//================================ TEST FIXTURES ===============================

class WheelEncoderTest : public testing::Test
{
public:

  // LIFECYCLE

  WheelEncoderTest()
  : enc_(0, 0, 1)
  {
  }

protected:

  // ATTRIBUTES

  WheelEncoder enc_;

}; // WheelEncoderTest

//=================================== TESTS ====================================

TEST_F(WheelEncoderTest, testClicks)
{
    // Test forward motion. B output follows A.
    enc_.update(1, 0);
    enc_.update(1, 1);
    enc_.update(0, 1);
    enc_.update(0, 0);
    EXPECT_EQ(4, enc_.clicks());

    // Test reverse motion. A output follows B.
    enc_.update(0, 1);
    enc_.update(1, 1);
    enc_.update(1, 0);
    enc_.update(0, 0);
    EXPECT_EQ(0, enc_.clicks());
}

TEST_F(WheelEncoderTest, testReset)
{
}

} // namespace btr
