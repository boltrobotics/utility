// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "utility/common/wheel_encoder.hpp"

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
