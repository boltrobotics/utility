// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "utility/common/maxsonar_lvez.hpp"

namespace btr
{

//========================================== TEST FIXTURES =========================================

class MaxSonarLvEzTest : public testing::Test
{
public:

  // LIFECYCLE

  MaxSonarLvEzTest()
  {
  }

protected:

  // ATTRIBUTES

  //MaxSonarLvEz sonar_;

}; // MaxSonarLvEzTest

//============================================= TESTS ==============================================

TEST_F(MaxSonarLvEzTest, testRange)
{
  uint16_t adc_sample = 61;
  uint16_t range = MaxSonarLvEz::range(adc_sample);
  ASSERT_EQ(774, range);
}

} // namespace btr
