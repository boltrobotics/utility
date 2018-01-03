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
#include "utility/value_tracker.hpp"

//================================ TEST FIXTURES ===============================

//=================================== TESTS ====================================

namespace btr
{

TEST(ValueTrackerTest, testValue)
{
  ValueTracker<double> tracker(3);

  // The tracker is empty at this point and all values are to be 0.
  ASSERT_EQ(0, tracker.value(0));

  tracker.push(1);
  tracker.push(2);
  tracker.push(3);

  ASSERT_EQ(1, tracker.value(0));
  ASSERT_EQ(2, tracker.value(1));
  ASSERT_EQ(3, tracker.value(2));

  tracker.push(4);
  tracker.push(5);

  ASSERT_EQ(3, tracker.value(0));
  ASSERT_EQ(4, tracker.value(1));
  ASSERT_EQ(5, tracker.value(2));

  tracker.push(6);

  ASSERT_EQ(4, tracker.value(0));
  ASSERT_EQ(5, tracker.value(1));
  ASSERT_EQ(6, tracker.value(2));
}

TEST(ValueTrackerTest, testDelta)
{
  uint32_t count = 5;
  ValueTracker<uint32_t> tracker(count);

  // The tracker is empty at this point and all values are to be 0.
  ASSERT_EQ(count, tracker.count());
  ASSERT_EQ(uint32_t(0), tracker.value(0));

  // Push 10 values.
  uint32_t last_val = 10;

  for (uint32_t i = 1; i <= last_val; i++) {
    tracker.push(i);
  }

  for (uint32_t i = 0, expected = 6; i < count; i++, expected++) {
    ASSERT_EQ(expected, tracker.value(i)) << "Loop index: " << i;
  }

  ASSERT_EQ(uint32_t(1), tracker.delta());

  // Compare the delta for all combinations of values.
  // The second position to tracker.delta() must be less than or equal to
  // the first.
  //
  for (uint32_t i = count; i > 0; --i) {
    for (uint32_t j = i, delta = 0; j > 0; --j, ++delta) {
      ASSERT_EQ(delta, tracker.delta(i - 1, j - 1))
        << "Loop index: " << (i-1) << "," << (j-1);
    }
  }
}

TEST(ValueTrackerTest, testInvalidDelta)
{
  ValueTracker<uint16_t> tracker(3);

  for (uint16_t i = 5; i <= 7; i++) {
    tracker.push(i);
  }

  ASSERT_EQ(7, tracker.value(2));
  ASSERT_EQ(6, tracker.value(1));
  ASSERT_EQ(5, tracker.value(0));
  ASSERT_EQ(2, tracker.delta(2, 0));

  // The delta for value 5 - 7 = -2, which is 65534 for unsigned 16-bit integer.
  //
  ASSERT_EQ(65534, tracker.delta(0, 2));
}

TEST(ValueTrackerTest, testMedianOdd)
{
  uint16_t data[] = { 7, 5, 2, 16, 4 };
  uint32_t size = sizeof(data) / sizeof(uint16_t);
  ValueTracker<uint16_t> tracker(size);

  for (uint32_t i = 0; i < size; i++) {
    tracker.push(data[i]);
  }

  uint16_t v = tracker.median();
  ASSERT_EQ(5, v);
}

TEST(ValueTrackerTest, testMedianEven)
{
  uint16_t data[] = { 7, 5, 2, 16, 4, 5 };
  uint32_t size = sizeof(data) / sizeof(uint16_t);
  ValueTracker<uint16_t> tracker(size);

  for (uint32_t i = 0; i < size; i++) {
    tracker.push(data[i]);
  }

  uint16_t v = tracker.median();
  ASSERT_EQ(5, v);
}

TEST(ValueTrackerTest, testMean)
{
  uint16_t data[] = { 7, 5, 2, 16, 4, 5 };
  uint32_t size = sizeof(data) / sizeof(uint16_t);
  ValueTracker<uint16_t> tracker(size);

  for (uint32_t i = 0; i < size; i++) {
    tracker.push(data[i]);
  }

  uint16_t v = tracker.mean();
  ASSERT_EQ(6, v);
}

} // namespace btr
