// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "utility/common/shared_ptr.hpp"
#include "utility/common/test_helpers.hpp"

namespace btr
{

TEST(SharedPtrTest, defaultCtor)
{
  SharedPtr<int> sp;
  ASSERT_EQ(1, sp.count());
}

TEST(SharedPtrTest, copyCtor)
{
  SharedPtr<int> sp1(new int(5));
  SharedPtr<int> sp2(sp1);

  ASSERT_EQ(2, sp1.count());
  ASSERT_EQ(2, sp2.count());
  ASSERT_EQ(5, *sp1);
  ASSERT_EQ(5, *sp2);

  *sp1 += 2;
  ASSERT_EQ(7, *sp1);
  ASSERT_EQ(7, *sp2);

  *sp2 -= 3;
  ASSERT_EQ(4, *sp1);
  ASSERT_EQ(4, *sp2);
}

TEST(SharedPtrTest, assignmentCtor)
{
  SharedPtr<int> sp1(new int(5));
  SharedPtr<int> sp2 = sp1;

  ASSERT_EQ(2, sp1.count());
  ASSERT_EQ(2, sp2.count());
  ASSERT_EQ(5, *sp1);
  ASSERT_EQ(5, *sp2);

  *sp1 += 2;
  ASSERT_EQ(7, *sp1);
  ASSERT_EQ(7, *sp2);

  *sp2 -= 3;
  ASSERT_EQ(4, *sp1);
  ASSERT_EQ(4, *sp2);
}

TEST(SharedPtrTest, pointerCtor)
{
  SharedPtr<int> sp(new int(5));
  ASSERT_EQ(1, sp.count());
  ASSERT_EQ(5, *sp);
}

} // namespace btr
