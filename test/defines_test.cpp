// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <gtest/gtest.h>

// PROJECT INCLUDES
#include "utility/defines.hpp"

//================================ TEST FIXTURES ===============================

//=================================== TESTS ====================================

namespace btr
{

TEST(StatusTest, set_status)
{
  uint32_t s1 = 0xFF000000;
  uint32_t s2 = 0x00FF0000;
  uint32_t vv = 0x0000FFFF;
  uint32_t status = vv;
  set_status(&status, s1);
  set_status(&status, s2);
  set_status(&status, 0);

  ASSERT_EQ((s1 | s2 | vv), status);
  ASSERT_EQ(false, is_ok(status));
  ASSERT_EQ(false, is_ok(&status));
  ASSERT_EQ(true, is_err(status));
  ASSERT_EQ(true, is_err(&status));

  status &= 0xFFFF0000;
  ASSERT_EQ((s1 | s2), status);
  ASSERT_EQ(false, is_ok(status));
  ASSERT_EQ(false, is_ok(&status));
  ASSERT_EQ(true, is_err(status));
  ASSERT_EQ(true, is_err(&status));

}

TEST(StatusTest, clear_status)
{
  uint32_t s1 = 0xFF000000;
  uint32_t s2 = 0x00FF0000;
  uint32_t vv = 0x0000FFFF;
  uint32_t status = vv;
  set_status(&status, s1);
  set_status(&status, s2);
  set_status(&status, 0);

  clear_status(&status);
  ASSERT_EQ(uint32_t(0x0000FFFF), status);
  ASSERT_EQ(true, is_ok(status));
  ASSERT_EQ(true, is_ok(&status));
  ASSERT_EQ(false, is_err(status));
  ASSERT_EQ(false, is_err(&status));
}

TEST(StatusTest, is_set)
{
  uint32_t s1 = 0x00120000;
  uint32_t s2 = 0x10080000;
  uint32_t s3 = 0x80100000;
  uint32_t status = 0;

  set_status(&status, s1);
  set_status(&status, s3);

  ASSERT_EQ(true,  is_set(&status, s1));
  ASSERT_EQ(false, is_set(&status, s2));
  ASSERT_EQ(true,  is_set(&status, s3));

  set_status(&status, s2);
  ASSERT_EQ(true, is_set(&status, s1));
  ASSERT_EQ(true, is_set(&status, s2));
  ASSERT_EQ(true, is_set(&status, s3));
}

} // namespace btr
