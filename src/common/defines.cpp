// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// PROJECT INCLUDES
#include "utility/common/defines.hpp"  // class implemented

namespace btr
{
namespace dev
{

#if BTR_STATUS_ENABLED > 0
static uint32_t status_;

uint32_t* status()
{
  return &status_;
}
#else
uint32_t* status()
{
  return nullptr;
}
#endif // BTR_STATUS_ENABLED > 0

} // namespace dev
} // namespace btr
