// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include "esp_timer.h"

// PROJECT INCLUDES
#include "utility/common/time.hpp"  // class implemented

#if BTR_TIME_ENABLED > 0

////////////////////////////////////////////////////////////////////////////////////////////////////
// Local defines {

// } Local defines

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static members {

// } Static members

////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs {

extern "C" {
} // extern "C"

// } ISRs

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
void Time::init()
{
  // Noop
}

// static
void Time::shutdown()
{
  // Noop
}

// static
uint32_t Time::sec()
{
  // ESP provides time in microseconds since boot (int64_t)
  return (esp_timer_get_time() / 1000000);
}

// static
uint32_t Time::millis()
{
  return (esp_timer_get_time() / 1000);
}

// static
uint32_t Time::diff(uint32_t head_time, uint32_t tail_time)
{
  return ((UINT32_MAX + head_time - tail_time) % UINT32_MAX);
}

} // namespace btr

#endif // BTR_TIME_ENABLED > 0
