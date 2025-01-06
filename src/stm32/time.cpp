// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PROJECT INCLUDES
#include "utility/common/time.hpp"  // class implemented

#if BTR_TIME_ENABLED > 0

namespace btr
{

static constexpr uint16_t RATE_SCALER_MS = 1000 / configTICK_RATE_HZ;

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
  return (xTaskGetTickCount() / configTICK_RATE_HZ);
}

// static
uint32_t Time::millis()
{
  return (xTaskGetTickCount() * RATE_SCALER_MS);
}

// static
uint32_t Time::diff(uint32_t head_time, uint32_t tail_time)
{
  return ((UINT32_MAX + head_time - tail_time) % UINT32_MAX);
}

} // namespace btr

#endif // BTR_TIME_ENABLED > 0
