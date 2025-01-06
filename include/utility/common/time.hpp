// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Time_hpp_
#define _btr_Time_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to time-keeping functions.
 */
class Time
{
public:

// OPERATIONS

  /**
   * Initialize.
   */
  static void init();

  /**
   * Shut down the time-keeping functions.
   */
  static void shutdown();

  /**
   * @return seconds since this uS started running
   */
  static uint32_t sec();

  /**
   * @return milliseconds since this uS started running
   */
  static uint32_t millis();

  /**
   * @param head_time - the time that comes after or equal to tail_time
   * @param tail_time - the time that comes before or equal to head_time
   * @return time difference between two values
   */
  static uint32_t diff(uint32_t head_time, uint32_t tail_time);
};

} // namespace btr

#endif // _btr_Time_hpp_
