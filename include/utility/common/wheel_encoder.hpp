// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_WheelEncoder_hpp_
#define _btr_WheelEncoder_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

// PROJECT INCLUDES

namespace btr
{

/**
 * The class counts the number of clicks that a virtual wheel moved forward and
 * backward.
 */
class WheelEncoder
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param a_state - 1 if A encoder output is set, 0 - if clear
   * @param b_state - 1 if B encoder output is set, 0 - if clear
   * @param direction_step - 1 for left encoder (B follows A), -1 for right
   *      encoder (A follows B)
   */
  WheelEncoder(uint8_t a_state, uint8_t b_state, uint8_t direction_step);

// OPERATIONS

  /**
   * Update encoder click count.
   *
   * @param a_state - 1 if A encoder output is set, 0 otherwise
   * @param b_state - 1 if B encoder output is set, 0 otherwise
   */
  void update(uint8_t a_state, uint8_t b_state);

  /**
   * Reset the number of clicks to zero.
   */
  void reset();

  /**
   * @return the number of clicks
   */
  uint16_t clicks() const;

private:

// ATTRIBUTES

  volatile uint16_t clicks_;
  volatile uint8_t direction_step_;
  volatile uint8_t a_state_;
  volatile uint8_t b_state_;

}; // class WheelEncoder

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline WheelEncoder::WheelEncoder(uint8_t a_state, uint8_t b_state, uint8_t direction_step)
  :
    clicks_(0),
    direction_step_(direction_step),
    a_state_(a_state),
    b_state_(b_state)
{
}

//============================================= OPERATIONS =========================================

inline void WheelEncoder::update(uint8_t a_state, uint8_t b_state)
{
  // Before unsigned 16-bit value rolls over, with 4"-diameter wheel, 75:1 gear ratio and
  // 48CPR encoder, a rover will travel:
  // Pi * 4 * 2^16 / (75 * 48) = 228.764" (or 5.81m = 581.06cm = 228.764" * 2.54cm)
  //
  // If B output follows A, use positive direction, reverse sign if A follows B.
  //
  if (a_state_ != a_state) {
    a_state_ = a_state;

    if (a_state_ != b_state_) {
      clicks_ += direction_step_;
    } else {
      clicks_ -= direction_step_;
    }
  } else if (b_state_ != b_state) {
    b_state_ = b_state;

    if (b_state_ != a_state_) {
      clicks_ -= direction_step_;
    } else {
      clicks_ += direction_step_;
    }
  }
}

inline void WheelEncoder::reset()
{
  clicks_ = 0;
}

inline uint16_t WheelEncoder::clicks() const
{
  return clicks_;
}

} // namespace btr

#endif // _btr_WheelEncoder_hpp_
