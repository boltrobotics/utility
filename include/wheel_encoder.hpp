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

#ifndef _btr_WheelEncoder_hpp_
#define _btr_WheelEncoder_hpp_

// SYSTEM INCLUDES
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
   * @param a_state - 1 if A encoder output is set, false otherwise
   * @param b_state - 1 if B encoder output is set, false otherwise
   * @param direction_step - 1 for left encoder (B follows A), -1 for right
   *      encoder (A follows B)
   */
  WheelEncoder(int8_t a_state, int8_t b_state, int8_t direction_step);

  // OPERATIONS

  /**
   * Update encoder click count.
   *
   * @param a_state - 1 if A encoder output is set, 0 otherwise
   * @param b_state - 1 if B encoder output is set, 0 otherwise
   */
  void update(int8_t a_state, int8_t b_state);

  /**
   * Reset the number of clicks to zero.
   */
  void reset();

  /**
   * @return the number of clicks
   */
  int16_t clicks() const;

private:

  // ATTRIBUTES

  int16_t clicks_;
  int8_t direction_;
  int8_t direction_step_;
  int8_t a_state_;
  int8_t b_state_;

}; // class WheelEncoder

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

inline WheelEncoder::WheelEncoder(int8_t a_state, int8_t b_state, int8_t direction_step)
: clicks_(0),
  direction_(0),
  direction_step_(direction_step),
  a_state_(a_state),
  b_state_(b_state)
{
}

//=================================== OPERATIONS ===============================

inline void WheelEncoder::update(int8_t a_state, int8_t b_state)
{
  // If B output follows A, use positive direction, reverse sign if A
  // follows B.
  //
  if (a_state_ != a_state) {
    a_state_ = a_state;

    if (a_state_ != b_state_) {
      direction_ = direction_step_;
    } else {
      direction_ = -direction_step_;
    }
    clicks_ += direction_;
  } else if (b_state_ != b_state) {
    b_state_ = b_state;

    if (b_state_ != a_state_) {
      direction_ = -direction_step_;
    } else {
      direction_ = direction_step_;
    }
    clicks_ += direction_;
  }
}

inline void WheelEncoder::reset()
{
  clicks_ = 0;
}

inline int16_t WheelEncoder::clicks() const
{
  return clicks_;
}

} // namespace btr

#endif // _btr_WheelEncoder_hpp_
