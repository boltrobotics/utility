// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_ValueTracker_hpp_
#define _btr_ValueTracker_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/sorters.hpp"

namespace btr
{

/**
 * The class keeps track of numeric values so as to calculate the delta, mean or median between
 * them. 
 */
template<typename T, uint8_t S = 5>
class ValueTracker
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param id - value tracker ID
   */
  ValueTracker(uint8_t id = 0);

// OPERATIONS

  /**
   * @return the identifier
   */
  uint8_t id() const;

  /**
   * Set all samples to 0.
   */
  void reset(T val = 0);

  /**
   * Add new value to the list.
   *
   * @param val - the value to add
   */
  void push(T val);

  /**
   * @return the count of tracked values 
   */
  uint8_t count() const;

  /**
   * @return the last pushed value
   */
  T last() const;

  /**
   * @return the value at given requested position. Positions start at 0.
   */
  T value(uint8_t requested_pos) const;

  /**
   * @return delta between last and one-before last values
   */
  T delta() const;

  /**
   * Compute the different between two values and return the result.
   *
   * @param upper_pos - the first value position
   * @param lower_pos - the second value position
   * @return the delta as in (upper value - lower value)
   */
  T delta(uint8_t upper_pos, uint8_t lower_pos) const;

  /**
   * @return the median value
   */
  T median() const;

  /**
   * @return the median value
   */
  T mean() const;

private:

// ATTRIBUTES

  T vals_[S];
  uint8_t pos_;
  uint8_t id_;

}; // class ValueTracker

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

template<typename T, uint8_t S>
inline ValueTracker<T,S>::ValueTracker(uint8_t id)
  :
    vals_(),
    pos_(0),
    id_(id)
{
}

//=================================== OPERATIONS ===============================

template<typename T, uint8_t S>
inline uint8_t ValueTracker<T,S>::id() const
{
  return id_;
}

template<typename T, uint8_t S>
inline void ValueTracker<T,S>::reset(T val)
{
  for (uint8_t i = 0; i < S; i++) {
    vals_[i] = val;
  }
}

template<typename T, uint8_t S>
inline void ValueTracker<T,S>::push(T val)
{
  vals_[pos_ % S] = val;
  ++pos_;
}

template<typename T, uint8_t S>
inline uint8_t ValueTracker<T,S>::count() const
{
  return S;
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::last() const
{
  // Assume that at least one value was pushed.
  //
  return vals_[(pos_ - 1) % S];
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::value(uint8_t requested_pos) const
{
  uint16_t index = (pos_ - 1) - ((S - 1) - requested_pos);
  return vals_[index % S];
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::delta() const
{
  return delta(S - 1, S - 2);
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::delta(uint8_t upper_pos, uint8_t lower_pos) const
{
  T upper_val = value(upper_pos);
  T lower_val = value(lower_pos);
  return (upper_val - lower_val);
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::median() const
{
  T arr[S] = { 0 };
  memcpy(arr, vals_, sizeof(T) * S);

  Sorters::insertionSort(arr, S);

  T result;

  if ((S % 2) != 0) {
    result = arr[S / 2];
  } else {
    result = (arr[S / 2] + arr[(S / 2) - 1]) / 2;
  }
  return result;
}

template<typename T, uint8_t S>
inline T ValueTracker<T,S>::mean() const
{
  T sum = 0;

  for (uint8_t i = 0; i < S; i++) {
    sum += vals_[i];
  }
  return sum / S;
}

} // namespace btr

#endif // _btr_ValueTracker_hpp_
