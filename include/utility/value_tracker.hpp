// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_ValueTracker_hpp_
#define _btr_ValueTracker_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/sorters.hpp"

namespace btr
{

/**
 * The class stores two values, prior and current, so as to track delta
 * between the two.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
template<typename T>
class ValueTracker
{
public:

  // LIFECYCLE

  /**
   * Ctor.
   *
   * @param count - the number of values to track
   */
  ValueTracker(uint32_t count = 2, uint8_t id = 0);

  /**
   * Dtor.
   */
  ~ValueTracker();

  // OPERATIONS

  /**
   * @return the identifier
   */
  uint8_t id() const;

  /**
   * Set all samples to 0.
   */
  void clear();

  /**
   * Add new value to the list.
   *
   * @param val - the value to add
   */
  void push(T val);

  /**
   * @return the count of tracked values 
   */
  uint32_t count() const;

  /**
   * @return the last pushed value
   */
  T last() const;

  /**
   * @return the value at given requested position. Positions start at 0.
   */
  T value(uint32_t requested_pos) const;

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
  T delta(uint32_t upper_pos, uint32_t lower_pos) const;

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

  T* vals_;
  uint32_t count_;
  uint32_t pos_;
  uint8_t id_;

}; // class ValueTracker

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

template<typename T>
inline ValueTracker<T>::ValueTracker(uint32_t count, uint8_t id)
  :
  vals_(new T[count]()),
  count_(count),
  pos_(0),
  id_(id)
{
}

template<typename T>
inline ValueTracker<T>::~ValueTracker()
{
  delete [] vals_;
  vals_ = nullptr;
}

//=================================== OPERATIONS ===============================

template<typename T>
inline uint8_t ValueTracker<T>::id() const
{
  return id_;
}

template<typename T>
inline void ValueTracker<T>::clear()
{
  for (int i = 0; i < count_; i++) {
    vals_[i] = 0;
  }
}

template<typename T>
inline void ValueTracker<T>::push(T val)
{
  vals_[pos_ % count_] = val;
  ++pos_;
}

template<typename T>
inline uint32_t ValueTracker<T>::count() const
{
  return count_;
}

template<typename T>
inline T ValueTracker<T>::last() const
{
  // Assume that at least one value was pushed.
  //
  return vals_[(pos_ - 1) % count_];
}

template<typename T>
inline T ValueTracker<T>::value(uint32_t requested_pos) const
{
  uint32_t index = (pos_ - 1) - ((count_ - 1) - requested_pos);
  return vals_[index % count_];
}

template<typename T>
inline T ValueTracker<T>::delta() const
{
  return delta(count_ - 1, count_ - 2);
}

template<typename T>
inline T ValueTracker<T>::delta(uint32_t upper_pos, uint32_t lower_pos) const
{
  T upper_val = value(upper_pos);
  T lower_val = value(lower_pos);
  return (upper_val - lower_val);
}

template<typename T>
inline T ValueTracker<T>::median() const
{
  T* arr = new T[count_];
  memcpy(arr, vals_, count_ * sizeof(T));

  Sorters::insertionSort(arr, count_);

  T result;

  if ((count_ % 2) != 0) {
    result = arr[count_ / 2];
  } else {
    result = (arr[count_ / 2] + arr[(count_ / 2) - 1]) / 2;
  }

  delete [] arr;
  return result;
}

template<typename T>
inline T ValueTracker<T>::mean() const
{
  T sum = 0;

  for (uint32_t i = 0; i < count_; i++) {
    sum += vals_[i];
  }
  return sum / count_;
}

} // namespace btr

#endif // _btr_ValueTracker_hpp_
