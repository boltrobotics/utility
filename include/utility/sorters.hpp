// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_Sorters_hpp_
#define _btr_Sorters_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

/**
 * The class provide facilities to sort data.
 *
 * IMPORTANT: Implementation has to be compatible with AVR platform.
 */
class Sorters
{
public:

  // LIFECYCLE

  Sorters() = delete;
  ~Sorters() = delete;

  // OPERATIONS

  /**
   * Sort array in-place using insertion sort.
   *
   * @param arr the elements to sort
   */
  template<typename T>
  static void insertionSort(T* arr, uint32_t size);

}; // class Sorters

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
inline void Sorters::insertionSort(T* arr, uint32_t size)
{
  for (uint32_t i = 1; i < size; i++) {
    uint32_t j = i;

    while (j > 0 && arr[j - 1] > arr[j]) {
      T tmp = arr[j];
      arr[j] = arr[j - 1];
      arr[j - 1] = tmp;
      j--;
    }
  }
}

} // namespace btr

#endif // _btr_Sorters_hpp_
