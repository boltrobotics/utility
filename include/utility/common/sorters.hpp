// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

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
   * @param arr - elements to sort
   * @param size - the number of elements
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
