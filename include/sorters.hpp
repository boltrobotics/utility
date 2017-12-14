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

#ifndef _utility_Sorters_hpp_
#define _utility_Sorters_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES

namespace utility {

/**
 * A one line description of the class.
 *
 * A longer description.
 */
class Sorters {
public:

// OPERATIONS

    /**
     * Sort array in-place using insertion sort.
     *
     * @param arr the elements to sort
     */
    template<typename T>
    static void insertionSort(T* arr, uint32_t size);

private:

// LIFECYCLE

    /**
     * Ctor.
     */
    Sorters();

    /**
     * Dtor.
     */
    virtual ~Sorters();

// ATTRIBUTES

}; // class Sorters

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

template<typename T>
inline void Sorters::insertionSort(T* arr, uint32_t size) {

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

} // namespace utility

#endif // _utility_Sorters_hpp_
