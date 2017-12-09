/* Copyright (C) 2017 Sergey Kapustin <boltrobotics.com>
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

#ifndef _utility_ValueTracker_hpp_
#define _utility_ValueTracker_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES

namespace utility {

/**
 * The class stores two values, prior and current, so as to track delta
 * between the two.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
template<typename T>
class ValueTracker {
public:

// LIFECYCLE

    /**
     * Ctor.
     *
     * @param count - the number of values to track
     */
    ValueTracker(uint32_t count = 2);

    /**
     * Dtor.
     */
    ~ValueTracker();

// OPERATIONS

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

private:

// OPERATIONS

// ATTRIBUTES

    T* vals_;
    uint32_t count_;
    uint32_t pos_;

}; // class ValueTracker

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

template<typename T>
inline ValueTracker<T>::ValueTracker(uint32_t count) :
    vals_(NULL),
    count_(count),
    pos_(0) {

    // Create the array and set all values to default for the type.
    vals_ = new T[count]();
}

template<typename T>
inline ValueTracker<T>::~ValueTracker() {
    delete [] vals_;
    vals_ = NULL;
}

//=================================== OPERATIONS ===============================

template<typename T>
inline void ValueTracker<T>::push(T val) {
    vals_[pos_ % count_] = val;
    ++pos_;
}

template<typename T>
inline uint32_t ValueTracker<T>::count() const {
    return count_;
}

template<typename T>
inline T ValueTracker<T>::last() const {
    // Assume that at least one value was pushed.
    //
    return vals_[(pos_ - 1) % count_];
}

template<typename T>
inline T ValueTracker<T>::value(uint32_t requested_pos) const {
    uint32_t index = (pos_ - 1) - ((count_ - 1) - requested_pos);
    return vals_[index % count_];
}

template<typename T>
inline T ValueTracker<T>::delta() const {
    return delta(count_ - 1, count_ - 2);
}

template<typename T>
inline T ValueTracker<T>::delta(uint32_t upper_pos, uint32_t lower_pos) const {
    T upper_val = value(upper_pos);
    T lower_val = value(lower_pos);
    return (upper_val - lower_val);
}

} // namespace utility

#endif // _utility_ValueTracker_hpp_
