/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef _utility_ValueCodec_hpp_
#define _utility_ValueCodec_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "buff.hpp"

namespace utility {

/**
 * The class converts between raw character data and C++ types.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
class ValueCodec {
public:

    enum Result {
        SUCCESS     = 0,
        SMALL_BUFF  = 1,
        SMALL_VALUE = 2
    };

// OPERATIONS

    /**
     * Extract an integer of type IntType from size bytes.
     *
     * @param buff - character data container
     * @param val - target value
     * @parma size - the number of characters that the integer occupies
     * @param msb - the data is in most-significant byte order 
     * @return the values returned by check()
     */
    template<typename IntType>
    static int getInt(Buff* buff, IntType* val, uint32_t size, bool msb = true);

    /**
     * @return one of the Result values:
     *  0 - on success
     *  1 - if buff->available() is less than source_size
     *  2 - if target_size is less than source size
     */
    static int check(Buff* buff, uint32_t target_size, uint32_t source_size);

private:

// LIFECYCLE

    // Not supported.
    //
    ValueCodec();
    ~ValueCodec();
    ValueCodec(const ValueCodec&);
    ValueCodec& operator=(const ValueCodec&);

}; // class ValueCodec

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

template<typename IntType>
inline int ValueCodec::getInt(
        Buff* buff, IntType* val, uint32_t size, bool msb) {

    int success = check(buff, sizeof(IntType), size);

    if (success == SUCCESS) {
        const uint8_t* data = buff->read_ptr();

        if (msb) {
            for (register uint32_t i = 0; i < size; i++) {
                uint8_t c = data[i];
                *val = ((*val << 8) | c);
            }
        } else {
            for (register int32_t i = int32_t(size); --i >= 0; ) {
                uint8_t c = data[i];
                *val = ((*val << 8) | c);
            }
        }

        buff->advanceReadPtr(size);
    }
    return success;
}

inline int ValueCodec::check(
        Buff* buff, uint32_t target_size, uint32_t source_size) {

    int success = 0;

    if (buff->available() < source_size) {
        success = SMALL_BUFF;
    } else if (source_size > target_size) {
        success = SMALL_VALUE;
    }
    return success;
}

} // namespace utility

#endif // _utility_ValueCodec_hpp_
