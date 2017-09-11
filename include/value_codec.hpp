/* Copyright (c) 2017, Sergey V. Kapustin <svkapustin.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef _utility_ValueCodec_hpp_
#define _utility_ValueCodec_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "buff.hpp"

namespace utility {

#define ARRAY(...) (const uint8_t[]) { __VA_ARGS__ }

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
     * @param bits - the number of bits
     * @return a mask representing the bit number
     */
    static uint8_t getMask(uint8_t bits);

    /**
     * Compose an integer of type IntType from variable length bit pattern.
     * If a byte of an integer has high bit set, more bits should be added from
     * the next byte. If the high bit is clear, this byte has the last 7 bits.
     *
     * @param - buff the data buffer
     * @param - target integer
     * @return SUCCESS or one of the error codes
     */
    template<typename IntType>
    static int getVarInt(Buff* buff, IntType* val);

    /**
     * Compose an integer of type IntType from the bits of the buffer.
     *
     * IMPORTANT: This function doesn't check for buffer and source/target
     * sizes
     *
     * @param buff - character data container
     * @param val - target value
     * @param bit_map - a high nibble of each entry indicates whether to get the
     *  number of bits, specified in the low nibble, starting from most-
     *  significant bit. Otherwise, use low-significant bits.
     * @see value_codec_test.cpp to clarify
     */
    template<typename IntType, typename IdxType, uint32_t N>
    static IntType getInt(Buff* buff, const IdxType (&bit_map)[N]);

    /**
     * Extract an integer of type IntType using only n bytes from the buffer.
     *
     * IMPORTANT: This function doesn't check for buffer and source/target
     * sizes
     *
     * @param buff - character data container
     * @param val - target value
     * @parma bytes - the number of bytes that the integer occupies
     * @param msb - the data is in most-significant byte order 
     */
    template<typename IntType>
    static IntType getInt(Buff* buff, uint32_t bytes, bool msb = true);

    /**
     * Check buffer and source/target sizes, then call getInt()
     *
     * @param buff - character data container
     * @param val - target value
     * @parma bytes - the number of bytes that the integer occupies
     * @param msb - the data is in most-significant byte order 
     * @return the values returned by check()
     */
    template<typename IntType>
    static int getInt(Buff* buff, IntType* val, uint32_t bytes, bool msb = true);

    /**
     * @return one of the Result values:
     *  0 - on success
     *  1 - if buff->available() is less than source_size
     *  2 - if target_size is less than source size
     */
    static int check(Buff* buff, uint32_t target_size, uint32_t source_size);

    /**
     * @return true if this host is little endian
     */
    static bool isLittleEndian();

    /**
     * Modulo operator to handle negative numbers. The % operator in C is not the
     * modulo but the remainder operator.
     *
     * @param a - left parameter
     * @param b - right parameter
     */
    template<typename T, typename U>
    static T modulo(T a, U b);

    /**
     * Convert a value between MSB and LSB ordering.
     *
     * @param val - the value
     */
    template<typename ValueType>
    static void swap(ValueType* val);

// ATTRIBUTES

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

inline uint8_t ValueCodec::getMask(uint8_t bits) {
    // Define a mapping from a number of bits (array index) to the bit pattern.
    //
    static const uint8_t mask[] = {
        0x0, 0x1, 0x3, 0x7, 0xF, 0x1F , 0x3F, 0x7F, 0xFF };
    return mask[bits];
}

template<typename IntType>
inline int ValueCodec::getVarInt(Buff* buff, IntType* val) {

    int success = SMALL_VALUE; 
    uint8_t bits = 7;
    uint8_t bit_map[] = { bits };
    int8_t bits_remain = sizeof(IntType) * 8;

    for (; bits_remain >= 0; bits_remain -= bits) {
        if (buff->available() == 0) { 
            success = SMALL_BUFF;
            break;
        }

        if (bits_remain < bits) {
            bit_map[0] = bits_remain;
        }

        uint8_t byte = *buff->read_ptr();
        uint8_t v = getInt<uint8_t>(buff, bit_map);
        *val = ((*val << bit_map[0]) | v);

        if ((byte & 0x80) == 0) {
            success = SUCCESS;
            break;
        }
    }

    return success;
}

template<typename IntType, typename IdxType, uint32_t N>
inline IntType ValueCodec::getInt(Buff* buff, const IdxType (&bit_map)[N]) {

    IntType val = 0;

    for (register uint32_t i = 0; i < N; i++) {
        uint8_t c = buff->read_ptr()[i];

        // When most-significatn bit in bit_map is set, get the number of bits,
        // which is specified in low-nibble, from the data byte starting at
        // most-significant bit position. Otherwise, get that number of bits
        // from the least-significant side.
        //
        uint8_t bits = bit_map[i] & 0x0F; // 0 through 8
        uint8_t mask = getMask(bits);
        uint8_t high = (bit_map[i] >> 7) & 1;
        uint8_t v = (high ? (c >> (8 - bits)) : c);
        val = ((val << bits) | (v & mask));
    }

    buff->advanceReadPtr(N);
    return val;
}

template<typename IntType>
inline IntType ValueCodec::getInt(Buff* buff, uint32_t bytes, bool msb) {

    IntType val = 0;

    if (msb) {
        for (register uint32_t i = 0; i < bytes; i++) {
            uint8_t c = buff->read_ptr()[i];
            val = ((val << 8) | c);
        }
    } else {
        for (register int32_t i = int32_t(bytes); --i >= 0; ) {
            uint8_t c = buff->read_ptr()[i];
            val = ((val << 8) | c);
        }
    }

    buff->advanceReadPtr(bytes);
    return val;
}

template<typename IntType>
inline int ValueCodec::getInt(
        Buff* buff, IntType* val, uint32_t bytes, bool msb) {

    int success = check(buff, sizeof(IntType), bytes);

    if (success == SUCCESS) {
        *val = getInt<IntType>(buff, bytes, msb);
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

inline bool ValueCodec::isLittleEndian() {
    int16_t number = 0x0001;
    char* ptr = (char*) &number;
    return (ptr[0] == 1);
}

template<typename T, typename U>
inline T ValueCodec::modulo(T a, U b) {
    T r = a % b;
    return (r < 0 ? r + b : r);
}

template<typename ValueType>
inline void ValueCodec::swap(ValueType* val) {

    uint8_t* bytes = reinterpret_cast<uint8_t*>(val);
    register int j = sizeof(ValueType) - 1;

    for (register int i = 0; i < j; i++, j--) {
        uint8_t tmp = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = tmp;
    }
}

} // namespace utility

#endif // _utility_ValueCodec_hpp_
