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

#ifndef _utility_ValueCodec_hpp_
#define _utility_ValueCodec_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "buff.hpp"

namespace utility {

#define ARRAY(...) (const uint8_t[]) { __VA_ARGS__ }

/**
 * The class converts between raw bytes and C++ types.
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

// LIFECYCLE

    ValueCodec() = delete;
    ~ValueCodec() = delete;
    ValueCodec(const ValueCodec&) = delete;
    ValueCodec& operator=(const ValueCodec&) = delete;

// OPERATIONS

    /**
     * @param bits - the number of bits
     * @return a mask representing the bit number
     */
    static uint8_t getMask(uint8_t bits);

    /**
     * Compose an integer of type IntType from a variable-length bit pattern.
     *
     * 7 bits of each byte are used to reconstruct the number. If a byte of
     * an integer has high bit set, more bits should be read from the next byte
     * to reconstruct the target number. If the high bit is clear, this byte has
     * the last 7 bits.
     *
     * @param - buff the data buffer
     * @param - target integer
     * @return the value from Result enum
     */
    template<typename IntType>
    static int get7BitVarInt(Buff* buff, IntType* val);

    /**
     * Compose an integer of type IntType from the bits of the buffer.
     *
     * @see value_codec_test.cpp for additional information
     *
     * IMPORTANT: This function doesn't check for buffer and source/target
     * sizes
     *
     * @param buff - data container
     * @param val - target value
     * @param bit_map - a high nibble of each entry indicates whether to read the
     *      number of bits (specified in the low nibble) starting from most-
     *      significant bit. If the high bit is not set, read low-significant
     *      bits.
     * @param move_ptr - move buff's read pointer by N bytes
     */
    template<typename IntType, typename IdxType, uint32_t N>
    static IntType getNBitVarInt(
            Buff* buff, const IdxType (&bit_map)[N], bool move_ptr = true);

    /**
     * Extract an integer of type IntType using only n bytes from the buffer.
     *
     * IMPORTANT: This function doesn't check for buffer and source/target
     * sizes
     *
     * @param buff - data container
     * @param val - target value
     * @parma bytes - the number of bytes that the integer occupies
     * @param msb - the data is in most-significant byte order 
     */
    template<typename IntType>
    static IntType getFixedInt(Buff* buff, uint32_t bytes, bool msb = true);

    /**
     * Check buffer and source/target sizes, then call getFixedInt()
     *
     * @param buff - data container
     * @param val - target value
     * @parma bytes - the number of bytes that the integer occupies
     * @param msb - the data is in most-significant byte order 
     * @return the values returned by check()
     */
    template<typename IntType>
    static int getFixedInt(
            Buff* buff, IntType* val, uint32_t bytes, bool msb = true);

    /**
     * Encode a number in different host order.
     *
     * @param buff - the buffer to store encoded value
     * @param num - the value to encode
     * @param msb - if true, encode in MSB order, otherwise in LSB
     */
    template<typename NumType>
    static void encodeNum(Buff* buff, NumType num, bool msb = true);

    /**
     * Check if the provided data can be converted into requested number.
     *
     * @param buff - the data container
     * @param target_size - byte-size of target number (e.g. uint32_t == 4 bytes)
     * @param source_size - amount of bytes to use for conversion to the target
     *      number
     *
     * @return one of the Result values:
     *      SMALL_BUFF - if buff->available() is less than source_size. Not
     *          enough raw bytes in the buffer
     *      SMALL_VALUE - if target_size is less than source size. If the
     *          requested amount of bytes is converted to a number, the
     *          target integer type needs to be larger
     *      SUCCESS - if all conditions above are met
     */
    static int check(Buff* buff, uint32_t target_size, uint32_t source_size);

    /**
     * @return true if this host is little endian
     */
    static bool isLittleEndian();

    /**
     * Convert a value between MSB and LSB host order.
     *
     * @param num - the numeric value
     */
    template<typename NumType>
    static void swap(NumType* num);

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
inline int ValueCodec::get7BitVarInt(Buff* buff, IntType* val) {

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
        uint8_t v = getNBitVarInt<uint8_t>(buff, bit_map, true);
        *val = ((*val << bit_map[0]) | v);

        if ((byte & 0x80) == 0) {
            success = SUCCESS;
            break;
        }
    }

    return success;
}

template<typename IntType, typename IdxType, uint32_t N>
inline IntType ValueCodec::getNBitVarInt(
        Buff* buff, const IdxType (&bit_map)[N], bool move_ptr) {

    IntType val = 0;

    for (register uint32_t i = 0; i < N; i++) {
        uint8_t c = buff->read_ptr()[i];

        // @see valuce_codec_test.cpp to clarify the logic here
        //
        // Low nibble will contain a number of bits (0-8) that represents a
        // number (0-15) IF a most-significant bit of this byte is set.
        // If high bit is set, read the number of bits from the most-significant
        // bit position.
        // Otherwise, get that number of bits from the least-significant side.

        // How many bits represent a number? Lower nibble: 0 through 8
        uint8_t bits = bit_map[i] & 0x0F;
        uint8_t mask = getMask(bits);
        uint8_t high_bit_set = (bit_map[i] >> 7) & 1;
        uint8_t v = (high_bit_set ? (c >> (8 - bits)) : c);
        val = ((val << bits) | (v & mask));
    }

    if (move_ptr) {
        buff->advanceReadPtr(N);
    }
    return val;
}

template<typename IntType>
inline IntType ValueCodec::getFixedInt(Buff* buff, uint32_t bytes, bool msb) {

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
inline int ValueCodec::getFixedInt(
        Buff* buff, IntType* val, uint32_t bytes, bool msb) {

    int success = check(buff, sizeof(IntType), bytes);

    if (success == SUCCESS) {
        *val = getFixedInt<IntType>(buff, bytes, msb);
    }

    return success;
}

template<typename NumType>
inline void ValueCodec::encodeNum(Buff* buff, NumType num, bool msb) {
    if (isLittleEndian()) {
        if (msb) {
            swap(&num);
        }
    } else {
        if (!msb) {
            swap(&num);
        }
    }
    buff->write(num);
}

inline int ValueCodec::check(
        Buff* buff, uint32_t target_size, uint32_t source_size) {

    int success = SUCCESS;

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

template<typename NumType>
inline void ValueCodec::swap(NumType* num) {

    uint8_t* bytes = reinterpret_cast<uint8_t*>(num);
    register int j = sizeof(NumType) - 1;

    for (register int i = 0; i < j; i++, j--) {
        uint8_t tmp = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = tmp;
    }
}

} // namespace utility

#endif // _utility_ValueCodec_hpp_
