/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
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

#ifndef _btr_ValueCodec_hpp_
#define _btr_ValueCodec_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/buff.hpp"
#include "utility/misc.hpp"

namespace btr
{

#define ARRAY(...) (const uint8_t[]) { __VA_ARGS__ }

/**
 * The class converts between raw bytes and C++ types.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
class ValueCodec
{
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
  static uint8_t mask(uint8_t bits);

  /**
   * Compose an integer of type T from a variable-length bit pattern.
   *
   * 7 bits of each byte are used to reconstruct the number. If a byte of
   * an integer has high bit set, more bits should be read from the next byte
   * to reconstruct the target number. If the high bit is clear, this byte has
   * the last 7 bits.
   *
   * @param buff - the data buffer
   * @param val - target value
   * @return the value from Result enum
   */
  static int varInt7Bits(Buff* buff, uint8_t* val);
  static int varInt7Bits(Buff* buff, uint16_t* val);

  /**
   * Compose an integer of type T from the bits of the buffer.
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
   * @param map_size - the size of bit_map
   * @param move_ptr - move buff's read pointer by N bytes
   */
  static void varIntNBits(
      Buff* buff, uint8_t* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr = true);
  static void varIntNBits(
      Buff* buff, uint16_t* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr = true);

  /**
   * Extract an integer of type T using only n bytes from the buffer.
   *
   * IMPORTANT: This function doesn't check for buffer and source/target
   * sizes
   *
   * @param buff - data container
   * @param val - target value
   * @parma bytes - the number of bytes that the integer occupies
   * @param msb - the data is in most-significant byte order 
   */
  static void fixedInt(const uint8_t* buff, uint8_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, uint16_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, uint32_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, uint64_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, int16_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, int32_t* val, uint32_t bytes, bool msb = true);
  static void fixedInt(const uint8_t* buff, int64_t* val, uint32_t bytes, bool msb = true);

  /**
   * Check buffer and source/target sizes, then call fixedInt()
   *
   * @param buff - data container
   * @param val - target value
   * @parma bytes - the number of bytes that the integer occupies
   * @param msb - the data is in most-significant byte order 
   * @return the values returned by check()
   */
  static int fixedInt(Buff* buff, uint8_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, uint16_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, uint32_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, uint64_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, int16_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, int32_t* val, uint32_t bytes, bool msb = true);
  static int fixedInt(Buff* buff, int64_t* val, uint32_t bytes, bool msb = true);

  /**
   * Encode a fixed-size integer.
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  static void fixedInt(Buff* buff, uint8_t val, bool msb = true);
  static void fixedInt(Buff* buff, uint16_t val, bool msb = true);
  static void fixedInt(Buff* buff, uint32_t val, bool msb = true);
  static void fixedInt(Buff* buff, uint64_t val, bool msb = true);
  static void fixedInt(Buff* buff, int8_t val, bool msb = true);
  static void fixedInt(Buff* buff, int16_t val, bool msb = true);
  static void fixedInt(Buff* buff, int32_t val, bool msb = true);
  static void fixedInt(Buff* buff, int64_t val, bool msb = true);

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
   * Encode an floating-point number as an integer by shifting decimal point to the right.
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename T, typename FloatType>
  static void encodeShiftf(Buff* buff, FloatType val, uint8_t dec_places, bool msb = true);

  /**
   * Encode an integer and fractional parts of a floating-point number into two integers
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename InType, typename OutType>
  static void encodeModf(Buff* buff, InType val, uint8_t dec_places, bool msb = true);

private:

  /**
   * Internal implementation.
   */
  template<typename T>
  static int varInt7BitsImpl(Buff* buff, T* val);

  template<typename T>
  static void varIntNBitsImpl(
      Buff* buff, T* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr);

  template<typename T>
  static void fixedIntImpl(const uint8_t* buff, T* val, uint32_t bytes, bool msb);

  template<typename T>
  static int fixedIntImpl(Buff* buff, T* val, uint32_t bytes, bool msb);

  template<typename T>
  static void fixedIntImpl(Buff* buff, T val, bool msb);

  /**
   * Convert a value between MSB and LSB host order.
   *
   * @param val - the numeric value
   */
  template<typename T>
  static void swap(T* val);

}; // class ValueCodec

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

template<typename T, typename FloatType>
inline void ValueCodec::encodeShiftf(Buff* buff, FloatType val, uint8_t dec_places, bool msb)
{
  T output = Misc::shiftfint(val, dec_places);
  fixedInt(buff, output, msb);
}

template<typename InType, typename OutType>
inline void ValueCodec::encodeModf(Buff* buff, InType val, uint8_t dec_places, bool msb)
{
  OutType ipart = 0;
  OutType fpart = 0;
  Misc::modfint(val, &ipart, &fpart, dec_places);
  fixedInt(buff, ipart, msb);
  fixedInt(buff, fpart, msb);
}

} // namespace btr

#endif // _btr_ValueCodec_hpp_
