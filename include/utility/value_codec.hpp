/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_ValueCodec_hpp_
#define _btr_ValueCodec_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/buff.hpp"
#include "utility/misc.hpp"

namespace btr
{

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
  template<typename T>
  static int varInt7Bits(Buff* buff, T* val);

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
   * @param move_ptr - move buff's read pointer by N bytes
   */
  template<typename T, uint32_t N>
  static void varIntNBits(Buff* buff, T* val, const uint8_t (&bit_map)[N], bool move_ptr);

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
  template<typename T>
  static void fixedInt(const uint8_t* buff, T* val, uint32_t bytes, bool msb);

  /**
   * Check buffer and source/target sizes, then call fixedInt()
   *
   * @param buff - data container
   * @param val - target value
   * @parma bytes - the number of bytes that the integer occupies
   * @param msb - the data is in most-significant byte order 
   * @return the values returned by check()
   */
  template<typename T>
  static int fixedInt(Buff* buff, T* val, uint32_t bytes, bool msb);

  /**
   * Encode a fixed-size integer.
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename T>
  static void fixedInt(Buff* buff, T val, bool msb);

  /**
   * Encode an floating-point number as an integer by shifting decimal point to the right.
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename T, typename FloatType>
  static void encodeShiftf(Buff* buff, FloatType val, uint8_t dec_places, bool msb);

  /**
   * Encode an integer and fractional parts of a floating-point number into two integers
   *
   * @param buff - the buffer to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename InType, typename OutType>
  static void encodeModf(Buff* buff, InType val, uint8_t dec_places, bool msb);

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
   * @param val - the numeric value
   */
  template<typename T>
  static void swap(T* val);

}; // class ValueCodec

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline uint8_t ValueCodec::mask(uint8_t bits)
{
  // Define a mapping from a number of bits (array index) to the bit pattern.
  //
  static const uint8_t mask[] = {
    0x0, 0x1, 0x3, 0x7, 0xF, 0x1F , 0x3F, 0x7F, 0xFF };
  return mask[bits];
}

template<typename T>
inline int ValueCodec::varInt7Bits(Buff* buff, T* val)
{
  int success = SMALL_VALUE;
  uint8_t bits = 7;
  uint8_t bit_map[] = { bits };
  int8_t bits_remain = sizeof(T) * 8;

  for (; bits_remain >= 0; bits_remain -= bits) {
    if (buff->available() == 0) { 
      success = SMALL_BUFF;
      break;
    }

    if (bits_remain < bits) {
      bit_map[0] = bits_remain;
    }

    uint8_t byte = *buff->read_ptr();
    uint8_t v = 0;
    varIntNBits(buff, &v, bit_map, true);
    *val = ((*val << bit_map[0]) | v);

    if ((byte & 0x80) == 0) {
      success = SUCCESS;
      break;
    }
  }

  return success;
}

template<typename T, uint32_t N>
inline void ValueCodec::varIntNBits(Buff* buff, T* val, const uint8_t (&bit_map)[N], bool move_ptr)
{
  *val = 0;

  for (uint32_t i = 0; i < N; i++) {
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
    uint8_t mask_val = mask(bits);
    uint8_t high_bit_set = (bit_map[i] >> 7) & 1;
    uint8_t v = (high_bit_set ? (c >> (8 - bits)) : c);
    *val = ((*val << bits) | (v & mask_val));
  }

  if (move_ptr) {
    buff->advanceReadPtr(N);
  }
}

template<typename T>
inline void ValueCodec::fixedInt(const uint8_t* buff, T* val, uint32_t bytes, bool msb)
{
  *val = 0;

  if (msb) {
    for (uint32_t i = 0; i < bytes; i++) {
      uint8_t c = buff[i];
      *val = ((*val << 8) | c);
    }
  } else {
    for (int32_t i = int32_t(bytes); --i >= 0; ) {
      uint8_t c = buff[i];
      *val = ((*val << 8) | c);
    }
  }
}

template<typename T>
inline int ValueCodec::fixedInt(Buff* buff, T* val, uint32_t bytes, bool msb)
{
  int success = check(buff, sizeof(T), bytes);

  if (success == SUCCESS) {
    fixedInt(buff->read_ptr(), val, bytes, msb);
    buff->advanceReadPtr(bytes);
  }
  return success;
}

template<typename T>
inline void ValueCodec::fixedInt(Buff* buff, T val, bool msb)
{
  if (isLittleEndian()) {
    if (msb) {
      swap(&val);
    }
  } else {
    if (!msb) {
      swap(&val);
    }
  }
  buff->write(val);
}

template<typename T, typename FloatType>
inline void ValueCodec::encodeShiftf(Buff* buff, FloatType val, uint8_t dec_places, bool msb)
{
  T output = 0;
  Misc::shiftfint(val, &output, dec_places);
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

inline int ValueCodec::check(Buff* buff, uint32_t target_size, uint32_t source_size)
{
  int success = SUCCESS;

  if (buff->available() < source_size) {
    success = SMALL_BUFF;
  } else if (source_size > target_size) {
    success = SMALL_VALUE;
  }
  return success;
}

inline bool ValueCodec::isLittleEndian()
{
  int16_t val = 0x0001;
  char* ptr = (char*) &val;
  return (ptr[0] == 1);
}

template<typename T>
inline void ValueCodec::swap(T* val)
{
  uint8_t* bytes = reinterpret_cast<uint8_t*>(val);
  int j = sizeof(T) - 1;

  for (int i = 0; i < j; i++, j--) {
    uint8_t tmp = bytes[i];
    bytes[i] = bytes[j];
    bytes[j] = tmp;
  }
}

} // namespace btr

#endif // _btr_ValueCodec_hpp_
