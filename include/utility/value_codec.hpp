// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

/** @file */

#ifndef _btr_ValueCodec_hpp_
#define _btr_ValueCodec_hpp_

// PROJECT INCLUDES
#include "utility/defines.hpp"
#include "utility/buff.hpp"
#include "utility/misc.hpp"

// SYSTEM INCLUDES
#if BTR_FLOAT_ENABLED > 0
#include <math.h>
#endif
#include <errno.h>

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
  static int decodeVarInt7Bits(Buff* buff, T* val);

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
  static void decodeVarIntNBits(Buff* buff, T* val, const uint8_t (&bit_map)[N], bool move_ptr);

  /**
   * Extract an integer of type T using only n bytes from the buffer.
   *
   * IMPORTANT: This function doesn't check for buffer and source/target
   * sizes
   *
   * @param buff - data container
   * @param val - target value
   * @param bytes - the number of bytes that the integer occupies
   * @param msb - the data is in most-significant byte order
   */
  template<typename T>
  static void decodeFixedInt(const uint8_t* buff, T* val, uint32_t bytes, bool msb);

  /**
   * Check buffer and source/target sizes, then call toFixedInt()
   *
   * @param buff - buffer to store encoded value
   * @param val - target value
   * @param bytes - the number of bytes that the integer occupies
   * @param msb - the data is in most-significant byte order 
   * @return the values returned by check()
   */
  template<typename T>
  static int decodeFixedInt(Buff* buff, T* val, uint32_t bytes, bool msb);

  /**
   * Encode a fixed-size integer into a raw buffer.
   *
   * @param buff - buffer object to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename T>
  static void encodeFixedInt(Buff* buff, T val, bool msb);

  /**
   * Encode a fixed-size integer into a raw buffer.
   *
   * @param buff - character array to store encoded value
   * @param val - the value to encode
   * @param msb - if true, encode in MSB order, otherwise in LSB
   */
  template<typename T>
  static void encodeFixedInt(uint8_t* buff, T val, bool msb);

#if BTR_FLOAT_ENABLED > 0

  /**
   * Encode a floating-point number as an integer by shifting decimal point to the right.
   *
   * @param buff - buffer object to store encoded value
   * @param val_bytes - number of bytes that the value occupies
   * @param val - value to encode
   * @param dec_places - number of digits after decimal point
   * @param msb - if true, encode in MSB order, otherwise in LSB
   * @return 0 on success, -1 on failure
   */
  static int encodeFloatToInt(
      Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb);

  /**
   * Encode a floating-point number as an integer by shifting decimal point to the right.
   *
   * @param buff - buffer to store encoded value
   * @param val_bytes - number of bytes that the value occupies
   * @param val - value to encode
   * @param dec_places - number of digits after decimal point
   * @param msb - if true, encode in MSB order, otherwise in LSB
   * @return 0 on success, -1 on failure
   */
  static int encodeFloatToInt(
      uint8_t* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb);

  /**
   * Encode an integer and fractional parts of a floating-point number into two integers.
   *
   * @param buff - the buffer to store encoded value
   * @param val_bytes - bytes in buffer
   * @param val - the value to encode
   * @param dec_places - number of digits after decimal point
   * @param msb - if true, encode in MSB order, otherwise in LSB
   * @return -1 on error, 0 otherwise
   */
  static int encodeFloatToIntParts(
      Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb);

  /**
   * Decode an integer into a floating-point number by shifting decimal point to the left.
   *
   * @param buff - buffer to read an integer from
   * @param bytes - bytes in buffer
   * @param val - value to store the result in
   * @param dec_places - number of digits after decimal point
   * @param msb - if true, encode in MSB order, otherwise in LSB
   * @return -1 on error, 0 otherwise
   */
  template<typename T, typename FloatType>
  static int decodeIntToFloat(
      const uint8_t* buff, uint8_t bytes, FloatType* val, uint8_t dec_places, bool msb);

  /**
   * Decode two consecutive integers into integer and fractional parts of a floating-point number.
   * Combine integer parts into a floating point number.
   *
   * @param buff - buffer to retrieve the integers from
   * @param val - value to store decoding result
   * @param dec_places - number of digits after decimal point
   * @param msb - if true, decode in MSB order, otherwise in LSB
   */
  template<typename T, typename FloatType>
  static int decodeIntPartsToFloat(Buff* buff, FloatType* val, uint8_t dec_places, bool msb);

#endif // BTR_FLOAT_ENABLED > 0

  /**
   * Check if the provided data can be converted into requested number.
   *
   * @param buff - the data container
   * @param target_size - byte-size of target number (e.g. uint32_t == 4 bytes)
   * @param source_size - amount of bytes to use for conversion to the target number
   * @return 0 on success, otherwise -1 on error, errno will be set to EDOM or ERANGE
   */
  static int check(Buff* buff, uint32_t target_size, uint32_t source_size);

  /**
   * @return true if this host is little endian
   */
  static bool isLittleEndian();

  /**
   * Convert a value between MSB and LSB host order considering the target and host order.
   *
   * @param val - numeric value
   * @param msb - target order
   */
  template<typename T>
  static void swap(T* val, bool msb);

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
inline int ValueCodec::decodeVarInt7Bits(Buff* buff, T* val)
{
  int rc = -1;
  int err = ERANGE;

  uint8_t bits = 7;
  uint8_t bit_map[] = { bits };
  int8_t bits_remain = sizeof(T) * 8;

  for (; bits_remain >= 0; bits_remain -= bits) {
    if (buff->available() == 0) { 
      err = ERANGE;
      break;
    }

    if (bits_remain < bits) {
      bit_map[0] = bits_remain;
    }

    uint8_t byte = *buff->read_ptr();
    uint8_t v = 0;
    decodeVarIntNBits(buff, &v, bit_map, true);
    *val = ((*val << bit_map[0]) | v);

    if ((byte & 0x80) == 0) {
      rc = 0;
      break;
    }
  }

  if (rc != 0) {
    errno = err;
  }
  return rc;
}

template<typename T, uint32_t N>
inline void ValueCodec::decodeVarIntNBits(
    Buff* buff, T* val, const uint8_t (&bit_map)[N], bool move_ptr)
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
inline void ValueCodec::decodeFixedInt(const uint8_t* buff, T* val, uint32_t bytes, bool msb)
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
inline int ValueCodec::decodeFixedInt(Buff* buff, T* val, uint32_t bytes, bool msb)
{
  int rc = check(buff, sizeof(T), bytes);

  if (0 == rc) {
    decodeFixedInt(buff->read_ptr(), val, bytes, msb);
    buff->advanceReadPtr(bytes);
  }
  return rc;
}

template<typename T>
inline void ValueCodec::encodeFixedInt(Buff* buff, T val, bool msb)
{
  swap(&val, msb);
  buff->write(val);
}

template<typename T>
inline void ValueCodec::encodeFixedInt(uint8_t* buff, T val, bool msb)
{
  swap(&val, msb);
  const uint8_t* val_bytes = reinterpret_cast<const uint8_t*>(&val);
  memcpy(buff, val_bytes, sizeof(T));
}

#if BTR_FLOAT_ENABLED > 0

inline int ValueCodec::encodeFloatToInt(
    Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  if (0 == encodeFloatToInt(buff->write_ptr(), val_bytes, val, dec_places, msb)) {
    buff->write_ptr() += val_bytes;
    return 0;
  }
  return -1;
}

inline int ValueCodec::encodeFloatToInt(
    uint8_t* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  if (sizeof(uint8_t) == val_bytes) {
    uint8_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint16_t) == val_bytes) {
    uint16_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint32_t) == val_bytes) {
    uint32_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint64_t) == val_bytes) {
    uint64_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else {
    errno = EDOM;
    return -1;
  }
  return 0;
}

inline int ValueCodec::encodeFloatToIntParts(
    Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  double ipart_tmp = 0;
  double fpart_tmp = modf(val, &ipart_tmp);

  if (sizeof(uint8_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint8_t>(ipart_tmp), msb);
  } else if (sizeof(uint16_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint16_t>(ipart_tmp), msb);
  } else if (sizeof(uint32_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint32_t>(ipart_tmp), msb);
  } else if (sizeof(uint64_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint64_t>(ipart_tmp), msb);
  } else {
    errno = EDOM;
    return -1;
  }
  return encodeFloatToInt(buff, val_bytes, fpart_tmp, dec_places, msb);
}

template<typename T, typename FloatType>
inline int ValueCodec::decodeIntToFloat(
    const uint8_t* buff, uint8_t bytes, FloatType* val, uint8_t dec_places, bool msb)
{
  T int_val;
  decodeFixedInt(buff, &int_val, bytes, msb);
  *val = int_val / pow(10, dec_places);
  return 0;
}

template<typename T, typename FloatType>
inline int ValueCodec::decodeIntPartsToFloat(
    Buff* buff, FloatType* val, uint8_t dec_places, bool msb)
{
  T ipart;
  if (-1 == decodeFixedInt(buff, &ipart, sizeof(T), msb)) {
    return -1;
  }

  T fpart;
  if (-1 == decodeFixedInt(buff, &fpart, sizeof(T), msb)) {
    return -1;
  }

  // TODO finish off
  (void)dec_places;
  *val = ipart + fpart;
  return -1;
}

#endif // BTR_FLOAT_ENABLED > 0

inline int ValueCodec::check(Buff* buff, uint32_t target_size, uint32_t source_size)
{
  int rc = 0;

  if (buff->available() < source_size) {
    rc = -1;
    errno = ERANGE;
  } else if (source_size > target_size) {
    rc = -1;
    errno = ERANGE;
  }
  return rc;
}

inline bool ValueCodec::isLittleEndian()
{
  int16_t val = 0x0001;
  char* ptr = (char*) &val;
  return (ptr[0] == 1);
}

template<typename T>
inline void ValueCodec::swap(T* val, bool msb)
{
  if (isLittleEndian()) {
    if (msb) {
      swap(val);
    }
  } else {
    if (!msb) {
      swap(val);
    }
  }
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
