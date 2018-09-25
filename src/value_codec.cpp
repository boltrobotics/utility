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

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/value_codec.hpp"  // class implemented

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

uint8_t ValueCodec::mask(uint8_t bits)
{
  // Define a mapping from a number of bits (array index) to the bit pattern.
  //
  static const uint8_t mask[] = {
    0x0, 0x1, 0x3, 0x7, 0xF, 0x1F , 0x3F, 0x7F, 0xFF };
  return mask[bits];
}

int ValueCodec::varInt7Bits(Buff* buff, uint8_t* val)
{
  return varInt7BitsImpl(buff, val);
}

int ValueCodec::varInt7Bits(Buff* buff, uint16_t* val)
{
  return varInt7BitsImpl(buff, val);
}

void ValueCodec::varIntNBits(
    Buff* buff, uint8_t* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr)
{
  varIntNBitsImpl(buff, val, bit_map, map_size, move_ptr);
}

void ValueCodec::varIntNBits(
    Buff* buff, uint16_t* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr)
{
  varIntNBitsImpl(buff, val, bit_map, map_size, move_ptr);
}

void ValueCodec::fixedInt(const uint8_t* buff, uint8_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, uint16_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, uint32_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, uint64_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, int16_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, int32_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

void ValueCodec::fixedInt(const uint8_t* buff, int64_t* val, uint32_t bytes, bool msb)
{
  fixedIntImpl(buff, val, bytes, msb);
}

// Buff-related fixedInt

int ValueCodec::fixedInt(Buff* buff, uint8_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, uint16_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, uint32_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, uint64_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, int16_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, int32_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

int ValueCodec::fixedInt(Buff* buff, int64_t* val, uint32_t bytes, bool msb)
{
  return fixedIntImpl(buff, val, bytes, msb);
}

// Encode integer values

void ValueCodec::fixedInt(Buff* buff, uint8_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, uint16_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, uint32_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, uint64_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, int8_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, int16_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, int32_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

void ValueCodec::fixedInt(Buff* buff, int64_t val, bool msb)
{
  fixedIntImpl(buff, val, msb);
}

int ValueCodec::check(Buff* buff, uint32_t target_size, uint32_t source_size)
{
  int success = SUCCESS;

  if (buff->available() < source_size) {
    success = SMALL_BUFF;
  } else if (source_size > target_size) {
    success = SMALL_VALUE;
  }
  return success;
}

bool ValueCodec::isLittleEndian()
{
  int16_t val = 0x0001;
  char* ptr = (char*) &val;
  return (ptr[0] == 1);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
int ValueCodec::varInt7BitsImpl(Buff* buff, T* val)
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

template<typename T>
void ValueCodec::varIntNBitsImpl(
    Buff* buff, T* val, const uint8_t* bit_map, uint32_t map_size, bool move_ptr)
{
  *val = 0;

  for (uint32_t i = 0; i < map_size; i++) {
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
    buff->advanceReadPtr(map_size);
  }
}

template<typename T>
void ValueCodec::fixedIntImpl(const uint8_t* buff, T* val, uint32_t bytes, bool msb)
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
int ValueCodec::fixedIntImpl(Buff* buff, T* val, uint32_t bytes, bool msb)
{
  int success = check(buff, sizeof(T), bytes);

  if (success == SUCCESS) {
    fixedIntImpl(buff->read_ptr(), val, bytes, msb);
    buff->advanceReadPtr(bytes);
  }
  return success;
}

template<typename T>
void ValueCodec::fixedIntImpl(Buff* buff, T val, bool msb)
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
