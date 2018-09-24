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

#ifndef _btr_Buff_hpp_
#define _btr_Buff_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

/**
 * The class provides raw data container and access methods to that data.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
class Buff
{
public:

// LIFECYCLE

  /**
   * Create a buffer object with the requested,  dynamically allocated memory.
   *
   * @param capacity - the initial buffer's capacity
   */
  Buff(uint32_t capacity = 128);

  /**
   * Dtor.
   */
  ~Buff();

// ACCESS

  const uint8_t* data() const;
  const uint8_t* end() const;
  const uint8_t* read_ptr() const;
  uint8_t*& read_ptr();
  uint8_t*& write_ptr();

  /**
   * @return the allocated memory size
   */
  uint32_t capacity() const;

  /**
   * @return the allocated memory size
   */
  uint32_t size() const;

  /**
   * @return the number of bytes between position 0 and read pointer.
   */
  uint32_t consumed() const;

  /**
   * @return the content length
   */
  uint32_t available() const;

  /**
   * @return the bytes in buffer available for writing
   */
  uint32_t remaining() const;

// OPERATIONS

  /**
   * Nullify allocated memory and set read/write pointers to position 0.
   */
  void reset();

  /**
   * Shift available data to the front of the buffer's storage so as read_ptr
   * points to buffer's start, and write_ptr to read_ptr + available bytes. 
   *
   * @return the remaining() bytes available for writing
   */
  uint32_t shift();

  /**
   * Extend the buffer by preserving the existing content and read/write
   * positions.
   *
   * @param bytes - two different cases:
   *  1. The number of bytes to add to the existing buffer, regardless of current free bytes
   *  2. The number of free bytes that the client requires, the function determines if it needs to
   *    extend the buffer at all and by how much using delta of required and remaining() bytes
   * @param minimial - if true, execute case #2 described above. Otherwise, When the parameter is
   *  false, execute case #1 above.
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient for the
   *  requested buffer size. Otherwise, return false to indicate that the new requested size
   *  exceeds allocated and available buffer space.
   * @return true if operation was successful, false otherwise
   */
  bool extend(uint32_t bytes, bool minimal = true, bool reserve_mem = true);

  /**
   * Set new size on the buffer. The size must be less than or equal to capacity.
   *
   * @param new_size - the new size in bytes
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient
   * @return true if operation was successful, false when size is greater than
   *  capacity
   */
  bool resize(uint32_t new_size, bool reserve_mem = true);

  /**
   * Reserve memory for the buffer. The data is preserved only if the new capacity is
   * greater than or equal to the previous size.
   *
   * IMPORTANT: This call uses expensive dynamic-memory manipulation functions. If this is
   * a concern, allocate the maximum required number of bytes when creating the buffer, and
   * then use resize() function instead.
   *
   * @param new_capacity - the new capacity in bytes
   * @return true if operation was successful, false otherwise
   */
  bool reserve(uint32_t new_capacity);

  /**
   * Advance read position.
   *
   * @param bytes - the number of bytes to advance read pointer
   */
  void advanceReadPtr(uint32_t bytes);

  /**
   * Read a single byte of content.
   *
   * @param v - the storage for the value
   * @param advance - the flag indicating to advance read position
   * @return false if no data is avalable
   */
  bool read(uint8_t* v, bool advance);

  /**
   * Read a chunk of content.
   *
   * @param v - the storage for the data
   * @param N - the number of data bytes
   * @param advance - the flag indicating to advance read position
   * @return false if not enough data is avalable, true otherwise
   */
  bool read(uint8_t* vals, uint32_t N, bool advance);

  /**
   * Write a scalar value to the buffer.
   *
   * @param data - the data to write
   * @parma extend_size - the flag whether to extend size if current size is not sufficient
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient
   * @return true if the data was written, false otherwise. The failure could
   *  be caused by inability to allocate new memory if extend parameter is true
   *  or insufficient memory if extend parameter is false.
   */
  bool write(uint8_t val, bool extend_size = true, bool reserve_mem = true);
  bool write(uint16_t val, bool extend_size = true, bool reserve_mem = true);
  bool write(int16_t val, bool extend_size = true, bool reserve_mem = true);

  /**
   * Write array of bytes to the buffer.
   *
   * @param data - the data to write
   * @parma extend_size - the flag whether to extend size if current size is not sufficient
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient
   * @return true if the data was written, false otherwise. The failure could
   *  be caused by inability to allocate new memory if extend parameter is true
   *  or insufficient memory if extend parameter is false.
   */
  bool write(const uint8_t* vals, uint32_t N, bool extend_size = true, bool reserve_mem = true);

private:

// OPERATIONS

  /**
   * Implementation.
   */

  template<typename T>
  bool readImpl(T* val, bool advance);

  template<typename T>
  bool readImpl(T* vals, uint32_t N, bool advance);

  template<typename T>
  bool writeImpl(T val, bool extend_size, bool reserve_mem);

  template<typename T>
  bool writeImpl(const T* vals, uint32_t N, bool extend_size, bool reserve_mem);

// ATTRIBUTES

  uint32_t capacity_;
  uint32_t size_;
  uint8_t* data_;
  uint8_t* read_ptr_;
  uint8_t* write_ptr_;

}; // class Buff

} // namespace btr

#endif // _btr_Buff_hpp_
