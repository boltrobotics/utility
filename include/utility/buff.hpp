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

#ifndef _btr_Buff_hpp_
#define _btr_Buff_hpp_

// SYSTEM INCLUDES
#include <stdlib.h>
#include <string.h>
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
   * @param bytes - the bytes to add to the existing buffer beyond what
   *  is remaining() UNLESS minimal parameter is set to true
   * @param minimial - if true, the flag treat bytes parameter as a target amount of free
   *  space that the caller requires. The code will allocate only the difference between
   *  requested bytes and the remaining() bytes. When the parameter is false, the bytes
   *  parameter is treated as an additional space (beyond what is remaining()) that the
   *  caller requires.
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient
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

  /**
   * Read a single byte of content.
   *
   * @param v - the storage for the value
   * @param advance - the flag indicating to advance read position
   * @return false if no data is avalable
   */
  template<typename T>
  bool read(T* v, bool advance = true);

  /**
   * Read a chunk of content.
   *
   * @param v - the storage for the data
   * @return false if not enough data is avalable, true otherwise
   */
  template<typename T, uint32_t N>
  bool readChunk(T (&vals)[N], bool advance = true);

  /**
   * Advance read position.
   *
   * @param bytes - the number of bytes to advance read pointer
   */
  void advanceReadPtr(uint32_t bytes);

  /**
   * Write scalar value to the buffer.
   *
   * @param data - the data to write
   * @parma extend_size - the flag whether to extend size if current size is not sufficient
   * @param reserve_mem - if true, allocate more memory if current capacity is insufficient
   * @return true if the data was written, false otherwise. The failure could
   *  be caused by inability to allocate new memory if extend parameter is true
   *  or insufficient memory if extend parameter is false.
   */
  template<typename T>
  bool write(T val, bool extend_size = true, bool reserve_mem = true);

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
  template<typename T, uint32_t N>
  bool writeChunk(const T (&vals)[N], bool extend_size = true, bool reserve_mem = true);

private:

// ATTRIBUTES

  uint32_t capacity_;
  uint32_t size_;
  uint8_t* data_;
  uint8_t* read_ptr_;
  uint8_t* write_ptr_;

}; // class Buff

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

inline Buff::Buff(uint32_t capacity)
: capacity_(capacity),
  size_(capacity),
  data_((uint8_t*) malloc(capacity_)),
  read_ptr_(data_),
  write_ptr_(data_)
{
}

inline Buff::~Buff()
{
  free(data_);
  data_ = nullptr;
  read_ptr_ = nullptr;
  write_ptr_ = nullptr;
}

inline const uint8_t* Buff::data() const
{
  return data_;
}

inline const uint8_t* Buff::end() const
{
  return (data_ + size_);
}

inline const uint8_t* Buff::read_ptr() const
{
  return read_ptr_;
}

inline uint8_t*& Buff::read_ptr()
{
  return read_ptr_;
}

inline uint8_t*& Buff::write_ptr()
{
  return write_ptr_;
}

inline void Buff::reset()
{
  memset(data_, '\0', size_);
  read_ptr_ = data_;
  write_ptr_ = data_;
}

inline uint32_t Buff::shift()
{
  // Check if at least one byte was consumed so as to shift the data down to
  // the begining of the buffer.
  //
  uint32_t bytes_consumed = consumed();

  if (bytes_consumed > 0) {
    uint32_t bytes_avail = available();

    if (bytes_consumed >= bytes_avail) {
      memcpy(data_, read_ptr(), bytes_avail);
    } else {
      uint8_t* next = data_;

      while (available() > 0) {
        *next = *read_ptr();
        next++;
        read_ptr()++;
      }
    }
    read_ptr() = data_;
    write_ptr() = data_ + bytes_avail;
  }
  return remaining();
}

inline bool Buff::extend(uint32_t bytes, bool minimal, bool reserve_mem)
{
  bool success = true;

  if (minimal) {
    uint32_t bytes_remain = remaining();

    if (bytes > bytes_remain) {
      success = resize(size_ + (bytes - bytes_remain), reserve_mem);
    }
  } else {
    success = resize(size_ + bytes, reserve_mem);
  }
  return success;
}

inline bool Buff::resize(uint32_t new_size, bool reserve_mem)
{
  bool result = true;

  if (new_size <= capacity()) {
    size_ = new_size;

    if (write_ptr() > end()) {
      write_ptr() = data_ + size_;
    }
    if (read_ptr() > end()) {
      read_ptr() = data_ + size_;
    }
  } else {
    if (reserve_mem) {
      result = reserve(new_size);

      if (result) {
        size_ = new_size;
      }
    } else {
      result = false;
    }
  }
  return result;
}

inline bool Buff::reserve(uint32_t new_capacity)
{
  if (new_capacity == 0) {
    return false;
  }

  uint32_t read_offset = read_ptr_ - data_;
  uint32_t write_offset = write_ptr_ - data_;
  uint8_t* data = (uint8_t*) realloc(data_, new_capacity * sizeof(uint8_t));

  if (data != nullptr) {
    capacity_ = new_capacity;
    size_ = (size_ <= capacity_ ? size_ : capacity_);

    data_ = data;
    const uint8_t* end = (data_ + size_);

    if ((data_ + read_offset) <= end) {
      read_ptr_ = data_ + read_offset;
    } else {
      read_ptr_ = data_;
    }
    if ((data_ + write_offset) <= end) {
      write_ptr_ = data_ + write_offset;
    } else {
      write_ptr_ = data_;
    }
    memset(write_ptr_, 'y', remaining());
    return true;
  } else {
    return false;
  }
}

inline uint32_t Buff::capacity() const
{
  return capacity_;
}

inline uint32_t Buff::size() const
{
  return size_;
}

inline uint32_t Buff::consumed() const
{
  return (read_ptr() - data());
}

inline uint32_t Buff::available() const
{
  return (write_ptr_ - read_ptr_);
}

inline uint32_t Buff::remaining() const
{
  return ((data_ + size_) - write_ptr_);
}

template<typename T>
inline bool Buff::read(T* v, bool advance)
{
  if (available() > 0) {
    *v = *read_ptr();

    if (advance) {
      advanceReadPtr(1);
    }
    return true;
  } else {
    return false;
  }
}

template<typename T, uint32_t N>
inline bool Buff::readChunk(T (&vals)[N], bool advance)
{
  if (available() >= N) {
    memcpy(vals, read_ptr(), N);

    if (advance) {
      advanceReadPtr(N);
    }
    return true;
  } else {
    return false;
  }
}

inline void Buff::advanceReadPtr(uint32_t bytes)
{
  read_ptr() += bytes;
}

template<typename T>
inline bool Buff::write(T val, bool extend_size, bool reserve_mem)
{
  T vals[] = { val };
  return writeChunk(vals, extend_size, reserve_mem);
}

template<typename T, uint32_t N>
inline bool Buff::writeChunk(const T (&vals)[N], bool extend_size, bool reserve_mem)
{
  bool success = true;
  uint32_t bytes_target = sizeof(T) * N;
  uint32_t bytes_remain = remaining();

  if (bytes_remain < bytes_target) {
    bytes_remain = shift();

    if (bytes_remain < bytes_target) {
      success = false;
    } else {
      success = true; // Some usable space is gained
    }
  }

  if (!success && extend_size) {
    success = extend(bytes_target, true, reserve_mem);
  }
  if (success) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(vals);
    memcpy(write_ptr(), bytes, bytes_target);
    write_ptr() += bytes_target;
  }

  return success;
}

} // namespace btr

#endif // _btr_Buff_hpp_
