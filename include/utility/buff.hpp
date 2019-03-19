// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

/** @file */

#ifndef _btr_Buff_hpp_
#define _btr_Buff_hpp_

// SYSTEM INCLUDES
#include <cstddef>
#include <cstring>
#include <cstdlib>

namespace btr
{

/** Define array of uint8_t values. */
#define ARRAY(...) (const uint8_t[]) { __VA_ARGS__ }
/** Define array of uint8_t values. */
#define ARRAYU8(...) (const uint8_t[]) { __VA_ARGS__ }
/** Define array of int8_t values. */
#define ARRAYS8(...) (const int8_t[]) { __VA_ARGS__ }
/** Define array of uint16_t values. */
#define ARRAYU16(...) (const uint16_t[]) { __VA_ARGS__ }
/** Define array of int16_t values. */
#define ARRAYS16(...) (const int16_t[]) { __VA_ARGS__ }
/** Define array of uint32_t values. */
#define ARRAYU32(...) (const uint32_t[]) { __VA_ARGS__ }
/** Define array of int32_t values. */
#define ARRAYS32(...) (const int32_t[]) { __VA_ARGS__ }
/** Define array of uint64_t values. */
#define ARRAYU64(...) (const uint64_t[]) { __VA_ARGS__ }
/** Define array of int64_t values. */
#define ARRAYS64(...) (const int64_t[]) { __VA_ARGS__ }

/**
 * The class provides raw data container and access methods to that data.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
class Buff
{
public:

  enum MOD {
    NO_MOD  = 0,
    EXTEND  = 1,
    RESERVE = 2,
    MINIMAL = 4,
    MOD_ALL = EXTEND | RESERVE | MINIMAL
  };

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
  uint8_t* data();
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
   * @param buff_mod - if RESERVE bit is set, allocate more memory if current capacity is
   *  insufficient for the requested buffer size. Otherwise, return false to indicate that the
   *  new requested size exceeds allocated and available buffer space. If MINIMAL bit is set,
   *    execute case #2 described above. Otherwise, When the parameter is false, execute case #1
   *    above.
   * @return true if operation was successful, false otherwise
   */
  bool extend(uint32_t bytes, int buff_mod);

  /**
   * Set new size on the buffer. The size must be less than or equal to capacity.
   *
   * @param new_size - the new size in bytes
   * @param buff_mod - if RESERVE bit is set, allocate more memory if current capacity is
   *  insufficient
   * @return true if operation was successful, false when size is greater than capacity
   */
  bool resize(uint32_t new_size, int buff_mod = RESERVE);

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
   * Read a single byte.
   *
   * @param val - the storage for the value
   * @param advance - the flag indicating to advance read position
   * @return false if no data is avalable
   */
  template<typename T>
  bool read(T* val, bool advance);

  /**
   * Read a chunk.
   *
   * @param vals - the storage for the data
   * @param vals_size - the number of data bytes
   * @param advance - the flag indicating to advance read position
   * @return false if not enough data is avalable, true otherwise
   */
  template<typename T>
  bool read(T* vals, uint32_t vals_size, bool advance);

  /**
   * Single-value version of write() array function.
   */
  template<typename T>
  bool write(const T val, int buff_mod = MOD_ALL);

  /**
   * Write array of bytes to the buffer. To write a single value, pass ARRAY(value)
   *
   * @param vals - the data to write
   * @param buff_mod -
   *  if RESERVE bit is set, allocate more memory if current capacity is insufficient.
   *  If EXTEND bit is set, extend the size if current size is not sufficient.
   *  For MINIMAL bit @see extend()
   * @return true if the data was written, false otherwise. The failure could
   *  be caused by inability to allocate new memory if extend parameter is true
   *  or insufficient memory if extend parameter is false.
   */
  template<typename T, uint32_t COUNT>
  bool write(const T (&vals)[COUNT], int buff_mod = MOD_ALL);

private:

// ATTRIBUTES

  uint32_t capacity_;
  uint32_t size_;
  uint8_t* data_;
  uint8_t* read_ptr_;
  uint8_t* write_ptr_;

}; // class Buff

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline Buff::Buff(uint32_t capacity)
  :
  capacity_(capacity),
  size_(capacity),
  data_(new uint8_t[capacity_]),
  read_ptr_(data_),
  write_ptr_(data_)
{
}

inline Buff::~Buff()
{
  delete [] data_;
  data_ = nullptr;
  read_ptr_ = nullptr;
  write_ptr_ = nullptr;
}

//============================================= ACCESS =============================================

inline const uint8_t* Buff::data() const
{
  return data_;
}

inline uint8_t* Buff::data()
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

//============================================= OPERATIONS =========================================

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

inline bool Buff::extend(uint32_t bytes, int buff_mod)
{
  bool success = true;

  if ((buff_mod & MINIMAL) == MINIMAL) {
    uint32_t bytes_remain = remaining();

    if (bytes > bytes_remain) {
      success = resize(size_ + (bytes - bytes_remain), buff_mod);
    }
  } else {
    success = resize(size_ + bytes, buff_mod);
  }
  return success;
}

inline bool Buff::resize(uint32_t new_size, int buff_mod)
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
    if ((buff_mod & RESERVE) == RESERVE) {
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

inline void Buff::advanceReadPtr(uint32_t bytes)
{
  read_ptr() += bytes;
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

template<typename T>
inline bool Buff::read(T* vals, uint32_t vals_size, bool advance)
{
  if (available() >= vals_size) {
    memcpy(vals, read_ptr(), vals_size);

    if (advance) {
      advanceReadPtr(vals_size);
    }
    return true;
  } else {
    return false;
  }
}

template<typename T>
inline bool Buff::write(T val, int buff_mod)
{
  T vals[] = { val };
  return write(vals, buff_mod);
}

template<typename T, uint32_t COUNT>
inline bool Buff::write(const T (&vals)[COUNT], int buff_mod)
{
  bool success = true;
  uint32_t bytes_target = sizeof(T) * COUNT;
  uint32_t bytes_remain = remaining();

  if (bytes_remain < bytes_target) {
    bytes_remain = shift();

    if (bytes_remain < bytes_target) {
      success = false;
    } else {
      success = true; // Some usable space is gained
    }
  }

  if (!success && ((buff_mod & EXTEND) == EXTEND)) {
    success = extend(bytes_target, buff_mod);
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
