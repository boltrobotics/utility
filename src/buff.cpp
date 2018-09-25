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
#include <stdlib.h>
#include <string.h>

// PROJECT INCLUDES
#include "utility/buff.hpp"  // class implemented

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Buff::Buff(uint32_t capacity)
  :
  capacity_(capacity),
  size_(capacity),
  data_((uint8_t*) malloc(capacity_)),
  read_ptr_(data_),
  write_ptr_(data_)
{
}

Buff::~Buff()
{
  free(data_);
  data_ = nullptr;
  read_ptr_ = nullptr;
  write_ptr_ = nullptr;
}

//============================================= ACCESS =============================================

const uint8_t* Buff::data() const
{
  return data_;
}

const uint8_t* Buff::end() const
{
  return (data_ + size_);
}

const uint8_t* Buff::read_ptr() const
{
  return read_ptr_;
}

uint8_t*& Buff::read_ptr()
{
  return read_ptr_;
}

uint8_t*& Buff::write_ptr()
{
  return write_ptr_;
}

uint32_t Buff::capacity() const
{
  return capacity_;
}

uint32_t Buff::size() const
{
  return size_;
}

uint32_t Buff::consumed() const
{
  return (read_ptr() - data());
}

uint32_t Buff::available() const
{
  return (write_ptr_ - read_ptr_);
}

uint32_t Buff::remaining() const
{
  return ((data_ + size_) - write_ptr_);
}

//============================================= OPERATIONS =========================================

void Buff::reset()
{
  memset(data_, '\0', size_);
  read_ptr_ = data_;
  write_ptr_ = data_;
}

uint32_t Buff::shift()
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

bool Buff::extend(uint32_t bytes, bool minimal, bool reserve_mem)
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

bool Buff::resize(uint32_t new_size, bool reserve_mem)
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

bool Buff::reserve(uint32_t new_capacity)
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

void Buff::advanceReadPtr(uint32_t bytes)
{
  read_ptr() += bytes;
}

bool Buff::read(uint8_t* v, bool advance)
{
  return readImpl(v, advance);
}

bool Buff::read(uint8_t* vals, uint32_t N, bool advance)
{
  return readImpl(vals, N, advance);
}

bool Buff::write(uint8_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(uint16_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(uint32_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(uint64_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(int8_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(int16_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(int32_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(int64_t val, bool extend_size, bool reserve_mem)
{
  return writeImpl(val, extend_size, reserve_mem);
}

bool Buff::write(const uint8_t* vals, uint32_t N, bool extend_size, bool reserve_mem)
{
  return writeImpl(vals, N, extend_size, reserve_mem);
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
bool Buff::readImpl(T* v, bool advance)
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
bool Buff::readImpl(T* vals, uint32_t N, bool advance)
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

template<typename T>
bool Buff::writeImpl(T val, bool extend_size, bool reserve_mem)
{
  T vals[] = { val };
  return writeImpl(vals, 1, extend_size, reserve_mem);
}

template<typename T>
bool Buff::writeImpl(const T* vals, uint32_t N, bool extend_size, bool reserve_mem)
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
