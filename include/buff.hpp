/* Copyright (C) 2017 Sergey Kapustin <boltrobotics.com>
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

#ifndef _utility_Buff_hpp_
#define _utility_Buff_hpp_

// SYSTEM INCLUDES
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

namespace utility {

/**
 * The class provides raw data container and access methods to that data.
 *
 * IMPORTANT: The class is shared by AVR and x86 plaforms. Keep it portable.
 */
class Buff {
public:

// LIFECYCLE

    /**
     * Ctor.
     *
     * @param size - the initial buffer size
     */
    Buff(uint32_t size = 128);

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
     * @param target_bytes_remain
     * @return true if the content was shifted and remaining() bytes are
     *  greater than or equal to the provided target_bytes_remain value
     */
    bool shift(uint32_t target_bytes_remain);

    /**
     * Extend the buffer by preserving the existing content and read/write
     * positions.
     *
     * @param size - the new size
     * @return true if operation was successful, false otherwise
     */
    bool extend(uint32_t size);

    /**
     * Resize the buffer. The data is preserved unless new size is less than
     * the previous.
     *
     * @param size - the new size
     * @return true if operation was successful, false otherwise
     */
    bool resize(uint32_t size);

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
     * Advance read position. Reset the buffer if read/writer pointers point to the
     * same position.
     *
     * @param bytes - the number of bytes to advance read pointer
     */
    void advanceReadPtr(uint32_t bytes);

    /**
     * Write scalar value to the buffer.
     *
     * @param data - the data to write
     * @parma extend_buff - the flag whether to extend the buffer if there is
     *  not enough allocated memory
     * @return true if the data was written, false otherwise. The failure could
     *  be caused by inability to allocate new memory if extend parameter is true
     *  or insufficient memory if extend parameter is false.
     */
    template<typename T>
    bool write(T val, bool extend_buff = true);

    /**
     * Write array of bytes to the buffer.
     *
     * @param data - the data to write
     * @parma extend_buff - the flag whether to extend the buffer if there is
     *  not enough allocated memory
     * @return true if the data was written, false otherwise. The failure could
     *  be caused by inability to allocate new memory if extend parameter is true
     *  or insufficient memory if extend parameter is false.
     */
    template<typename T, uint32_t N>
    bool writeChunk(const T (&vals)[N], bool extend_buff = true);

private:

// ATTRIBUTES

    uint32_t size_;
    uint8_t* data_;
    uint8_t* read_ptr_;
    uint8_t* write_ptr_;

}; // class Buff

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

inline Buff::Buff(uint32_t size) :
    size_(size),
    data_((uint8_t*) malloc(size_)),
    read_ptr_(data_),
    write_ptr_(data_)
{
}

inline Buff::~Buff()
{
    free(data_);
    data_ = NULL;
    read_ptr_ = NULL;
    write_ptr_ = NULL;
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
    memset(data_, 0, size_);
    read_ptr_ = data_;
    write_ptr_ = data_;
}

inline bool Buff::shift(uint32_t target_bytes_remain)
{
    uint32_t bytes_consumed = consumed();

    // Check if the buffer has some room to use for shifting.
    //
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

    return (target_bytes_remain <= remaining());
}

inline bool Buff::extend(uint32_t size)
{
    uint32_t bytes_remain = remaining();
    bool success = resize(size_ + (size - bytes_remain));
    return success;
}

inline bool Buff::resize(uint32_t size)
{
    uint32_t read_offset = read_ptr_ - data_;
    uint32_t write_offset = write_ptr_ - data_;
    uint8_t* data = (uint8_t*) realloc(data_, size * sizeof(uint8_t));

    if (data != NULL) {
        size_ = size;
        data_ = data;
        const uint8_t* end = this->end();

        if ((data_ + read_offset) < end) {
            read_ptr_ = data_ + read_offset;
        } else {
            read_ptr_ = data_;
        }
        if ((data_ + write_offset) < end) {
            write_ptr_ = data_ + write_offset;
        } else {
            write_ptr_ = data_;
        }
        memset(write_ptr_, 0, remaining());
        return true;
    } else {
        return false;
    }
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
inline bool Buff::write(T val, bool extend_buff)
{
    T vals[] = { val };
    return writeChunk(vals, extend_buff);
}

template<typename T, uint32_t N>
inline bool Buff::writeChunk(const T (&vals)[N], bool extend_buff)
{
    bool success = true;
    uint32_t bytes_target = sizeof(T) * N;
    uint32_t bytes_remain = remaining();

    if (bytes_remain < bytes_target) {
        success = shift(bytes_target);
    }
    if (!success && extend_buff) {
        success = extend(bytes_target);
    }
    if (success) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(vals);
        memcpy(write_ptr(), bytes, bytes_target);
        write_ptr() += bytes_target;
    }
    return success;
}

} // namespace utility

#endif // _utility_Buff_hpp_
