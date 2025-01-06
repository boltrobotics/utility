// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_I2C_hpp_
#define _btr_I2C_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/defines.hpp"
#include "utility/common/value_codec.hpp"

namespace btr
{

/**
 * The class defines an interface for handling I2C communication. Class implementations are
 * defined in platform-specific directories under utility/src/.
 */
class I2C
{
public:

// LIFECYCLE

  /**
   * No-op destructor.
   */
  virtual ~I2C() = default;

// OPERATIONS

  /**
   * Provide a device identified by port id.
   * The device is statically allocated (if configured @see defines.hpp) and is closed initially.
   *
   * @param dev_id - I2C device id, 0 or 1 for STM32F103, 0 for AVR.
   * @param open - open the device if it is not already
   * @return device instance
   */
  static I2C* instance(uint32_t dev_id, bool open);

  /**
   * Check if device is open.
   *
   * @return true if device is initialized, false otherwise
   */
  bool isOpen();

  /**
   * Initialize the device.
   */
  virtual void open() = 0;

  /**
   * Shut down the device.
   */
  virtual void close() = 0;

  /**
   * Scan for i2c devices and provide the number of available devices.
   *
   * @return upper 16 bits is status code as described in defines.hpp, lower 16 bits contain
   *  the number of detected devices.
   */
  uint32_t scan();

  /**
   * Write address, register and variable-byte value.
   *
   * @param addr - the address of a device to send the data to
   * @param reg - the address of a register on the device
   * @param value - the value
   * @return status code as described in defines.hpp
   */
  template<typename T>
  uint32_t write(uint8_t addr, uint8_t reg, T value);

  /**
   * Write address, register and multi-byte value.
   *
   * @param addr - the address of a device to send the data to
   * @param reg - the address of a register on the device
   * @param buff - the buffer with data
   * @param count - the number of bytes in buff
   * @return status code as described in defines.hpp
   */
  uint32_t write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t count);

  /**
   * Read value from a given register.
   *
   * @param addr - slave address
   * @param reg - register to read from
   * @param val - value to store received data to
   * @return status code as described in defines.hpp
   */
  template<typename T>
  uint32_t read(uint8_t addr, uint8_t reg, T* val);

  /**
   * Read multi-byte value from a specific register.
   *
   * @param addr - slave address
   * @param reg - register to read from
   * @param buff - buffer to store the data in
   * @param count - number of bytes to read
   * @return status code as described in defines.hpp
   */
  uint32_t read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count);

  /**
   * Read multi-byte value.
   *
   * @param addr - slave address
   * @param buff - the buffer to store the data to
   * @param count - the number of bytes in buff
   * @param stop_comm - if true, stop i2c communication once the data is read. If false, do NOT
   *  stop the communication. The reasons is that the read operation started elsewhere (other
   *  read() function, in which case that function will call stop instead.
   * @return status code as described in defines.hpp
   */
  uint32_t read(uint8_t addr, uint8_t* buff, uint8_t count, bool stop_comm = true);

protected:

// LIFECYCLE

  /**
   * Ctor.
   */
  I2C();

// OPERATIONS

  /**
   * Close, then open this I2C object.
   */
  void reset();

  /**
   * Start I2C transaction and send an address.
   *
   * @param addr - slave address
   * @param rw - 1 for read, 0 for write
   * @return status code as described in defines.hpp
   */
  virtual uint32_t start(uint8_t addr, uint8_t rw) = 0;

  /**
   * Generate I2C stop condition.
   * @return status code as described in defines.hpp
   */
  virtual uint32_t stop() = 0;

  /**
   * Send a single byte.
   *
   * @param val - the byte
   * @return status code as described in defines.hpp
   */
  virtual uint32_t sendByte(uint8_t val) = 0;

  /**
   * Receive one byte, the status should be ACK or NACK.
   *
   * @param ack - expect status ACK, otherwise NACK
   * @param val - buffer to store received value
   * @return status code as described in defines.hpp
   */
  virtual uint32_t receiveByte(bool ack, uint8_t* val) = 0;

// ATTRIBUTES

  /** Temporary buffer to read/write a byte to. */
  uint8_t buff_[sizeof(uint64_t)];
  /** Flag indicating if the device is open. */
  bool open_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

template<typename T>
inline uint32_t I2C::write(uint8_t addr, uint8_t reg, T value)
{
  if (sizeof(T) > 1 && ValueCodec::isLittleEndian()) {
    ValueCodec::swap(&value);
  }

  const uint8_t* buff = reinterpret_cast<uint8_t*>(&value);
  return write(addr, reg, buff, sizeof(T));
}

template<typename T>
inline uint32_t I2C::read(uint8_t addr, uint8_t reg, T* val)
{
  int rc = I2C::read(addr, reg, buff_, sizeof(T));

  if (is_ok(rc)) {
    ValueCodec::decodeFixedInt(buff_, val, sizeof(T), true);
  }
  return rc;
}

} // namespace btr

#endif // _btr_I2C_hpp_
