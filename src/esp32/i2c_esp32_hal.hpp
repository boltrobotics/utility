// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_I2C_ESP32_Hal_hpp_
#define _btr_I2C_ESP32_Hal_hpp_

// SYSTEM INCLUDES
//#include "driver/i2c_types.h"

// PROJECT INCLUDES
#include "utility/common/i2c.hpp"

namespace btr
{

/**
 * The class implements i2c interface for ESP32 platform.
 */
class I2C_ESP32_Hal : public I2C
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  I2C_ESP32_Hal();

// OPERATIONS

  /**
   * @see I2C::open
   */
  virtual void open();

  /**
   * @see I2C::close
   */
  virtual void close();

protected:

// OPERATIONS

  /**
   * @see I2C::start
   */
  virtual uint32_t start(uint8_t addr, uint8_t rw);

  /**
   * @see I2C::stop
   */
  virtual uint32_t stop();

  /**
   * @see I2C::sendByte
   */
  virtual uint32_t sendByte(uint8_t val);

  /**
   * @see I2C::receiveByte
   */
  virtual uint32_t receiveByte(bool ack, uint8_t* val);

  /**
   * Wait while I2C is busy with another transation.
   *
   * @return status code as described in defines.hpp
   */
  uint32_t waitBusy();

private:

// ATTRIBUTES

  i2c_master_bus_handle_t bus_handle_;                                                              
  i2c_master_dev_handle_t dev_handle_; 
};

} // namespace btr

#endif // _btr_I2C_ESP32_Hal_hpp_
