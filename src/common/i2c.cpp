// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// PROJECT INCLUDES
#include "utility/common/i2c.hpp"

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

I2C::I2C()
  :
    buff_(),
    open_(false)
{
}

//============================================= OPERATIONS =========================================

bool I2C::isOpen()
{
  return open_;
}

uint32_t I2C::scan()
{
  if (isOpen()) {
    uint32_t rc = BTR_DEV_ENOERR;
    uint32_t count = 0;

    for (uint8_t addr = 0; addr < BTR_I2C_SCAN_MAX; addr++) {
      rc = start(addr, BTR_I2C_READ);

      if (is_ok(rc)) {
        ++count;
      } else {
        break;
      }
      stop();
    }
    set_status(status(), rc);
    return (rc | count);
  }
  return BTR_DEV_ENOTOPEN;
}

uint32_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t* buff, uint8_t bytes)
{
  if (isOpen()) {
    uint32_t rc = start(addr, BTR_I2C_WRITE);
    uint32_t count = 0;

    if (is_ok(rc)) {
      rc = sendByte(reg);

      if (is_ok(rc)) {
        ++count;

        for (uint8_t i = 0; i < bytes; i++) {
          rc = sendByte(buff[i]);

          if (is_err(&rc)) {
            break;
          }
          ++count;
        }
      }
      stop();
    }
    set_status(status(), rc);
    return (rc | count);
  }
  return BTR_DEV_ENOTOPEN;
}

uint32_t I2C::read(uint8_t addr, uint8_t reg, uint8_t* buff, uint8_t count)
{
  if (isOpen()) {
    uint32_t rc = start(addr, BTR_I2C_WRITE);

    if (is_ok(rc)) {
      rc = sendByte(reg);

      if (is_ok(rc)) {
        // Stop then start in read (I2C restart).
        stop();
        rc = read(addr, buff, count, false);
      }
      stop();
    }
    set_status(status(), rc);
    return rc;
  }
  return BTR_DEV_ENOTOPEN;
}

uint32_t I2C::read(uint8_t addr, uint8_t* buff, uint8_t bytes, bool stop_comm)
{
  if (isOpen()) {
    uint32_t rc = start(addr, BTR_I2C_READ);
    uint32_t count = 0;

    if (is_ok(rc)) {
      if (bytes == 0) {
        bytes++;
      }

      uint8_t nack = bytes - 1;

      for (uint8_t i = 0; i < bytes; i++) {
        if (i != nack) {
          rc = receiveByte(true, &buff[i]);
        } else {
          rc = receiveByte(false, &buff[i]);
        }
        if (is_err(rc)) {
          break;
        }
      }
      if (stop_comm) {
        stop();
      }
    }
    set_status(status(), rc);
    return (rc | count);
  }
  return BTR_DEV_ENOTOPEN;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

void I2C::reset()
{
  close();
  open();
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0
