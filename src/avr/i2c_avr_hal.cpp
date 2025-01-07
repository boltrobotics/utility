// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// PROJECT INCLUDES
#include "utility/avr/i2c_avr_hal.hpp"  // class implemented
#include "utility/common/time.hpp"

// SYSTEM INCLDUES
#include <util/twi.h>
#include <util/delay.h>

#if BTR_I2C0_ENABLED > 0

namespace btr
{

static I2C_AVR_Hal i2c_0;

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
I2C* I2C::instance(uint32_t dev_id, bool open)
{
  (void) dev_id;

  if (open) {
    i2c_0.open();
  }
  return &i2c_0;
}

void I2C_AVR_Hal::open()
{
  // Set pull-ups. See discussion about external vs internal pull-ups:
  // http://www.avrfreaks.net/forum/twi-i2c-without-external-pull-resistors:

  if (BTR_I2C_INTERNAL_PULLUP) {
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    set_bit(PORTC, 4);
    set_bit(PORTC, 5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // ATmega2560: PD0 (SCL0), PD1 (SDA)
    set_bit(PORTD, 0);
    set_bit(PORTD, 1);
#endif
  } else {
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    clear_bit(PORTC, 4);
    clear_bit(PORTC, 5);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    clear_bit(PORTD, 0);
    clear_bit(PORTD, 1);
#endif
  }

  // Set speed.
  clear_bit(TWSR, TWPS0);
  clear_bit(TWSR, TWPS1);
  set_reg(TWBR, ((F_CPU / uint32_t(BTR_I2C_SPEED)) - 16) / 2);

  // Enable TWI operation and interface.
  set_reg(TWCR, BV(TWEN) | BV(TWEA));
  open_ = true;
}

void I2C_AVR_Hal::close()
{
  // Switch off TWI and terminate all ongoing transmissions
  set_reg(TWCR, 0);
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint32_t I2C_AVR_Hal::start(uint8_t addr, uint8_t rw)
{
  set_reg(TWCR, BV(TWINT) | BV(TWSTA) | BV(TWEN));
  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      rc = BTR_ESTART;
      reset();
      return rc;
    }

    uint8_t addr_rw = 0;

    if (BTR_I2C_READ == rw) {
      addr_rw = BTR_I2C_READ_ADDR(addr);
    } else {
      addr_rw = BTR_I2C_WRITE_ADDR(addr);
    }

    set_reg(TWDR, addr_rw);
    set_reg(TWCR, BV(TWINT) | BV(TWEN));

    rc = waitBusy();

    if (is_ok(rc)) {
      if (TW_MT_SLA_NACK == TW_STATUS || TW_MR_SLA_NACK == TW_STATUS) {
        rc = BTR_ENOACK;
        stop();
      } else if ((TW_MT_SLA_ACK != TW_STATUS) && (TW_MR_SLA_ACK != TW_STATUS)) {
        rc = BTR_ESTART;
        reset();
      }
    }
  }
  return rc;
}

uint32_t I2C_AVR_Hal::stop()
{
  uint32_t rc = BTR_ENOERR;
  set_reg(TWCR, BV(TWINT) | BV(TWEN) | BV(TWSTO));

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = MILLIS();

    while (bit_is_set(TWCR, TWSTO)) {
      if (TIME_DIFF(MILLIS(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_ETIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_clear(TWCR, TWSTO);
  }
  return rc;
}

uint32_t I2C_AVR_Hal::sendByte(uint8_t val)
{
  set_reg(TWDR, val);
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_DATA_NACK == TW_STATUS) {
      rc = BTR_ESENDBYTE;
      stop();
    } else if (TW_MT_DATA_ACK != TW_STATUS) {
      rc = BTR_ESENDBYTE;
      reset();
    }
  }
  return rc;
}

uint32_t I2C_AVR_Hal::receiveByte(bool expect_ack, uint8_t* val)
{
  set_reg(TWCR, BV(TWINT) | BV(TWEN));

  if (expect_ack) {
    set_reg(TWCR, TWCR | BV(TWEA));
  }

  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    if (TW_MT_ARB_LOST == TW_STATUS) {
      rc = BTR_ERECVBYTE;
      reset();
      return rc;
    }
  }

  *val = TWDR;

  if (expect_ack) {
    if (TW_MR_DATA_ACK != TW_STATUS) {
      rc = BTR_ENOACK;
    }
  } else {
    if (TW_MR_DATA_NACK != TW_STATUS) {
      rc = BTR_ENONACK;
    }
  }
  return rc;
}

uint32_t I2C_AVR_Hal::waitBusy()
{
  uint32_t rc = BTR_ENOERR;

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = MILLIS();

    while (bit_is_clear(TWCR, TWINT)) {
      if (TIME_DIFF(MILLIS(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_ETIMEOUT;
        reset();
        break;
      }
    }
  } else {
    loop_until_bit_is_set(TWCR, TWINT);
  }
  return rc;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_I2C0_ENABLED > 0
