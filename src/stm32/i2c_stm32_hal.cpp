// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <cstddef>
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"
#include "FreeRTOS.h"
#include "task.h"

// PROJECT INCLUDES
#include "utility/stm32/i2c_stm32_hal.hpp"  // class implemented
#include "utility/common/time.hpp"

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

#if BTR_I2C0_ENABLED > 0
static I2C_STM32_Hal i2c_0(I2C1);
#endif

#if BTR_I2C1_ENABLED > 0
static I2C_STM32_Hal i2c_1(I2C2);
#endif

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

I2C_STM32_Hal::I2C_STM32_Hal(uint32_t dev_id)
  :
    I2C(),
    dev_id_(dev_id)
{
}

//============================================= OPERATIONS =========================================

// static
I2C* I2C::instance(uint32_t id, bool open)
{
  switch (id) {
#if BTR_I2C0_ENABLED > 0
    case 0:
      if (open) {
        i2c_0.open();
      }
      return &i2c_0;
#endif
#if BTR_I2C1_ENABLED > 0
    case 1:
      if (open) {
        i2c_1.open();
      }
      return &i2c_1;
#endif
    default:
      set_status(status(), BTR_EINVAL);
      return nullptr;
  }
}

void I2C_STM32_Hal::open()
{
	rcc_periph_clock_enable(RCC_GPIOB);
	//rcc_periph_clock_enable(RCC_AFIO);

  if (I2C1 == dev_id_) {
    rcc_periph_clock_enable(RCC_I2C1);
	  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
    gpio_set(GPIOB, GPIO6 | GPIO7);
    gpio_primary_remap(0, 0); 
  } else {
    rcc_periph_clock_enable(RCC_I2C2);
	  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO10 | GPIO11);
    gpio_set(GPIOB, GPIO10 | GPIO11);
  }

	i2c_peripheral_disable(dev_id_);
	//i2c_reset(dev_id_); OBSOLETE?
	I2C_CR1(dev_id_) &= ~I2C_CR1_STOP; // Clear stop.

  // Set bus speed
  i2c_set_clock_frequency(dev_id_, 36);
  i2c_speeds speed = (uint32_t(BTR_I2C_SPEED) > 100000 ? i2c_speed_fm_400k : i2c_speed_sm_100k);
  // Set mode, ccr, and trise.
  i2c_set_speed(dev_id_, speed, (rcc_apb1_frequency / 1000000));

	i2c_set_dutycycle(dev_id_, I2C_CCR_DUTY_DIV2);
	//i2c_set_own_7bit_slave_address(dev_id_, 0x23);
	i2c_peripheral_enable(dev_id_);
  open_ = true;
}

void I2C_STM32_Hal::close()
{
	i2c_peripheral_disable(dev_id_);
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint32_t I2C_STM32_Hal::start(uint8_t addr, uint8_t rw)
{
  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    // Clear acknowledge failure.
    I2C_SR1(dev_id_) &= ~I2C_SR1_AF;
		// Disable stop generation.
    i2c_clear_stop(dev_id_);

    if (BTR_I2C_READ == rw) {
      i2c_enable_ack(dev_id_);
    }

    i2c_send_start(dev_id_);
    uint32_t start_ms = Time::millis();

    while (false == (
          (I2C_SR1(dev_id_) & I2C_SR1_SB) &&
          (I2C_SR2(dev_id_) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
    {
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_ETIMEOUT;
        reset();
        return rc;
      }
      taskYIELD();
    }

    i2c_send_7bit_address(dev_id_, addr, rw);
    start_ms = Time::millis();

    // Wait until the address is sent and ACK received, or either NACK or time-out occurs.
    while (false == (I2C_SR1(dev_id_) & I2C_SR1_ADDR)) {
      // Check if ACK Failed.
      if (I2C_SR1(dev_id_) & I2C_SR1_AF) {
        rc = BTR_ENOACK;
        stop();
        return rc;
      }
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_ETIMEOUT;
        stop();
        return rc;
      }
      taskYIELD();
    }
    // Clear status register.
    I2C_SR2(dev_id_);
  }
  return 0;
}

uint32_t I2C_STM32_Hal::stop()
{
  i2c_send_stop(dev_id_);
  return BTR_ENOERR;
}

uint32_t I2C_STM32_Hal::sendByte(uint8_t val)
{
	i2c_send_data(dev_id_, val);

  uint32_t rc = BTR_ENOERR;
  uint32_t start_ms = Time::millis();

  // Wait for send to finish or time out.
	while (false == (I2C_SR1(dev_id_) & (I2C_SR1_BTF))) {
    if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
      rc = BTR_ETIMEOUT;
      stop();
      break;
    }
		taskYIELD();
	}
  return rc;
}

uint32_t I2C_STM32_Hal::receiveByte(bool expect_ack, uint8_t* val)
{
	if (false == expect_ack) {
		i2c_disable_ack(dev_id_);
  } else {
    i2c_enable_ack(dev_id_);
  }

  uint32_t rc = BTR_ENOERR;
  uint32_t start_ms = Time::millis();

	while (false == (I2C_SR1(dev_id_) & I2C_SR1_RxNE)) {
    if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
      rc = BTR_ETIMEOUT;
      reset();
      return rc;
    }
		taskYIELD();
	}
	
  *val = i2c_get_data(dev_id_);
  return rc;
}

uint32_t I2C_STM32_Hal::waitBusy()
{
  uint32_t rc = BTR_ENOERR;

  if (BTR_I2C_IO_TIMEOUT_MS > 0) {
    uint32_t start_ms = Time::millis();

    while (I2C_SR2(dev_id_) & I2C_SR2_BUSY) {
      if (Time::diff(Time::millis(), start_ms) > BTR_I2C_IO_TIMEOUT_MS) {
        rc = BTR_ETIMEOUT;
        reset();
        break;
      }
      taskYIELD();
    }
  } else {
    while (I2C_SR2(dev_id_) & I2C_SR2_BUSY) {
      taskYIELD();
    }
  }
  return rc;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_I2C_ENABLED > 0
