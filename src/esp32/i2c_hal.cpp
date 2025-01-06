// Copyright (C) 2024 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/* ESP-IDF docs:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
 */

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/esp32/i2c_hal.hpp"  // class implemented

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

#if BTR_I2C0_ENABLED > 0
static I2C i2c_0(0);
#endif

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

I2C_Hal::I2C_Hal()
  :
    I2C(),
    bus_handle_(nullptr),
    dev_handle_(nullptr)
{
}

//============================================= OPERATIONS =========================================

// static
I2C* I2C::instance(uint32_t dev_id, bool open)
{
  (void) dev_id;

  if (open && false == isOpen()) {
    i2c_0.open();
  }
  return &i2c_0;
}

void I2C::open()
{
  if (bus_handle_) {
    close();
  }

  i2c_master_bus_config_t config = {
    .clk_source = BTR_I2C_CLK_SRC,
    .i2c_port = BTR_I2C_MASTER_PORT,
    .scl_io_num = BTR_I2C_MASTER_SCL_IO,
    .sda_io_num = BTR_I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = BTR_I2C_GLITCH_IGNORE_COUNT,
    .flags.enable_internal_pullup = BTR_I2C_INTERNAL_PULLUP,
  };

  esp_err_t err = i2c_new_master_bus(&config, &bus_handle_);

  if (ESP_OK == err) {
    open_ = true;
  }

  uint32_t rc = BTR_DEV_ENOERR;

  if (ESP_OK != err) {
    ESP_ERROR_CHECK(err)

    if (ESP_ERR_NO_MEM == rc) {
      rc = BTR_DEV_ENOMEM; // Create I2C bus failed because of out of memory.
    } else if (ESP_ERR_NOT_FOUND == rc) {
      rc = BTR_DEV_EFAIL; // No more free bus.
    } else {
      rc = BTR_DEV_EINVAL; // I2C bus initialization failed because of invalid argument.
    }
  }
  return rc;
}

void I2C::close()
{
  i2c_del_master_bus(bus_handle_);
  bus_handle_ = nullptr;
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint32_t I2C::start(uint8_t addr, uint8_t rw)
{
  (void) rw;

  if (dev_handle_) {
    stop();
  }

  uint32_t rc = waitBusy();

  if (is_ok(rc)) {
    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = addr,
      .scl_speed_hz = BTR_I2C_SPEED,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_);

    if (ESP_OK != err) {
      ESP_ERROR_CHECK(err)

      if (ESP_ERR_NO_MEM == rc) {
        rc = BTR_DEV_ENOMEM; 
      } else {
        rc = BTR_DEV_EINVAL; 
      }
    }
  }
  return rc;
}

uint32_t I2C::stop()
{
  i2c_master_bus_rm_device(dev_handle_);
  dev_handle_ = nullptr;
  return BTR_DEV_ENOERR;
}

uint32_t I2C::sendByte(uint8_t val)
{
  esp_err_t err = i2c_master_transmit(dev_handle_, &val, sizeof(uint8_t), BTR_I2C_IO_TIMEOUT_MS);
  uint32_t rc = BTR_DEV_ENOERR;

  if (ESP_OK != err) {
    ESP_ERROR_CHECK(err)

    if (ESP_ERR_TIMEOUT == rc) {
      // Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
      rc = BTR_DEV_ETIMEOUT;
    } else {
      rc = BTR_DEV_EINVAL; // ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
    }
  }
  return rc;
}

uint32_t I2C::receiveByte(bool expect_ack, uint8_t* val)
{
  esp_err_t err = i2c_master_receive(dev_handle_, val, sizeof(uint8_t), BTR_I2C_IO_TIMEOUT_MS);
  uint32_t rc = BTR_DEV_ENOERR;

  if (ESP_OK != err) {
    ESP_ERROR_CHECK(err)

    if (ESP_ERR_TIMEOUT == rc) {
      // Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
      rc = BTR_DEV_ETIMEOUT;
    } else {
      rc = BTR_DEV_EINVAL; // ESP_ERR_INVALID_ARG: I2C master receive parameter invalid.
    }
  }
  return rc;
}

uint32_t I2C::waitBusy()
{
  esp_err_t err = i2c_master_bus_wait_all_done(bus_handle_, BTR_I2C_IO_TIMEOUT_MS);
  uint32_t rc = BTR_DEV_ENOERR;

  if (ESP_OK != err)
    if (ESP_ERR_TIMEOUT == err) {
      rc = BTR_DEV_ETIMEOUT;
    } else { 
      rc = BTR_DEV_EFAIL;          
    }
  }
  return rc;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_I2C_ENABLED > 0
