// Copyright (C) 2024 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/* ESP-IDF docs:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
 */

// SYSTEM INCLUDES
#include "driver/i2c_master.h"

// PROJECT INCLUDES
#include "i2c_esp32_hal.hpp"  // class implemented

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

namespace btr
{

static I2C_ESP32_Hal i2c_0;

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

I2C_ESP32_Hal::I2C_ESP32_Hal()
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

  if (open) {
    i2c_0.open();
  }
  return &i2c_0;
}

void I2C_ESP32_Hal::open()
{
  if (bus_handle_) {
    close();
  }

  i2c_master_bus_config_t config;
  config.i2c_port = BTR_I2C_MASTER_PORT;
  config.sda_io_num = (gpio_num_t) BTR_I2C_MASTER_SDA_IO;
  config.scl_io_num = (gpio_num_t) BTR_I2C_MASTER_SCL_IO;
  config.clk_source = BTR_I2C_CLK_SRC;
  config.glitch_ignore_cnt = BTR_I2C_GLITCH_IGNORE_COUNT;
  config.flags.enable_internal_pullup = BTR_I2C_INTERNAL_PULLUP;

  esp_err_t err = i2c_new_master_bus(&config, &bus_handle_);

  if (ESP_OK == err) {
    open_ = true;
  } else {
    ESP_ERROR_CHECK(err);

    if (ESP_ERR_NO_MEM == err) {
      set_status(status(), BTR_ENOMEM);
    } else if (ESP_ERR_NOT_FOUND == err) {
      // No more free bus.
      set_status(status(), BTR_EFAIL);
    } else {
      // I2C bus initialization failed because of invalid argument.
      set_status(status(), BTR_EINVAL);
    }
  }
}

void I2C_ESP32_Hal::close()
{
  i2c_del_master_bus(bus_handle_);
  bus_handle_ = nullptr;
  open_ = false;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

uint32_t I2C_ESP32_Hal::start(uint8_t addr, uint8_t rw)
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
      ESP_ERROR_CHECK(err);

      if (ESP_ERR_NO_MEM == err) {
        rc = BTR_ENOMEM; 
      } else {
        rc = BTR_EINVAL; 
      }
    }
  }
  return rc;
}

uint32_t I2C_ESP32_Hal::stop()
{
  i2c_master_bus_rm_device(dev_handle_);
  dev_handle_ = nullptr;
  return BTR_ENOERR;
}

uint32_t I2C_ESP32_Hal::sendByte(uint8_t val)
{
  esp_err_t err = i2c_master_transmit(dev_handle_, &val, sizeof(uint8_t), BTR_I2C_IO_TIMEOUT_MS);

  if (ESP_OK != err) {
    ESP_ERROR_CHECK(err);

    if (ESP_ERR_TIMEOUT == err) {
      // Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
      return BTR_ETIMEOUT;
    } else {
      return BTR_EINVAL; // ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
    }
  } else {
    return BTR_ENOERR;
  }
}

uint32_t I2C_ESP32_Hal::receiveByte(bool expect_ack, uint8_t* val)
{
  esp_err_t err = i2c_master_receive(dev_handle_, val, sizeof(uint8_t), BTR_I2C_IO_TIMEOUT_MS);

  if (ESP_OK != err) {
    ESP_ERROR_CHECK(err);

    if (ESP_ERR_TIMEOUT == err) {
      // Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
      return BTR_ETIMEOUT;
    } else {
      return BTR_EINVAL; // ESP_ERR_INVALID_ARG: I2C master receive parameter invalid.
    }
  } else {
    return BTR_ENOERR;
  }
}

uint32_t I2C_ESP32_Hal::waitBusy()
{
  esp_err_t err = i2c_master_bus_wait_all_done(bus_handle_, BTR_I2C_IO_TIMEOUT_MS);

  if (ESP_OK != err) {
    if (ESP_ERR_TIMEOUT == err) {
      return BTR_ETIMEOUT;
    } else { 
      return BTR_EFAIL;          
    }
  } else {
    return BTR_ENOERR;
  }
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // BTR_I2C_ENABLED > 0
