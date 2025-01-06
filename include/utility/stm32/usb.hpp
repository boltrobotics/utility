// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usb_hpp_
#define _btr_Usb_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to USB device on STM32F103C8T6 microcontroller.
 */
class Usb
{
public:

// LIFECYCLE

  Usb() = default;
  ~Usb() = default;

// OPERATIONS

  /**
   * Provide an USB instance identified by given id.
   *
   * @param id - port number of a USB per the platform this code is built for
   * @param open - open the instance if it is not already if true, false otherwise
   * @return an instance of a USB device. The instance may need to be initialized.
   */
  static Usb* instance(uint32_t id, bool open);

  /**
   * Check if device is open.
   *
   * @return true if port is open, false otherwise
   */
  static bool isOpen();

  /**
   * Open the device.
   */
  static int open();

  /**
   * Close the device.
   */
  static void close();

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  static int available();

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param queue_selector - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  static int flush(DirectionType queue_selector);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param timeout - maximum time to wait to submit one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  submitted
   */
  static uint32_t send(const char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_TX_TIMEOUT_MS);

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @param timeout - maximum time to wait to receive one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  received
   */
  static uint32_t recv(char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_RX_TIMEOUT_MS);
};

} // namespace btr

#endif // _btr_Usb_hpp_
