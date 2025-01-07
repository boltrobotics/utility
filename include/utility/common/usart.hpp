// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class provides an interface to a USART device.
 */
class Usart
{
public:

// LIFECYCLE

  virtual ~Usart() = default;

// OPERATIONS

  /**
   * Provide an USART instance identified by given id.
   *
   * @param id - port number of a USART per the platform this code is built for
   * @param open - open the instance if it is not already if true, false otherwise
   * @return an instance of a USART device. The instance will need to be initialized
   *  unless already done so.
   */
  static Usart* instance(uint32_t id, bool open);

  /**
   * Check if device is open.
   *
   * @return true if port is open, false otherwise
   */
  virtual bool isOpen() = 0;

  /**
   * Open USART device.
   *
   * @param baud - baud rate must be one of standard values
   * @param data_bits - number of data bits
   * @param stop_bits - number of stop bits
   * @param parity - parity type
   * @param port - serial IO port name (e.g. on x86: /dev/ttyS0, on AVR: nullptr)
   * @return 0 on success, -1 on failure
   * @see http://man7.org/linux/man-pages/man3/termios.3.html
   */
  virtual int open(
      uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity,
      const char* port = nullptr) = 0;

  /**
   * Stop the device, queues, clocks.
   */
  virtual void close() = 0;

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  virtual int available() = 0;

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param dir - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  virtual int flush(DirectionType dir) = 0;

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param timeout - maximum time to wait to submit one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  submitted
   */
  virtual uint32_t send(
      const char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_TX_TIMEOUT_MS) = 0;

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @param timeout - maximum time to wait to receive one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  received
   */
  virtual uint32_t recv(
      char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_RX_TIMEOUT_MS) = 0;
};

} // namespace btr

#endif // _btr_Usart_hpp_
