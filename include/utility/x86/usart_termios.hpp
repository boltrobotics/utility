// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_UsartTermios_hpp__
#define _btr_UsartTermios_hpp__

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class UsartTermios
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   */
  UsartTermios();

  /**
   * Dtor.
   */
  ~UsartTermios();

// OPERATIONS

  /**
   * Configure USART parameters.
   *
   * @param port_name - serial IO port name (e.g., /dev/ttyS0)
   * @param baud_rate - baud rate. It must be one of values specified by in termios.h
   *  @see http://man7.org/linux/man-pages/man3/termios.3.html
   * @param data_bits
   * @param parity - @see ParityType
   * @param timeout - serial operation timeout
   */
  void configure(
      const char* port_name,
      uint32_t baud_rate,
      uint8_t data_bits,
      uint8_t parity,
      uint32_t timeout = BTR_USART_IO_TIMEOUT_MS);

  /**
   * Check if port is open.
   *
   * @return true if port is open, false otherwise
   */
  bool isOpen();

  /**
   * Initialize the device.
   */
  int open();

  /**
   * Stop the device, close the port.
   */
  void close();

  /**
   * @param timeout - timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  int setTimeout(uint32_t timeout);

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  int available();

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param queue_selector - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  int flush(DirectionType queue_selector);

  /**
   * Send a single character.
   *
   * @param ch - the character to send
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(char ch, bool drain = false);

  /**
   * Send data from buff up to null character.
   *
   * @param buff - data buffer
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff, bool drain = false);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param drain - block until all output has been transmitted
   * @return 0 on success, -1 if queue was full
   */
  int send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * Receive a single character.
   *
   * @return the received character or -1 on error
   */
  int recv();

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @return bytes received or -1 on error
   */
  int recv(char* buff, uint32_t bytes);

private:

// OPERATIONS

  /**
   * Convert numeric BAUD rate to termios-specific one.
   *
   * @param num - the number baud rate
   * @return termios baud rate
   */
  static int getNativeBaud(int num);

// ATTRIBUTES

  const char* port_name_;
  uint32_t baud_rate_;
  uint8_t data_bits_;
  uint8_t parity_;
  int port_;
  size_t timeout_;

}; // class UsartTermios

} // namespace btr

#endif // _btr_UsartTermios_hpp__
