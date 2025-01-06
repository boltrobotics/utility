// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usart_hpp_
#define _btr_Usart_hpp_

// SYSTEM INCLUDES
#if BTR_X86 > 0
#include <boost/asio.hpp>
namespace bio = boost::asio;
#elif BTR_STM32 > 0
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <FreeRTOS.h>
#include <queue.h>
#endif

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

namespace btr
{

//==================================================================================================

/**
 * The class provides an interface to a USART device.
 */
class Usart
{
public:

// LIFECYCLE

#if BTR_X86 > 0
  /**
   * Create an instance and initialize data members.
   */
  Usart();

  /**
   * Call close()
   */
  ~Usart();

#elif BTR_AVR > 0
  /**
   * Create an instance and initialize data members.
   */
  Usart(
      volatile uint8_t* ubrr_h,
      volatile uint8_t* ubrr_l,
      volatile uint8_t* ucsr_a,
      volatile uint8_t* ucsr_b,
      volatile uint8_t* ucsr_c,
      volatile uint8_t* udr);

#elif BTR_STM32 > 0
  /**
   * Create an instance and initialize data members.
   */
  Usart(
      rcc_periph_clken rcc_gpio,
      rcc_periph_clken rcc_usart,
      uint32_t port,
      uint32_t pin,
      uint32_t irq,
      uint16_t tx,
      uint16_t rx,
      uint16_t cts,
      uint16_t rts);
#endif

  /**
   * Provide an USART instance identified by given id.
   *
   * @param id - port number of a USART per the platform this code is built for
   * @param open - open the instance if it is not already if true, false otherwise
   * @return an instance of a USART device. The instance will need to be initialized
   *  unless already done so.
   */
  static Usart* instance(uint32_t id, bool open);

// OPERATIONS

  /**
   * Check if device is open.
   *
   * @return true if port is open, false otherwise
   */
  bool isOpen();

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
  int open(
      uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity,
      const char* port = nullptr);

  /**
   * Stop the device, queues, clocks.
   */
  void close();

  /**
   * Check if there is data in receive queue.
   *
   * @return bytes available on the serial port or -1 if failed to retrieve the value
   */
  int available();

  /**
   * Flush pending, not-transmitted and non-read, data on the serial port.
   *
   * @param dir - one of:
   *  IN - flushes data received but not read.
   *  OUT - flushes data written but not transmitted.
   *  INOUT - flushes both data received but not read, and data written but not transmitted.
   */
  int flush(DirectionType dir);

  /**
   * Send a number of bytes from the buffer.
   *
   * @param buff - data buffer
   * @param bytes - number of bytes
   * @param timeout - maximum time to wait to submit one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  submitted
   */
  uint32_t send(const char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_TX_TIMEOUT_MS);

  /**
   * Receive a number of bytes and store in the buffer.
   *
   * @param buff - buffer to store received data
   * @param bytes - the number of bytes to receive
   * @param timeout - maximum time to wait to receive one character
   * @return bits from 16 up to 24 contain error code(s), lower 16 bits contains the number of bytes
   *  received
   */
  uint32_t recv(char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_RX_TIMEOUT_MS);

// ATTRIBUTES

#if BTR_X86 > 0
  bio::io_service     io_service_;
  bio::serial_port    serial_port_;
  bio::deadline_timer timer_;
  uint16_t            bytes_transferred_;
#elif BTR_AVR > 0
  volatile uint8_t* ubrr_h_;
  volatile uint8_t* ubrr_l_;
  volatile uint8_t* ucsr_a_;
  volatile uint8_t* ucsr_b_;
  volatile uint8_t* ucsr_c_;
  volatile uint8_t* udr_;
#elif BTR_STM32 > 0
  rcc_periph_clken rcc_gpio_;
  rcc_periph_clken rcc_usart_;
  uint32_t port_;
  uint32_t pin_;
  uint32_t irq_;
  uint16_t tx_;
  uint16_t rx_;
  uint16_t cts_;
  uint16_t rts_;
  QueueHandle_t tx_q_;
#endif

  volatile uint16_t rx_error_;
  bool enable_flush_;

#if BTR_STM32 > 0 || BTR_AVR > 0

#if BTR_USART_RX_BUFF_SIZE > 256
  volatile uint16_t rx_head_;
  volatile uint16_t rx_tail_;
#else
  volatile uint8_t rx_head_;
  volatile uint8_t rx_tail_;
#endif // BTR_USART_RX_BUFF_SIZE > 256

  uint8_t rx_buff_[BTR_USART_RX_BUFF_SIZE];
#endif // BTR_STM32 > 0 || BTR_AVR > 0

#if BTR_AVR > 0

#if BTR_USART_TX_BUFF_SIZE > 256
  volatile uint16_t tx_head_;
  volatile uint16_t tx_tail_;
#else
  volatile uint8_t tx_head_;
  volatile uint8_t tx_tail_;
#endif // BTR_USART_TX_BUFF_SIZE > 256

  uint8_t tx_buff_[BTR_USART_TX_BUFF_SIZE];
#endif // BTR_AVR > 0
};

} // namespace btr

#endif // _btr_Usart_hpp_
