// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usart_STM32_Hal_hpp_
#define _btr_Usart_STM32_Hal_hpp_

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <FreeRTOS.h>
#include <queue.h>

// PROJECT INCLUDES
#include "utility/common/usart.hpp"
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class implements USART device on STM32.
 */
class UsartStm32Hal : public Usart
{
public:

// LIFECYCLE

  /**
   * Create an instance and initialize data members.
   */
  UsartStm32Hal(
      rcc_periph_clken rcc_gpio,
      rcc_periph_clken rcc_usart,
      uint32_t port,
      uint32_t pin,
      uint32_t irq,
      uint16_t tx,
      uint16_t rx,
      uint16_t cts,
      uint16_t rts);

// OPERATIONS

  /**
   * @see Usart#isOpen
   */
  virtual bool isOpen();

  /**
   * @see Usart#open
   */
  virtual int open(
      uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity,
      const char* port = nullptr);

  /**
   * @see Usart#close
   */
  virtual void close();

  /**
   * @see Usart#available
   */
  virtual int available();

  /**
   * @see Usart#flush
   */
  virtual int flush(DirectionType dir);

  /**
   * @see Usart#send
   */
  virtual uint32_t send(
      const char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_TX_TIMEOUT_MS);

  /**
   * @see Usart#recv
   */
  virtual uint32_t recv(char* buff, uint16_t bytes, uint32_t timeout = BTR_USART_RX_TIMEOUT_MS);

private:

// ATTRIBUTES

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

  volatile uint16_t rx_error_;
  bool enable_flush_;

#if BTR_USART_RX_BUFF_SIZE > 256
  volatile uint16_t rx_head_;
  volatile uint16_t rx_tail_;
#else
  volatile uint8_t rx_head_;
  volatile uint8_t rx_tail_;
#endif // BTR_USART_RX_BUFF_SIZE > 256

  uint8_t rx_buff_[BTR_USART_RX_BUFF_SIZE];
};

} // namespace btr

#endif // _btr_Usart_STM32_Hal_hpp_
