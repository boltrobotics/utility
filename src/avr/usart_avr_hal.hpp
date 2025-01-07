// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usart_Avr_Hal_hpp_
#define _btr_Usart_Avr_Hal_hpp_

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/usart.hpp"
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class implements USART device on AVR.
 */
class UsartAvrHal : public Usart
{
public:

// LIFECYCLE

  /**
   * Create an instance and initialize data members.
   */
  UsartAvrHal(
      volatile uint8_t* ubrr_h,
      volatile uint8_t* ubrr_l,
      volatile uint8_t* ucsr_a,
      volatile uint8_t* ucsr_b,
      volatile uint8_t* ucsr_c,
      volatile uint8_t* udr);

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

// ATTRIBUTES

  volatile uint8_t* ubrr_h_;
  volatile uint8_t* ubrr_l_;
  volatile uint8_t* ucsr_a_;
  volatile uint8_t* ucsr_b_;
  volatile uint8_t* ucsr_c_;
  volatile uint8_t* udr_;

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

#if BTR_USART_TX_BUFF_SIZE > 256
  volatile uint16_t tx_head_;
  volatile uint16_t tx_tail_;
#else
  volatile uint8_t tx_head_;
  volatile uint8_t tx_tail_;
#endif // BTR_USART_TX_BUFF_SIZE > 256

  uint8_t tx_buff_[BTR_USART_TX_BUFF_SIZE];
};

} // namespace btr

#endif // _btr_Usart_Avr_Hal_hpp_
