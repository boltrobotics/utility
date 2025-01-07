// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_Usart_X86_Hal_hpp_
#define _btr_Usart_X86_Hal_hpp_

// SYSTEM INCLUDES
#include <boost/asio.hpp>
namespace bio = boost::asio;

// PROJECT INCLUDES
#include "utility/common/usart.hpp"
#include "utility/common/defines.hpp"

namespace btr
{

/**
 * The class implements USART device on x86.
 */
class UsartX86Hal : public Usart
{
public:

// LIFECYCLE

  /**
   * Create an instance and initialize data members.
   */
  UsartX86Hal();

  /**
   * Call close()
   */
  virtual ~UsartX86Hal();

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

  bio::io_service     io_service_;
  bio::serial_port    serial_port_;
  bio::deadline_timer timer_;
  uint16_t            bytes_transferred_;

  volatile uint16_t rx_error_;
  bool enable_flush_;
};

} // namespace btr

#endif // _btr_Usart_X86_Hal_hpp_
