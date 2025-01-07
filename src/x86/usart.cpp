// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <boost/bind/bind.hpp>
#include <sys/ioctl.h>

// PROJECT INCLUDES
#include "utility/common/usart.hpp"

#define BOOST_SYSTEM_NO_DEPRECATED

namespace btr
{

static void onTimeout(Usart* u, const boost::system::error_code& error)
{
  // When the timer is cancelled, the error generated is bio::operation_aborted.
  //
  if (!error) {
    // When the timer fires, there is no error, therefore just cancel pending operation.
    u->serial_port_.cancel();
  }
}

static void timeAsyncOpr(Usart* u, uint32_t timeout)
{
  u->timer_.expires_from_now(boost::posix_time::milliseconds(timeout));
  u->timer_.async_wait(boost::bind(&onTimeout, u, bio::placeholders::error));
  u->io_service_.run();
}

static void onOprComplete(Usart* u, const boost::system::error_code& err, size_t bytes_transferred)
{
  u->bytes_transferred_ = bytes_transferred;

  if (err) {
    // When timer cancels operation, the error is 89, Operation canceled.
    errno = err.value();
  }
  u->timer_.cancel();
}

#if BTR_USART0_ENABLED > 0
static Usart usart_0;
#endif
#if BTR_USART1_ENABLED > 0
static Usart usart_1;
#endif
#if BTR_USART2_ENABLED > 0
static Usart usart_2;
#endif
#if BTR_USART3_ENABLED > 0
static Usart usart_3;
#endif

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart()
  :
    io_service_(),
    serial_port_(io_service_),
    timer_(io_service_),
    bytes_transferred_(0)
{
}

Usart::~Usart()
{
  close();
}

//============================================= OPERATIONS =========================================

// static
Usart* Usart::instance(uint32_t id, bool open)
{
  (void) open;

  switch (id) {
#if BTR_USART0_ENABLED > 0
    case 0:
      if (open) {
        usart_0.open(BTR_USART0_BAUD, BTR_USART0_DATA_BITS, BTR_USART0_STOP_BITS, BTR_USART0_PARITY,
            BTR_USART0_PORT);
      }
      return &usart_0;
#endif
#if BTR_USART1_ENABLED > 0
    case 1:
      if (open) {
        usart_1.open(BTR_USART1_BAUD, BTR_USART1_DATA_BITS, BTR_USART1_STOP_BITS, BTR_USART1_PARITY,
           BTR_USART1_PORT);
      }
      return &usart_1;
#endif
#if BTR_USART2_ENABLED > 0
    case 2:
      if (open) {
        usart_2.open(BTR_USART2_BAUD, BTR_USART2_DATA_BITS, BTR_USART2_STOP_BITS, BTR_USART2_PARITY,
           BTR_USART2_PORT);
      }
      return &usart_2;
#endif
#if BTR_USART3_ENABLED > 0
    case 3:
      if (open) {
        usart_3.open(BTR_USART3_BAUD, BTR_USART3_DATA_BITS, BTR_USART3_STOP_BITS, BTR_USART3_PARITY,
           BTR_USART3_PORT);
      }
      return &usart_3;
#endif
    default:
      return nullptr;
  }
}

bool Usart::isOpen()
{
  return serial_port_.is_open();
}

int Usart::open(
    uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity, const char* port)
{
  errno = 0;
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (0 == ec.value()) {
    serial_port_.set_option(bio::serial_port::baud_rate(baud));
    serial_port_.set_option(bio::serial_port::character_size(data_bits));
    //serial_port_.set_option(bio::serial_port::flow_control(flow_ctrl));

    switch (parity) {
      case EVEN:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::even));
        break;
      case ODD:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::odd));
        break;
      case NONE:
      default:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::none));
    }

    switch (stop_bits) {
      case ONEPOINTFIVE:
        serial_port_.set_option(bio::serial_port::stop_bits(
              bio::serial_port_base::stop_bits::onepointfive));
        break;
      case TWO:
        serial_port_.set_option(bio::serial_port::stop_bits(
              bio::serial_port_base::stop_bits::two));
        break;
      case ONE:
      default:
        serial_port_.set_option(bio::serial_port::stop_bits(
              bio::serial_port_base::stop_bits::one));
    }
  } else {
    errno = ec.value();
    return -1;
  }
  return 0;
}

void Usart::close()
{
  if (serial_port_.is_open()) {
    timer_.cancel();
    serial_port_.cancel();
    serial_port_.close();
  }
}

int Usart::available()
{
  int bytes_available;
  ioctl(serial_port_.lowest_layer().native_handle(), FIONREAD, &bytes_available);
  return bytes_available;
}

int Usart::flush(DirectionType dir)
{
  errno = 0;
  int rc = 0;

  switch (dir) {
    case IN:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIFLUSH);
      break;
    case OUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
      break;
    case INOUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  }
  return rc;
}

uint32_t Usart::send(const char* buff, uint16_t bytes, uint32_t timeout)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_write(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(
        &onOprComplete, this, bio::placeholders::error, bio::placeholders::bytes_transferred));

  timeAsyncOpr(this, timeout);
  return bytes_transferred_;
}

uint32_t Usart::recv(char* buff, uint16_t bytes, uint32_t timeout)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_read(
      serial_port_,
      bio::buffer(buff, bytes),
      boost::bind(
        &onOprComplete, this, bio::placeholders::error, bio::placeholders::bytes_transferred));

  timeAsyncOpr(this, timeout);
  return bytes_transferred_;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr
