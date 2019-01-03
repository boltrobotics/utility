// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <boost/bind.hpp>
#include <iostream>

// PROJECT INCLUDES
#include "utility/x86/serial_io_boost.hpp"  // class implemented
#include "utility/buff.hpp"

#define BOOST_SYSTEM_NO_DEPRECATED
#ifndef SERIAL_IO_TIMEOUT
#define SERIAL_IO_TIMEOUT 100
#endif

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

SerialIOBoost::SerialIOBoost()
  :
  io_service_(),
  serial_port_(io_service_),
  timer_(io_service_),
  bytes_transferred_(0),
  timeout_(SERIAL_IO_TIMEOUT)
{
}

SerialIOBoost::~SerialIOBoost()
{
  close();
}

//============================================= OPERATIONS =========================================

int SerialIOBoost::open(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout_millis)
{
  if (serial_port_.is_open()) {
    close();
  }

  errno = 0;
  boost::system::error_code ec;
  serial_port_.open(port_name, ec);

  if (0 == ec.value()) {
    serial_port_.set_option(bio::serial_port::baud_rate(baud_rate));
    serial_port_.set_option(bio::serial_port::character_size(data_bits));

    switch (parity) {
      case PARITY_EVEN:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::even));
        break;
      case PARITY_ODD:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::odd));
        break;
      case PARITY_NONE:
      default:
        serial_port_.set_option(bio::serial_port::parity(bio::serial_port::parity::none));
    }
  } else {
    errno = ec.value();
    return -1;
  }
  return 0;
}

void SerialIOBoost::close()
{
  serial_port_.close();
}

void SerialIOBoost::setTimeout(uint32_t timeout_millis)
{
  timeout_ = timeout_millis;
}

int SerialIOBoost::flush(FlashType queue_selector)
{
  errno = 0;
  int rc = 0;

  switch (queue_selector) {
    case FLUSH_IN:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIFLUSH);
      break;
    case FLUSH_OUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCOFLUSH);
      break;
    case FLUSH_INOUT:
      rc = tcflush(serial_port_.lowest_layer().native_handle(), TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  }
  return rc;
}

ssize_t SerialIOBoost::recv(Buff* buff, uint32_t bytes)
{
  if (buff->remaining() < bytes) {
    errno = ENOBUFS;
    return -1;
  }

  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_read(
      serial_port_,
      bio::buffer(buff->write_ptr(), bytes),
      boost::bind(
        &SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  buff->write_ptr() += bytes_transferred_;
  return bytes_transferred_;
}

ssize_t SerialIOBoost::send(Buff* buff)
{
  io_service_.reset();
  errno = 0;
  bytes_transferred_ = 0;

  bio::async_write(
      serial_port_,
      bio::buffer(buff->read_ptr(), buff->available()),
      boost::bind(&SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  buff->read_ptr() += bytes_transferred_;
  return bytes_transferred_;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

void SerialIOBoost::timeAsyncOpr()
{
  timer_.expires_from_now(boost::posix_time::milliseconds(timeout_));
  timer_.async_wait(boost::bind(&SerialIOBoost::onTimeout, this, bio::placeholders::error));
  io_service_.run();
}

void SerialIOBoost::onOprComplete(const boost::system::error_code& err, size_t bytes_transferred)
{
  bytes_transferred_ = bytes_transferred;

  if (err) {
    // When timer cancels operation, the error is 89, Operation canceled.
    errno = err.value();
  }
  timer_.cancel();
}

void SerialIOBoost::onTimeout(const boost::system::error_code& error)
{
  // When the timer is cancelled, the error generated is bio::operation_aborted.
  //
  if (!error) {
    // When the timer fires, there is no error, therefore just cancel pending operation.
    serial_port_.cancel();
  }
}

} // namespace btr
