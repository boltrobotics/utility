// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <boost/bind.hpp>

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
  err_(),
  expected_bytes_(0),
  timeout_(SERIAL_IO_TIMEOUT)
{
}

SerialIOBoost::~SerialIOBoost()
{
  serial_port_.close();
}

//============================================= OPERATIONS =========================================

int SerialIOBoost::open(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout_millis)
{
  boost::system::error_code ec;
  serial_port_.open(port_name, ec);

  if (ec) {
    err_ = std::error_code(ec.value(), std::generic_category());
  } else {
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
  }

  if (err_) {
    errno = err_.value();
    return -1;
  }
  return 0;
}

void SerialIOBoost::setTimeout(uint32_t timeout_millis)
{
  timeout_ = timeout_millis;
}

int SerialIOBoost::flush(FlashType queue_selector)
{
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

int SerialIOBoost::recv(Buff* buff)
{
  io_service_.reset();
  expected_bytes_ = buff->remaining();

  bio::async_read(
      serial_port_,
      bio::buffer(buff->write_ptr(), expected_bytes_),
      boost::bind(
        &SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  if (!err_) {
    buff->write_ptr() += expected_bytes_;
  } else {
    errno = err_.value();
    return -1;
  }
  return 0;
}

int SerialIOBoost::send(Buff* buff)
{
  io_service_.reset();
  expected_bytes_ = buff->available();

  bio::async_write(
      serial_port_,
      bio::buffer(buff->read_ptr(), expected_bytes_),
      boost::bind(&SerialIOBoost::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

  timeAsyncOpr();

  if (!err_) {
    buff->read_ptr() += expected_bytes_;
  } else {
    errno = err_.value();
    return -1;
  }
  return 0;
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
  if (err) {
    err_ = std::make_error_code(static_cast<std::errc>(err.value()));
  } else if (bytes_transferred != expected_bytes_) {
    err_ = std::make_error_code(std::errc::message_size);
  } else {
    err_.clear();
  }

  timer_.cancel();
}

void SerialIOBoost::onTimeout(const boost::system::error_code& error)
{
  // When the timer is cancelled, the error is generated.
  //
  if (!error) {
    serial_port_.cancel();
  }
}

} // namespace btr
