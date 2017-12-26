/* Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef _btr_SerialIO_hpp__
#define _btr_SerialIO_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <termios.h>

// PROJECT INCLUDES
#include "buff.hpp"

namespace bio = boost::asio;

namespace btr
{

/**
 * The class provides a send/receive interface to a serial port.
 */
class SerialIO
{
public:

  // LIFECYCLE

  /**
   * Ctor.
   *
   * @param port - serial IO port
   * @param baud_rate - baud rate
   * @param timeout - serial operation timeout in milliseconds
   */
  SerialIO(const std::string& port, int baud_rate, int timeout);

  /**
   * Dtor.
   */
  ~SerialIO();

  // OPERATIONS

  /**
   * Read data from serial port.
   *
   * @param bytes - the number of bytes to read
   */
  std::error_code recv(Buff* buff);

  /**
   * Write data to serial port. The function increments buffer->read_ptr() by
   * the amount of buff->available() bytes on successful operation.
   *
   * @param data - the data to send
   */
  std::error_code send(Buff* buff);

private:

  // OPERATIONS

  /**
   * Schedule a timer for asynchronous operation execution.
   */
  void timeAsyncOpr();

  /**
   * Asynchronous operation completion handler.
   *
   * @param error - the error if any occured
   * @param bytes_transferred - the number of bytes transferred
   */
  void onOprComplete(const boost::system::error_code& err, size_t bytes_transferred);

  /**
   * Timer callback.
   *
   * @param error - the error if any
   */
  void onTimeout(const boost::system::error_code& error);

  // ATTRIBUTES

  bio::io_service     io_service_;
  bio::serial_port    serial_;
  bio::deadline_timer timer_;
  std::error_code     err_;
  size_t              expected_bytes_;
  size_t              timeout_;

}; // class SerialIO

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

SerialIO::SerialIO(const std::string& port, int baud_rate, int timeout)
: io_service_(),
  serial_(io_service_, port),
  timer_(io_service_),
  err_(),
  expected_bytes_(0),
  timeout_(timeout)
{
    serial_.set_option(bio::serial_port::baud_rate(baud_rate));
}

SerialIO::~SerialIO()
{
    serial_.close();
}

//=================================== OPERATIONS ===============================

std::error_code SerialIO::recv(Buff* buff)
{
    io_service_.reset();
    expected_bytes_ = buff->remaining();

    bio::async_read(
      serial_,
      bio::buffer(buff->write_ptr(), expected_bytes_),
      boost::bind(
        &SerialIO::onOprComplete,
        this,
        bio::placeholders::error,
        bio::placeholders::bytes_transferred));

    timeAsyncOpr();

    if (!err_) {
        buff->write_ptr() += expected_bytes_;
    }
    return err_;
}

std::error_code SerialIO::send(Buff* buff)
{
    io_service_.reset();
    expected_bytes_ = buff->available();

    bio::async_write(
      serial_,
      bio::buffer(buff->read_ptr(), expected_bytes_),
      boost::bind(&SerialIO::onOprComplete,
        this,
        bio::placeholders::error, 
        bio::placeholders::bytes_transferred));

    timeAsyncOpr();

    if (!err_) {
        buff->read_ptr() += expected_bytes_;
    }

    return err_;
}

///////////////////////////////////// PRIVATE //////////////////////////////////

//=================================== OPERATIONS ===============================

void SerialIO::timeAsyncOpr()
{
    timer_.expires_from_now(boost::posix_time::milliseconds(timeout_));
    timer_.async_wait(boost::bind(&SerialIO::onTimeout, this, bio::placeholders::error));
    io_service_.run();
}

void SerialIO::onOprComplete(
        const boost::system::error_code& err,
        size_t bytes_transferred)
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

void SerialIO::onTimeout(const boost::system::error_code& error)
{
    // When the timer is cancelled, the error is generated.
    //
    if (!error) {
        serial_.cancel(); // timed out
    }
}

} // namespace btr

#endif // _btr_SerialIO_hpp__
