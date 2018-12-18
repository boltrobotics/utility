// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL v3

#ifndef _btr_SerialIOBoost_hpp__
#define _btr_SerialIOBoost_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>

// PROJECT INCLUDES

namespace bio = boost::asio;

namespace btr
{

class Buff;

/**
 * The class provides a send/receive interface to a serial port.
 */
class SerialIOBoost
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
  SerialIOBoost(const std::string& port, int baud_rate, int timeout);

  /**
   * Dtor.
   */
  ~SerialIOBoost();

  // OPERATIONS

  /**
   * Flush not-transmitted and non-read data on the serial port.
   */
  std::error_code flush();

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

}; // class SerialIOBoost

} // namespace btr

#endif // _btr_SerialIOBoost_hpp__
