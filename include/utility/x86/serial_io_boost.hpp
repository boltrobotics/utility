// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

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

  typedef enum
  {
    PARITY_NONE,
    PARITY_ODD,
    PARITY_EVEN
  } ParityType;

  typedef enum
  {
    FLUSH_IN,
    FLUSH_OUT,
    FLUSH_INOUT
  } FlashType;

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param port - serial IO port
   * @param baud_rate - baud rate
   * @param timeout - serial operation timeout in milliseconds
   */
  SerialIOBoost();

  /**
   * Dtor.
   */
  ~SerialIOBoost();

// OPERATIONS

  /**
   * Open serial port.
   *
   * @param port - serial IO port name (e.g., /dev/ttyS0)
   * @param baud_rate - baud rate. It must be one of values specified by in termios.h
   *  @see http://man7.org/linux/man-pages/man3/termios.3.html
   * @param data_bits
   * @param parity - @see ParityType
   * @param timeout - serial operation timeout in milliseconds
   * @return 0 on success, -1 on failure
   */
  int open(
      const char* port_name,
      uint32_t baud_rate,
      uint8_t data_bits,
      uint8_t parity,
      uint32_t timeout_millis);

  /**
   * Close serial port.
   */
  void close();

  /**
   * @param timeout_millis 
   */
  void setTimeout(uint32_t timeout_millis);

  /**
   * Flush not-transmitted and non-read data on the serial port.
   *
   * @param queue_selector - one of:
   *  FLUSH_IN - flushes data received but not read.
   *  FLUSH_OUT - flushes data written but not transmitted.
   *  FLUSH_INOUT - flushes both data received but not read, and data written but not transmitted.
   *
   *  @see termios(3)
   */
  int flush(FlashType queue_selector);

  /**
   * Read data from serial port.
   *
   * @param buff - byte container
   * @param bytes - the number of bytes to read
   */
  int recv(Buff* buff, uint32_t bytes);

  /**
   * Write data to serial port. The function increments buffer->read_ptr() by
   * the amount of buff->available() bytes on successful operation.
   *
   * @param data - the data to send
   */
  int send(Buff* buff);

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
  bio::serial_port    serial_port_;
  bio::deadline_timer timer_;
  int                 err_;
  size_t              expected_bytes_;
  size_t              timeout_;

}; // class SerialIOBoost

} // namespace btr

#endif // _btr_SerialIOBoost_hpp__
