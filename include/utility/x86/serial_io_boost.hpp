// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_SerialIOBoost_hpp__
#define _btr_SerialIOBoost_hpp__

// SYSTEM INCLUDES
#include <boost/asio.hpp>

// PROJECT INCLUDES

namespace bio = boost::asio;

namespace btr
{

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
   * @return bytes available on the serial port
   */
  uint32_t available();

  /**
   * Read data from serial port.
   *
   * @param buff - container for received bytes
   * @param bytes - the number of bytes to read
   * @return the number of bytes transferred
   */
  ssize_t recv(char* buff, uint32_t bytes);

  /**
   * Write data to serial port.
   *
   * @param data - the data to send
   * @param bytes - the number of bytes to send
   * @param drain - block until all output has been transmitted
   * @return the number of bytes transferred
   */
  ssize_t send(const char* buff, uint32_t bytes, bool drain = false);

  /**
   * Transmit a continuous stream of 0 bits.
   *
   * @param duration - the length of the transmission. If duration is greater than 0, 0 bits are
   *  transmitted for duration milliseconds. If duration is 0, 0 bits are transmitted for 0.25
   *  seconds.
   */
  int sendBreak(uint32_t duration);

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
  size_t              bytes_transferred_;
  size_t              timeout_;

}; // class SerialIOBoost

} // namespace btr

#endif // _btr_SerialIOBoost_hpp__
