/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_SerialIOTermios_hpp__
#define _btr_SerialIOTermios_hpp__

// SYSTEM INCLUDES
#include <string>
#include <system_error>

// PROJECT INCLUDES

namespace btr
{

class Buff;

/**
 * The class provides a send/receive interface to a serial port.
 */
class SerialIOTermios
{
public:

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param port - serial IO port
   * @param baud_rate - baud rate. It must be one of values specified by in termios.h
   *  @see http://man7.org/linux/man-pages/man3/termios.3.html
   * @param timeout - serial operation timeout in milliseconds
   */
  SerialIOTermios(const std::string& port, int baud_rate = 57600, int timeout = 0);

  /**
   * Dtor.
   */
  ~SerialIOTermios();

// OPERATIONS

  /**
   * Close and open the port.
   */
  void reset();

  /**
   * Flush not-transmitted and non-read data on the serial port.
   */
  std::error_code flush();

  /**
   * @return bytes available on the serial port
   */
  uint32_t available();

  /**
   * Set the number of bytes to read.
   *
   * @param bytes - the byte count
   */
  void setReadMinimum(uint32_t bytes);

  /**
   * Read data from serial port.
   *
   * @param buff - the buffer to read into
   * @param bytes - the number of bytes to read
   */
  std::error_code recv(Buff* buff, uint32_t bytes);

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
   * Convert numeric BAUD rate to termios-specific one.
   *
   * @param num - the number baud rate
   * @return termios baud rate
   */
  static int getNativeBaud(int num);

// ATTRIBUTES

  std::string port_name_;
  int baud_rate_;
  int timeout_millis_;
  int port_;

}; // class SerialIOTermios

} // namespace btr

#endif // _btr_SerialIOTermios_hpp__
