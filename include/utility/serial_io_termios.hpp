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
#include "utility/buff.hpp"

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
   * Flush not-transmitted and non-read data on the serial port.
   */
  std::error_code flush();


  /**
   * @return bytes available on the serial port
   */
  int available();

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

  // ATTRIBUTES

  int port_;

}; // class SerialIO

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

SerialIO::SerialIO(const std::string& port, int baud_rate, int timeout)
: port_()
{
  file = open(port.c_str(), O_RDWR | O_NOCTTY);

  if (file < 0) {
      throw "Failed to open " + port;
  }

  struct termios options;
  tcgetattr(file, &options);
  options.c_cflag = CS8 | CREAD | CLOCAL | B115200;
  options.c_iflag = IGNPAR | IGNCR;
  options.c_lflag &= ~ICANON;
  options.c_cc[VTIME] = timeout / 100; // In tenths of seconds
  options.c_cc[VMIN] = 0;
  tcflush(file, TCIOFLUSH);
  tcsetattr(file, TCSANOW, &options);
}

SerialIO::~SerialIO()
{
    close(port_);
}

//=================================== OPERATIONS ===============================

std::error_code SerialIO::flush()
{
  if (0 == tcflush(port_, TCIOFLUSH)) {
    return std::error_code(0, std::generic_category());
  } else {
    return std::error_code(errno, std::generic_category());
  }
}

int SerialIO::available()
{
  int bytes_avaiable;
  ioctl(port_, FIONREAD, &bytes_available);
  return bytes_available;
}

std::error_code SerialIO::recv(Buff* buff)
{
  int count = 0;

  while (buff->available() > 0) {
    count = read(port_, buff->write_ptr(), 1);

    if (count > 0) {
      buff->write_ptr()++;
    } else {
      break;
    }
  }

  if (count < 0) {
    return std::error_code(errno, std::generic_category());
  } else {
    return std::error_code();
  }
}

std::error_code SerialIO::send(Buff* buff)
{
  int count = write(port_, buff.read_ptr(), buff.available());

  if (count < 0) {
    return std::error_code(errno, std::generic_category());
  } else {
    return std::error_code();
  }
}

///////////////////////////////////// PRIVATE //////////////////////////////////

//=================================== OPERATIONS ===============================

} // namespace btr

#endif // _btr_SerialIO_hpp__
