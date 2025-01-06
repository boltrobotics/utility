// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

// PROJECT INCLUDES
#include "utility/x86/usart_termios.hpp"

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

UsartTermios::UsartTermios()
  :
  port_name_(nullptr),
  baud_rate_(),
  data_bits_(),
  parity_(),
  port_(-1)
{
}

UsartTermios::~UsartTermios()
{
  close();
}

//============================================= OPERATIONS =========================================

void UsartTermios::configure(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout)
{
  port_name_ = port_name;
  baud_rate_ = baud_rate;
  data_bits_ = data_bits;
  parity_ = parity;
  timeout_ = timeout;
}

bool UsartTermios::isOpen()
{
  return (port_ != -1);
}

int UsartTermios::open()
{
  port_ = ::open(port_name_, O_RDWR | O_NOCTTY);

  if (port_ < 0) {
    errno = EBADF;
    return -1;
  }

  struct termios options;

  if (tcgetattr(port_, &options) != 0) {
    return -1;
  }

  cfmakeraw(&options);
  //bzero(&options, sizeof(struct termios));
  //options.c_cflag = CS8 | CREAD | CLOCAL | baud_rate;
  //options.c_iflag = IGNPAR | IGNCR;
  //options.c_lflag &= ~ICANON;

  switch (parity_) {
    case NONE:
      break;
    case EVEN:
      options.c_cflag |= PARENB;
      break;
    case ODD:
      options.c_cflag |= PARENB | PARODD;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  switch (data_bits_) {
    case 8:
      options.c_cflag |= CS8;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  int baud_rate = getNativeBaud(baud_rate_);

  if (cfsetospeed(&options, baud_rate) != 0
      || cfsetispeed(&options, baud_rate) != 0
      || tcsetattr(port_, TCSANOW, &options) != 0
      || flush(INOUT) != 0)
  {
    return -1;
  }

  int rc = setTimeout(timeout_);
  return rc;
}

void UsartTermios::close()
{
  if (port_ != -1) {
    ::close(port_);
    port_ = -1;
  }
}

int UsartTermios::setTimeout(uint32_t timeout)
{
  struct termios options;
  int rc = 0;

  if ((rc = tcgetattr(port_, &options)) == 0) {
    // 1. VMIN = 0 and VTIME > 0
    //  read returns if one or more bytes are available or VTIME expires
    // 2. VMIN > 0 and VTIME > 0
    //  read returns when either VMIN characters are received or VTIME BETWEEN characters expires
    //
    // See http://unixwiz.net/techtips/termios-vmin-vtime.html
    //
    // WARNING: VTIME is in tenths of a second, so the minimum can be set is 100 milliseconds
    //
    options.c_cc[VTIME] = timeout / 100;
    options.c_cc[VMIN] = 0;
    rc = tcsetattr(port_, TCSANOW, &options);
  }
  return rc;
}

int UsartTermios::available()
{
  int bytes_available;
  ioctl(port_, FIONREAD, &bytes_available);
  return bytes_available;
}

int UsartTermios::flush(DirectionType queue_selector)
{
  int rc = 0;

  switch (queue_selector) {
    case IN:
      rc = tcflush(port_, TCIFLUSH);
      break;
    case OUT:
      rc = tcflush(port_, TCOFLUSH);
      break;
    case INOUT:
      rc = tcflush(port_, TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  };
  return rc;
}

int UsartTermios::send(char ch, bool drain)
{
  return send(&ch, 1, drain);
}

int UsartTermios::send(const char* buff, bool drain)
{
  uint32_t bytes = strlen(buff);
  return send(buff, bytes, drain);
}

int UsartTermios::send(const char* buff, uint32_t bytes, bool drain)
{
  int rc = write(port_, buff, bytes);

  if (drain) {
    tcdrain(port_);
  }
  return rc;
}

int UsartTermios::recv()
{
  char buff[1];
  return recv(buff, 1);
}

int UsartTermios::recv(char* buff, uint32_t bytes)
{
  int rc = read(port_, buff, bytes);
  return rc;
}

#if 0
void UsartTermios::setReadMinimum(uint32_t bytes)
{
  struct termios options;
  tcgetattr(port_, &options);
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = bytes;
  tcsetattr(port_, TCSANOW, &options);
}

int UsartTermios::sendBreak(uint32_t duration)
{
  return tcsendbreak(port_, duration);
}
#endif

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

int UsartTermios::getNativeBaud(int num)
{
  int baud = B57600;

  switch (num) {
    case 9600:
      baud = B9600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      break;
  };

  return baud;
}

} // namespace btr
