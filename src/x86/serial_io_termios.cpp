// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

// PROJECT INCLUDES
#include "utility/x86/serial_io_termios.hpp"  // class implemented
#include "utility/buff.hpp"

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

SerialIOTermios::SerialIOTermios()
  :
  port_name_(""),
  baud_rate_(),
  data_bits_(),
  parity_(),
  port_(-1)
{
}

SerialIOTermios::~SerialIOTermios()
{
  close();
}

//============================================= OPERATIONS =========================================

int SerialIOTermios::open(
    const char* port_name,
    uint32_t baud_rate,
    uint8_t data_bits,
    uint8_t parity,
    uint32_t timeout_millis)
{
  port_name_ = port_name;
  data_bits_ = data_bits;
  parity_ = parity;

  baud_rate_ = getNativeBaud(baud_rate);
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
  //options.c_cflag = CS8 | CREAD | CLOCAL | baud_rate_;
  //options.c_iflag = IGNPAR | IGNCR;
  //options.c_lflag &= ~ICANON;

  switch (parity) {
    case PARITY_NONE:
      break;
    case PARITY_EVEN:
      options.c_cflag |= PARENB;
      break;
    case PARITY_ODD:
      options.c_cflag |= PARENB | PARODD;
      break;
    default:
      errno = EINVAL;
      return -1;
  }

  switch (data_bits) {
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

  baud_rate_ = getNativeBaud(baud_rate);

  if (cfsetospeed(&options, baud_rate_) != 0
      || cfsetispeed(&options, baud_rate_) != 0
      || tcsetattr(port_, TCSANOW, &options) != 0
      || flush(FLUSH_INOUT) != 0)
  {
    return -1;
  }

  int rc = setTimeout(timeout_millis);
  return rc;
}

void SerialIOTermios::close()
{
  if (port_ != -1) {
    ::close(port_);
    port_ = -1;
  }
}

int SerialIOTermios::setTimeout(uint32_t timeout_millis)
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
    options.c_cc[VTIME] = timeout_millis / 100;
    options.c_cc[VMIN] = 0;
    rc = tcsetattr(port_, TCSANOW, &options);
  }
  return rc;
}

int SerialIOTermios::flush(FlashType queue_selector)
{
  int rc = 0;

  switch (queue_selector) {
    case FLUSH_IN:
      rc = tcflush(port_, TCIFLUSH);
      break;
    case FLUSH_OUT:
      rc = tcflush(port_, TCOFLUSH);
      break;
    case FLUSH_INOUT:
      rc = tcflush(port_, TCIOFLUSH);
      break;
    default:
      errno = EINVAL;
      rc = -1;
  };
  return rc;
}

uint32_t SerialIOTermios::available()
{
  uint32_t bytes_available;
  ioctl(port_, FIONREAD, &bytes_available);
  return bytes_available;
}

void SerialIOTermios::setReadMinimum(uint32_t bytes)
{
  struct termios options;
  tcgetattr(port_, &options);
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = bytes;
  tcsetattr(port_, TCSANOW, &options);
}

int SerialIOTermios::recv(Buff* buff, uint32_t bytes)
{
  if (buff->remaining() < bytes) {
    errno = ENOBUFS;
    return -1;
  }

  ssize_t rc = 0;

  do {
    rc = read(port_, buff->write_ptr(), bytes);

    if (rc > 0) {
      // At least 1 byte is received, continue reading until received requested bytes.
      buff->write_ptr() += rc;
      bytes -= rc;
    } else if (rc == 0) {
      errno = ETIME;
      break; // Reached EOF or the call has timed out.
    } else {
      rc = -1; // Error occured.
      break;
    }
  } while (bytes > 0);

  return rc;
}

int SerialIOTermios::send(Buff* buff)
{
  int rc = 0;
  
  while (buff->available() > 0) {
    rc = write(port_, buff->read_ptr(), buff->available());

    // If system call was interrupted, try to write the available data again.
    //
    if (rc == -1) {
      if (errno != EINTR) {
        break;
      }
    }

    buff->read_ptr() += rc;
  }
  return rc;
}

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

int SerialIOTermios::getNativeBaud(int num)
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
