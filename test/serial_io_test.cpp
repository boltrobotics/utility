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

// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <unistd.h>
#include <signal.h>
#include <sstream>

// PROJECT INCLUDES
#include "serial_io.hpp"
#include "buff.hpp"
#include "test_helpers.hpp"

namespace bsy = boost::system;
namespace ber = bsy::errc;

namespace btr
{

#define PRG "socat"
#define TTY_SIM_0 "/tmp/ttySIM0"
#define TTY_SIM_1 "/tmp/ttySIM1"
#define PTY0 "PTY,link=" TTY_SIM_0 ",raw,echo=0"
#define PTY1 "PTY,link=" TTY_SIM_1 ",raw,echo=0"

//------------------------------------------------------------------------------

class PseudoTTY
{
public:
  PseudoTTY()
  {
    switch (child_pid_ = vfork()) {
      case -1:
          throw std::runtime_error("Failed to fork pseudo TTY");
      case 0: // child
          execlp(PRG, PRG, PTY0, PTY1, (char*) NULL);
          throw std::runtime_error("Failed to exec: " PRG);
      default: // parent
          boost::this_thread::sleep(boost::posix_time::millisec(50));
          break;
    }
  }

  ~PseudoTTY()
  {
    if (child_pid_ > 0) {
        kill(child_pid_, SIGTERM);
        waitpid(child_pid_, NULL, 0);
    }
  }

private:
  pid_t child_pid_;
};

//------------------------------------------------------------------------------

class SerialIOTest : public testing::Test
{
public:

  // LIFECYCLE

  SerialIOTest()
  : tty_(),
    act_serial_(TTY_SIM_0, 38400, 100),
    sim_serial_(TTY_SIM_1, 38400, 100),
    wbuff_(),
    rbuff_(),
    success_(),
    timeout_(std::make_error_code(std::errc::operation_canceled))
  {
    // TODO: Verify if newline is actuall required.
    uint8_t h[] =  { 'h','e','l','l','o','\n' };
    wbuff_.resize(sizeof(h));
    wbuff_.writeChunk(h);
    // Don't expect to receive endline character(s)
    rbuff_.resize(wbuff_.size());
  }

protected:

  // ATTRIBUTES

  PseudoTTY tty_;
  SerialIO act_serial_;
  SerialIO sim_serial_;
  Buff wbuff_;
  Buff rbuff_;
  std::error_code success_;
  std::error_code timeout_;
};

//------------------------------------------------------------------------------

struct Term
{
  Term()
  {
    // O_NDELAY causes "resource temporarily unavailable" when there is no
    // data.
    file = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

    if (file < 0) {
        throw "Failed to open device";
    }

    struct termios options;
    tcgetattr(file, &options);
    options.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
    options.c_iflag = IGNPAR | IGNCR;
    options.c_lflag &= ~ICANON;
    options.c_cc[VTIME] = 10; // 1 second
    options.c_cc[VMIN] = 0;
    tcflush(file, TCIOFLUSH);
    tcsetattr(file, TCSANOW, &options);
  }

  ~Term()
  {
    close(file);
  }

  std::error_code send(Buff& buff)
  {
    int count = write(file, buff.read_ptr(), buff.available());

    if (count < 0) {
        return std::error_code(errno, std::generic_category());
    } else {
        return std::error_code();
    }
  }

  std::error_code recv(Buff* buff)
  {
    int count = 0;

    while (buff->available() > 0) {
        count = read(file, buff->write_ptr(), 1);

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

  int file;
};

//------------------------------------------------------------------------------

// Tests {

TEST_F(SerialIOTest, ReadWriteOK)
{
  std::error_code e = sim_serial_.send(&wbuff_);
  ASSERT_EQ(success_, e) << " Message: " << e.message();

  e = act_serial_.recv(&rbuff_);

  ASSERT_EQ(success_, e) << " Message: " << e.message();
  ASSERT_EQ(0, memcmp(wbuff_.data(), rbuff_.data(), wbuff_.size())) << TestHelpers::toHex(rbuff_);
}

TEST_F(SerialIOTest, ReadTimeout)
{
  std::error_code e = act_serial_.recv(&rbuff_);
  ASSERT_EQ(timeout_, e) << " Message: " << e.message();
}

TEST_F(SerialIOTest, DISABLED_WriteTimeout)
{
  // FIXME: Write time-out simulation doesn't work.
  Buff large_buff;
  large_buff.resize(65536);
  std::error_code e = sim_serial_.send(&large_buff);
  ASSERT_EQ(timeout_, e) << " Message: " << e.message();
}

// } Tests

} // namespace btr
