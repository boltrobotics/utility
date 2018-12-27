// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_PseudoTTY_hpp__
#define _btr_PseudoTTY_hpp__

// SYSTEM INCLUDES
#include <unistd.h>
#include <thread>
#include <chrono>
#include <sys/wait.h>
#include <signal.h>

using namespace std::chrono_literals;

namespace btr
{

#define PRG "socat"
#define TTY_SIM_0 "/tmp/ttySIM0"
#define TTY_SIM_1 "/tmp/ttySIM1"
#define PTY0 "PTY,link=" TTY_SIM_0 ",raw,echo=0"
#define PTY1 "PTY,link=" TTY_SIM_1 ",raw,echo=0"

/**
 * The class opens send and receive interfaces to a serial port upon constructing an instance.
 * The serial port is to close when the instance gets destroyed.
 */
class PseudoTTY
{
public:

// LIFECYCLE

  /**
   * Forks a child process.
   */
  PseudoTTY();

  /**
   * Terminates child process and reaps its PID
   */
  ~PseudoTTY();

private:

// ATTRIBUTES

  pid_t child_pid_;
};

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline PseudoTTY::PseudoTTY()
{
  switch (child_pid_ = vfork()) {
    case -1:
        throw std::runtime_error("Failed to fork pseudo TTY");
    case 0: // child
        execlp(PRG, PRG, PTY0, PTY1, (char*) NULL);
        throw std::runtime_error("Failed to exec: " PRG);
    default: // parent
        std::this_thread::sleep_for(50ms);
        break;
  }
}

inline PseudoTTY::~PseudoTTY()
{
  if (child_pid_ > 0) {
      kill(child_pid_, SIGTERM);
      waitpid(child_pid_, NULL, 0);
  }
}

} // namespace btr

#endif // _btr_PseudoTTY_hpp__
