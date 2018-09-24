/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
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
#include <unistd.h>
#include <thread>
#include <chrono>
#include <sys/wait.h>
#include <signal.h>

// PROJECT INCLUDES
#include "utility/pseudo_tty.hpp"  // class implemented

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

PseudoTTY::PseudoTTY()
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

PseudoTTY::~PseudoTTY()
{
  if (child_pid_ > 0) {
      kill(child_pid_, SIGTERM);
      waitpid(child_pid_, NULL, 0);
  }
}

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr
