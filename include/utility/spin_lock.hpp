// Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

#ifndef _btr_SpinLock_hpp_
#define _btr_SpinLock_hpp_

// SYSTEM INCLUDES
#include <atomic>

namespace btr
{

/**
 * The class provide facilities to sort data.
 *
 * IMPORTANT: Implementation has to be compatible with AVR platform.
 */
class SpinLock
{
public:

// LIFECYCLE

  SpinLock() = default;
  ~SpinLock() = default;

// OPERATIONS

  void lock();
  void unlock();

private:

// ATTRIBUTES

  std::atomic_flag lock_ = ATOMIC_FLAG_INIT;

}; // class SpinLock

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline void SpinLock::lock()
{
  while (lock_.test_and_set(std::memory_order_acquire))
  {}
}

inline void SpinLock::unlock()
{
  lock_.clear(std::memory_order_release);
}

} // namespace btr

#endif // _btr_SpinLock_hpp_
