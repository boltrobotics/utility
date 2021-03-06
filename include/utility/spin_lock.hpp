// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_SpinLock_hpp_
#define _btr_SpinLock_hpp_

// SYSTEM INCLUDES
#if BTR_X86 > 0
#include <atomic>
#endif

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

#if BTR_X86 > 0
  std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
#endif

}; // class SpinLock

/////////////////////////////////////////////// INLINE /////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= OPERATIONS =========================================

inline void SpinLock::lock()
{
#if BTR_X86 > 0
  while (lock_.test_and_set(std::memory_order_acquire))
  {}
#endif
}

inline void SpinLock::unlock()
{
#if BTR_X86 > 0
  lock_.clear(std::memory_order_release);
#endif
}

} // namespace btr

#endif // _btr_SpinLock_hpp_
