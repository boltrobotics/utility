// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_SharedPtr_hpp_
#define _btr_SharedPtr_hpp_

// SYSTEM INCLUDES
#include <stdlib.h>

// PROJECT INCLUDES
#include "utility/common/spin_lock.hpp"

namespace btr
{

/**
 * Provide a reference count.
 */
class RefCounter
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  RefCounter();

// OPERATIONS

  /**
   * Increment count.
   */
  void increment();

  /**
   * Decrement count.
   */
  int decrement();

  /**
   * @return the count
   */
  int count() const;

private:

// ATTRIBUTES

  SpinLock lock_;
  int count_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * The class represents a smart pointer that keeps track of the number of references to an
 * object. When reference cont goes to zero, the object is deleted.
 *
 * Custom shared pointer is required for development on C++ compatible platforms that don't
 * support standard template library.
 */
template <typename PtrType>
class SharedPtr
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  SharedPtr();

  /**
   * Copy ctor.
   */
  SharedPtr(const SharedPtr<PtrType>& shared_ptr);

  /**
   * Assignment.
   */
  SharedPtr<PtrType>& operator=(const SharedPtr<PtrType>& shared_ptr);

  /**
   * @param ptr
   */
  SharedPtr(PtrType* ptr);

  /**
   *
   */
  ~SharedPtr();

// OPERATIONS

  /**
   */
  PtrType& operator*();

  /**
   */
  PtrType* operator->();

  /**
   */
  PtrType* get();

  /**
   * @return the count
   */
  int count() const;

private:

// OPERATIONS


// ATTRIBUTES

  RefCounter* counter_;
  PtrType*    ptr_;

}; // class SharedPtr

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              INLINE
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

inline RefCounter::RefCounter() :
  lock_(),
  count_(1)
{
}

template<typename PtrType>
inline SharedPtr<PtrType>::SharedPtr() :
  counter_(new RefCounter()),
  ptr_(0)
{
}

template<typename PtrType>
inline SharedPtr<PtrType>::SharedPtr(const SharedPtr<PtrType>& shared_ptr) :
  counter_(shared_ptr.counter_),
  ptr_(shared_ptr.ptr_)
{
  counter_->increment();
}

template<typename PtrType>
inline SharedPtr<PtrType>& SharedPtr<PtrType>::operator=(const SharedPtr<PtrType>& shared_ptr)
{
  if (this != &shared_ptr) {
    if (0 == counter_->decrement()) {
#if BTR_ARD > 0
      ptr_->~PtrType();
      free(ptr_);
      counter_->~RefCounter();
      free(counter_);
#else
      delete ptr_;
      delete counter_;
#endif
    }

    ptr_ = shared_ptr.ptr_;
    counter_ = shared_ptr.counter_;
    counter_->increment();
  }
  return *this;
}

template<typename PtrType>
inline SharedPtr<PtrType>::SharedPtr(PtrType* ptr) :
  counter_(new RefCounter()),
  ptr_(ptr)
{
}

template<typename PtrType>
inline SharedPtr<PtrType>::~SharedPtr()
{
  if (0 == counter_->decrement()) {
#if BTR_ARD > 0
      ptr_->~PtrType();
      free(ptr_);
      counter_->~RefCounter();
      free(counter_);
#else
      delete ptr_;
      delete counter_;
#endif
  }
}

//============================================= OPERATIONS =========================================

inline void RefCounter::increment()
{
  lock_.lock();
  ++count_;
  lock_.unlock();
}

inline int RefCounter::decrement()
{
  lock_.lock();
  --count_;
  lock_.unlock();
  return count_;
}

inline int RefCounter::count() const
{
  return count_;
}

template<typename PtrType>
PtrType& SharedPtr<PtrType>::operator*()
{
  return *ptr_;
}

template<typename PtrType>
PtrType* SharedPtr<PtrType>::operator->()
{
  return ptr_;
}

template<typename PtrType>
PtrType* SharedPtr<PtrType>::get()
{
  return ptr_;
}

template<typename PtrType>
int SharedPtr<PtrType>::count() const
{
  return counter_->count();
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_SharedPtr_hpp_
