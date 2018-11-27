/* Copyright (C) 2018 Bolt Robotics <info@boltrobotics.com> */

#ifndef _btr_SharedPtr_hpp_
#define _btr_SharedPtr_hpp_

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

  int count_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * The class represents an AVL self-balancing binary search tree.
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
      delete ptr_;
      delete counter_;
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
    delete ptr_;
    delete counter_;
  }
}

//============================================= OPERATIONS =========================================

inline void RefCounter::increment()
{
  ++count_;
}

inline int RefCounter::decrement()
{
  return --count_;
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
int SharedPtr<PtrType>::count() const
{
  return counter_->count();
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_SharedPtr_hpp_
