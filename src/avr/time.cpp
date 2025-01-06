// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <avr/io.h>
#include <util/atomic.h>
#include <time.h>

// PROJECT INCLUDES
#include "utility/common/time.hpp"  // class implemented

#if BTR_TIME_ENABLED > 0

////////////////////////////////////////////////////////////////////////////////////////////////////
// Local defines {

#define TCCRA     TCCR0A
#define TCCRB     TCCR0B
#define TCNT      TCNT0
#define TIMSK     TIMSK0
#define OCIE      OCIE0A
#define OCR       OCR0A
#define TOIE      TOIE0
#define TIFR      TIFR0
#define TOV       TOV0
#define CS0       CS00
#define CS1       CS01
#define CS2       CS02
#define WGM       WGM01
// See: http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
#define OCR_VECT  TIMER0_COMPA_vect
#define OVF_VECT  TIMER0_OVF_vect

// CPU thresholds are the maximum values for 8-bit OCR to trigger at 1 millisecond.
#if F_CPU > 16320000UL
#define BTR_TIME_PRESCALER  256
#define BTR_TIME_CLK_SEL    ( BV(CS2) )
#elif F_CPU > 2040000UL
#define BTR_TIME_PRESCALER  64
#define BTR_TIME_CLK_SEL    ( BV(CS1) | BV(CS0) )
#elif F_CPU > 255
#define BTR_TIME_PRESCALER  8
#define BTR_TIME_CLK_SEL    ( BV(CS1) )
#endif

#define BTR_TIME_ONE_SEC_MS 1000
#define BTR_TIME_SCALER     1000
#define BTR_TIME_OCR        ((F_CPU / BTR_TIME_PRESCALER) / BTR_TIME_SCALER)

// } Local defines

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static members {

static volatile uint32_t millis_ = 0;

// } Static members

////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs {

//ISR(OVF_VECT)
ISR(OCR_VECT)
{
  uint32_t m = millis_ + 1;

  // This assumes that ISR is invoked every one millisecond.
  if (0 == (m % BTR_TIME_ONE_SEC_MS)) {
    system_tick();
  }
  millis_ = m;
}

// } ISRs

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

// static
void Time::init()
{
  set_bit(TCCRA, WGM);
  TCCRB = BTR_TIME_CLK_SEL;
  OCR = BTR_TIME_OCR;
  set_bit(TIMSK, OCIE);
}

// static
void Time::shutdown()
{
  clear_bit(TIMSK, OCIE);
}

// static
uint32_t Time::sec()
{
  return time(nullptr);
}

// static
uint32_t Time::millis()
{
  uint32_t v;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    v = millis_;
  }
  return v;
}

// static
uint32_t Time::diff(uint32_t head_time, uint32_t tail_time)
{
  return ((UINT32_MAX + head_time - tail_time) % UINT32_MAX);
}

} // namespace btr

#endif // BTR_TIME_ENABLED > 0
