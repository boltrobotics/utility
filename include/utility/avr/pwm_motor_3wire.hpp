// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_PwmMotor3Wire_hpp_
#define _btr_PwmMotor3Wire_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

#define GEAR_PRESCALER    1     // F_CPU 16MHz / 1 = 16MHz
#define GEAR_PWM_PERIOD   400   // Private timer 8MHz (16MHz / 2 center-align) / 22KHz = 400
#define SERVO_PRESCALER   16    // F_CPU 16MHz / 16 = 1MHz
#define SERVO_PWM_PERIOD  20000 // Private timer 1MHz / 50Hz = 20000

namespace btr
{

class PwmMotor3Wire
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  PwmMotor3Wire(
      volatile uint8_t* tccr_a,
      volatile uint8_t* tccr_b,
      volatile uint8_t* icr,
      volatile uint16_t* ocr,
      uint8_t ina_pin,
      uint8_t inb_pin,
      uint8_t pwm_pin);

// OPERATIONS

  void setVelocity(int16_t speed, uint8_t reverse);

private:

// ATTRIBUTES

  volatile uint16_t* ocr_;
  uint8_t ina_pin_;
  uint8_t inb_pin_;
  uint8_t pwm_pin_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

PwmMotor3Wire::PwmMotor3Wire(
    volatile uint8_t* tccr_a,
    volatile uint8_t* tccr_b,
    volatile uint8_t* icr,
    volatile uint16_t* ocr,
    uint8_t ina_pin,
    uint8_t inb_pin,
    uint8_t pwm_pin
  ) :
  ocr_(ocr),
  ina_pin_(ina_pin),
  inb_pin_(inb_pin),
  pwm_pin_(pmw_pin)
{
  pinMode(ina_pin_, OUTPUT);
  pinMode(inb_pin_, OUTPUT);
  pinMode(pwm_pin_, OUTPUT);

#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  // For DC use phase-frequency correct, for servos use Fast PWM

  // There are 2 control registers for 8-bit timer (TCCRnA:B) and 3 for 16-bit timer (TCCRnA:C).
  //
  // COM1Ax (T1CA): 1 0 = Clear OCnA/OCnB on compare match when up-counting, set on compare match
  //  when down-counting (Table 17-5).
  // COM1Bx (T1CB): same as T1CA
  // WGMn3:0: 1000 = PWM, Phase and Frequency Correct, TOP: ICRn
  //
  // TODO Replace timer 1 reference (COM1xn, WGMnn, CSnn) with variables.
  //
  //*tccr_a = 0b10100000; // Bits 1:0 are WGMn1:0. Bits 7,5 are COMnA1, COMnB1
  *tccr_a = (1 << COM1A1) | (1 << COM1B1);
  //*tccr_b = 0b00010001; // Bits 4:3 are WGMn3:2. Bit 0 is CSn0
  *tccr_b = (1 << WGM13) | (1 << CS10);

  // ICR (16-bit in size) is updated with TCNTn value and defines TOP value. ICR is supported
  // only by 16-bit timers. If a fixed TOP value is required, the ICRn Register can be used as an
  // alternative, freeing the OCRnA to be used as PWM output.
  //
  // Assuming F_CPU = 16MHz, on 8MHZ clock one period is 400 ticks (20KHz, 1 tick/50 uS).
  // F_CPU 16MHz / 1 (no prescaling) / 2 (phase-correct dual-slope) / 400 (ICRn TOP) = 20KHz
  //
  *icr = max_period;

#endif
}

//============================================= OPERATIONS =========================================

void PwmMotor3Wire::setVelocity(int16_t speed, uint8_t reverse)
{
  *ocr_ = speed;

  if (speed != 0) {
    if (reverse) {
      digitalWrite(ina_pin_, LOW);
      digitalWrite(inb_pin_, HIGH);
    } else {
      digitalWrite(ina_pin_, HIGH);
      digitalWrite(inb_pin_, LOW);
    }
  } else {
    digitalWrite(ina_pin_, LOW);
    digitalWrite(inb_pin_, LOW);
  }
}

} // namespace btr

#endif // _btr_PwmMotor3Wire_hpp_
