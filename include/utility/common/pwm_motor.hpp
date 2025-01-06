// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_PwmMotor_hpp_
#define _btr_PwmMotor_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

namespace btr
{

/**
 * Driver for a PWM motor (L298N, VNH5019, etc.).
 *
 * PWM motor options:
 *  1) INA - direction 1, digital pin
 *     INB - direction 2, digital pin
 *     PWM - speed
 *     Board: L298N (can use configuration #2)
 *  2) Use 2 PWM pins for both ports INA and INB
 *     Board: MX1508
 *
 * Optional
 *  - CS - current sense, analog pin (Ali 2SP30, L298N IC but not on Ali breakout board)
 *  - DIAG - diagnostic, digital pin (Pololu 2SP30, VNH5019)
 */
template<typename PwmMotorImpl, typename... Mixins>
class PwmMotor : public Mixins...
{
public:

// LIFECYCLE

  /**
   * Ctor.
   */
  PwmMotor(const PwmMotorImpl& impl, const Mixins&... mixins);

// OPERATIONS

  /**
   * Set velocity.
   *
   * @param velocity
   */
  void setVelocity(int16_t velocity);

private:

// ATTRIBUTES

  PwmMotorImpl pwm_motor_impl_;

}; // class PwmMotor

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

template<typename PwmMotorImpl, typename... Mixins>
PwmMotor<PwmMotorImpl, Mixins...>::PwmMotor(const PwmMotorImpl& impl, const Mixins&... mixins)
  :
  Mixins(mixins)...,
  pwm_motor_impl_(impl)
{
}

//============================================= OPERATIONS =========================================

template<typename PwmMotorImpl, typename... Mixins>
void PwmMotor<PwmMotorImpl, Mixins...>::setVelocity(int16_t velocity)
{
  uint8_t forward = 1;

  if (velocity < 0) {
    // Make velocity a positive quantity.
    velocity = -velocity;
    // Preserve the direction.
    forward = 0;
  }

  uint16_t speed = velocity;

  if (speed > pwm_motor_impl_.max_duty()) {
    speed = pwm_motor_impl_.max_duty();
  }

  pwm_motor_impl_.setSpeed(speed, forward);
}

#if 0
uint8_t PwmMotor::getLoad()
{
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  return analogRead(current_sense_pin_) * 34;
}

uint8_t PwmMotor::getFault()
{
  return !digitalRead(diag_pin_);
}
#endif

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================


/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr

#endif // _btr_PwmMotor_hpp_
