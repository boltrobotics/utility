// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_PwmMotor3Wire_hpp_
#define _btr_PwmMotor3Wire_hpp_

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#define GEAR_PRESCALER    4     // F_CPU 72MHz / 4 = 18MHz
#define GEAR_PWM_PERIOD   400   // Private timer 9MHz (18MHz / 2 center-align) / 22.5KHz = 400
#define SERVO_PRESCALER   72    // F_CPU 72MHz / 72 = 1MHz
#define SERVO_PWM_PERIOD  20000 // Private timer 1MHz / 50Hz = 20000

namespace btr
{

class PwmMotor3Wire
{
public:

  enum MotorType
  {
    GEAR,
    SERVO
  };

// LIFECYCLE

  /**
   * Ctor.
   *
   * @param motor_type - motor type, GEAR or SERVO
   * @param rcc_timer_clk - timer clock ID, RCC_TIMx
   * @param timer - timer ID, TIMx
   * @param timer_ocid - timer output channel ID (TIMC_OCx)
   * @param rcc_pwm_clk
   * @param pwm_port
   * @param pwm_pin
   * @param rcc_ina_clk
   * @param ina_port
   * @param ina_pin
   * @param rcc_inb_clk
   * @param inb_port
   * @param inb_pin
   * @param max_speed - in ticks between 0 - GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  PwmMotor3Wire(
      MotorType motor_type,
      rcc_periph_clken rcc_timer_clk,
      uint32_t timer,
      tim_oc_id timer_ocid,
      rcc_periph_clken rcc_pwm_clk,
      uint32_t pwm_port,
      uint16_t pwm_pin,
      rcc_periph_clken rcc_ina_clk,
      uint32_t ina_port,
      uint16_t ina_pin,
      rcc_periph_clken rcc_inb_clk,
      uint32_t inb_port,
      uint16_t inb_pin,
      uint16_t max_speed);

// OPERATIONS

  /**
   * @param velocity - PWM duty value. Reverse if negative, forward if positive, stop if zero
   */
  void setVelocity(int16_t velocity);

  /**
   * @return maximum duty in ticks between 0 and GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  uint16_t maxSpeed() const;

private:

// ATTRIBUTES

  uint32_t timer_;
  tim_oc_id timer_ocid_;
  uint32_t pwm_port_;
  uint16_t pwm_pin_;
  uint32_t ina_port_;
  uint16_t ina_pin_;
  uint32_t inb_port_;
  uint16_t inb_pin_;
  uint16_t max_speed_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

PwmMotor3Wire::PwmMotor3Wire(
    MotorType motor_type,
    rcc_periph_clken rcc_timer_clk,
    uint32_t timer,
    tim_oc_id timer_ocid,
    rcc_periph_clken rcc_pwm_clk,
    uint32_t pwm_port,
    uint16_t pwm_pin,
    rcc_periph_clken rcc_ina_clk,
    uint32_t ina_port,
    uint16_t ina_pin,
    rcc_periph_clken rcc_inb_clk,
    uint32_t inb_port,
    uint16_t inb_pin,
    uint16_t max_speed
    ) :
  timer_(timer),
  timer_ocid_(timer_ocid),
  pwm_port_(pwm_port),
  pwm_pin_(pwm_pin),
  ina_port_(ina_port),
  ina_pin_(ina_pin),
  inb_port_(inb_port),
  inb_pin_(inb_pin),
  max_speed_(max_speed)
{
  rcc_periph_clock_enable(rcc_timer_clk);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(rcc_ina_clk);
  rcc_periph_clock_enable(rcc_inb_clk);
  rcc_periph_clock_enable(rcc_pwm_clk);

  //gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM1_REMAP_NO_REMAP);

  gpio_set_mode(ina_port_, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ina_pin_);
  gpio_set_mode(inb_port_, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, inb_pin_);
  gpio_set_mode(pwm_port_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pwm_pin_);
  gpio_clear(ina_port_, ina_pin_);
  gpio_clear(inb_port_, inb_pin_);

  timer_disable_counter(timer_);
  //rcc_periph_reset_pulse(RST_TIM1); // replaces reset_timer(timer_)

  if (motor_type == GEAR) {
    // Set timer to center-aligned mode (Phase & Frequency Correct).
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_2, TIM_CR1_DIR_UP);

    // Prescalers: PLL (72e6) -> APBn (72e6 / 2 = 36e6) -> global timer (36e6 * 2 = 72e6) ->
    //  private timer (72e6 / 4 = 18e6) -> PWM center-aligned (18e6 / 2 = 9MHz)
    timer_set_prescaler(timer_, GEAR_PRESCALER);

    // Sets TIMx_ARR register (pwm period).
    // For servo at 50Hz: 180000 ticks at 9MHz => 20 mS period (180000 * 1/9e6)
    // For gear motor at 22,500Hz = 400 ticks at 9MHz clock => 44 uS period (400 * 1/9e6)
    // Only pulse width matters for servos (not PWM duty cycle, e.g. 20mS)
    timer_set_period(timer_, GEAR_PWM_PERIOD);
  } else {
    // To conserve power, stop sending servo control pulses (change mode to input, AVRP.220)
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_, SERVO_PRESCALER);
    timer_set_period(timer_, SERVO_PWM_PERIOD);
  }

  timer_enable_preload(timer_);
  timer_continuous_mode(timer_);

  // PWM1 mode sets output pin when TIMx_CNT < TIMx_CCR, otherwise it clears it.
  timer_disable_oc_output(timer_, timer_ocid_);
  timer_set_oc_mode(timer_, timer_ocid_, TIM_OCM_PWM1);
  timer_enable_oc_output(timer_, timer_ocid_);

  // Set TIMx_CCRx register (pwm duty). If the compare value in TIMx_CCRx is greater than the
  // auto-reload value in TIMx_ARR then OCxREF is held at 1. If the compare value is 0 then
  // OCxREF is held at 0 (STM32.388).
  timer_set_oc_value(timer_, timer_ocid_, 0);
  timer_enable_counter(timer_);
}

//============================================= OPERATIONS =========================================

void PwmMotor3Wire::setVelocity(int16_t velocity)
{
  if (velocity > 0) {
    timer_set_oc_value(timer_, timer_ocid_, velocity);
    gpio_set(ina_port_, ina_pin_);
    gpio_clear(inb_port_, inb_pin_);
  } else if (velocity < 0) {
    timer_set_oc_value(timer_, timer_ocid_, -velocity);
    gpio_clear(ina_port_, ina_pin_);
    gpio_set(inb_port_, inb_pin_);
  } else {
    timer_set_oc_value(timer_, timer_ocid_, velocity);
    gpio_clear(ina_port_, ina_pin_);
    gpio_clear(inb_port_, inb_pin_);
  }
}

uint16_t PwmMotor3Wire::maxSpeed() const
{
  return max_speed_;
}

} // namespace btr

#endif // _btr_PwmMotor3Wire_hpp_
