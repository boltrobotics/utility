// Copyright (C) 2018 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_PwmMotor2Wire_hpp_
#define _btr_PwmMotor2Wire_hpp_

// SYSTEM INCLUDES
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

// Prescalers:
//  PLL (72e6)
//  -> APB1:                   72MHz / 2 = 36MHz
  //  -> global timer:            36MHz * 2 = 72MHz (only when prescaler is greater than 1)
//  -> private timer:          36MHz / 1 = 36MHz
//  -> PWM center-aligned:     36MHz / 2 = 18MHz
//  -> private timer period:   18MHz / 20kHz = 900
//
// The above calculation results in frequency of 18.001kHz looking at oscilloscope. Set period to
// 320 (?) for 22.500kHz. 320/4 gives 20k.
//
#define GEAR_PWM_PERIOD   900
#define GEAR_PRESCALER    1

#define SERVO_PRESCALER   72    // F_CPU 72MHz / 72 = 1MHz
#define SERVO_PWM_PERIOD  20000 // Private timer 1MHz / 50Hz = 20000

namespace btr
{

class PwmMotor2Wire
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
   * @param motor_type - GEAR or SERVO
   * @param rcc_timer_clk - RCC_GPIOB for PB* pins, etc.
   * @param timer - RCC_TIM4 for T4* timer, etc.
   * @param timer_ocid_fw - TIM_OC3 for output compare channel 3, etc.
   * @param timer_ocid_bw - TIM_OC4 for output compare channel 4, etc.
   * @param rcc_pwm_clk_fw - 
   * @param pwm_port_fw
   * @param pwm_pin_fw
   * @param rcc_pwm_clk_bw
   * @param pwm_port_bw
   * @param pwm_pin_bw
   * @param max_duty - in ticks between 0 - GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  PwmMotor2Wire(
      MotorType motor_type,
      rcc_periph_clken rcc_timer_clk,
      uint32_t timer,
      tim_oc_id timer_ocid_fw,
      tim_oc_id timer_ocid_bw,
      rcc_periph_clken rcc_pwm_clk_fw,
      uint32_t pwm_port_fw,
      uint16_t pwm_pin_fw,
      rcc_periph_clken rcc_pwm_clk_bw,
      uint32_t pwm_port_bw,
      uint16_t pwm_pin_bw,
      uint16_t max_duty);

// OPERATIONS

  /**
   * @param duty - absolute value represents pwm duty, between 0 and maxDuty(); sign represents
   *  direction.
   */
  void setDuty(int16_t duty);

  /**
   * @return maximum duty in ticks between 0 and GEAR_PWM_PERIOD/SERVO_PWM_PERIOD
   */
  uint16_t maxDuty() const;

private:

// ATTRIBUTES

  uint32_t timer_;
  tim_oc_id timer_ocid_fw_;
  tim_oc_id timer_ocid_bw_;
  uint32_t pwm_port_fw_;
  uint16_t pwm_pin_fw_;
  uint32_t pwm_port_bw_;
  uint16_t pwm_pin_bw_;
  uint16_t max_duty_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

PwmMotor2Wire::PwmMotor2Wire(
      MotorType motor_type,
      rcc_periph_clken rcc_timer_clk,
      uint32_t timer,
      tim_oc_id timer_ocid_fw,
      tim_oc_id timer_ocid_bw,
      rcc_periph_clken rcc_pwm_clk_fw,
      uint32_t pwm_port_fw,
      uint16_t pwm_pin_fw,
      rcc_periph_clken rcc_pwm_clk_bw,
      uint32_t pwm_port_bw,
      uint16_t pwm_pin_bw,
      uint16_t max_duty
    ) :
  timer_(timer),
  timer_ocid_fw_(timer_ocid_fw),
  timer_ocid_bw_(timer_ocid_bw),
  pwm_port_fw_(pwm_port_fw),
  pwm_pin_fw_(pwm_pin_fw),
  pwm_port_bw_(pwm_port_bw),
  pwm_pin_bw_(pwm_pin_bw),
  max_duty_(max_duty)
{
  if (motor_type == GEAR) {
    if (max_duty_ > GEAR_PWM_PERIOD) {
      max_duty_ = GEAR_PWM_PERIOD;
    }
  } else {
    if (max_duty_ > SERVO_PWM_PERIOD) {
      max_duty_ = SERVO_PWM_PERIOD;
    }
  }

  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(rcc_timer_clk);
  rcc_periph_clock_enable(rcc_pwm_clk_fw);
  rcc_periph_clock_enable(rcc_pwm_clk_bw);

  //gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM1_REMAP_NO_REMAP);

  gpio_set_mode(pwm_port_fw_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pwm_pin_fw_);
  gpio_set_mode(pwm_port_bw_, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, pwm_pin_bw_);

  timer_disable_counter(timer_);
  //rcc_periph_reset_pulse(RST_TIM1); // replaces reset_timer(timer_)

  if (motor_type == GEAR) {
    // Set timer to center-aligned mode (Phase & Frequency Correct).
    // cms_1: interrupt flags are set when counting down 
    // cms_2: interrupt flags are set when counting up 
    // cms_3: interrupt flags are set when counting up and down 
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_2, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_, GEAR_PRESCALER);

    // Sets TIMx_ARR register (pwm period).
    timer_set_period(timer_, GEAR_PWM_PERIOD);
  } else {
    // To conserve power, stop sending servo control pulses (change mode to input, AVRP.220)
    timer_set_mode(timer_, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_, SERVO_PRESCALER);

    // Sets TIMx_ARR register (pwm period).
    // For servo at 50Hz: 180000 ticks at 9MHz => 20 mS period (180000 * 1/9e6).
    // For servo, only pulse width matters not PWM duty cycle, e.g. 20mS.
    timer_set_period(timer_, SERVO_PWM_PERIOD);
  }

  timer_enable_preload(timer_);
  timer_continuous_mode(timer_);

  // PWM1 mode sets output pin when TIMx_CNT < TIMx_CCR, otherwise it clears it.
  timer_disable_oc_output(timer_, timer_ocid_fw_);
  timer_disable_oc_output(timer_, timer_ocid_bw_);
  // TIM_OCM_PWM1: When counting up, the output channel is active (high) when the timer's count
  // is less than the timer capture/compare register, or else the channel goes low.
  timer_set_oc_mode(timer_, timer_ocid_fw_, TIM_OCM_PWM1);
  timer_set_oc_mode(timer_, timer_ocid_bw_, TIM_OCM_PWM1);
  timer_enable_oc_output(timer_, timer_ocid_fw_);
  timer_enable_oc_output(timer_, timer_ocid_bw_);

  // Set TIMx_CCRx register (pwm duty). If the compare value in TIMx_CCRx is greater than the
  // auto-reload value in TIMx_ARR then OCxREF is held at 1. If the compare value is 0 then
  // OCxREF is held at 0 (STM32.388).
  timer_set_oc_value(timer_, timer_ocid_fw_, 0);
  timer_set_oc_value(timer_, timer_ocid_bw_, 0);
  timer_enable_counter(timer_);
}

//============================================= OPERATIONS =========================================

void PwmMotor2Wire::setDuty(int16_t duty)
{
  // Process the values in this range: [-(max_duty - 2), (max_duty - 2)]

  if (duty > 0) {
    if ((duty + 1) > max_duty_) {
      duty = (max_duty_ - 1);
    }
    timer_set_oc_value(timer_, timer_ocid_fw_, max_duty_);
    timer_set_oc_value(timer_, timer_ocid_bw_, (max_duty_ - duty));
  } else if (duty < 0) {
    uint16_t duty_tmp = -duty;

    if ((duty_tmp + 1) > max_duty_) {
      duty_tmp = (max_duty_ - 1);
    }
    timer_set_oc_value(timer_, timer_ocid_bw_, max_duty_);
    timer_set_oc_value(timer_, timer_ocid_fw_, (max_duty_ - duty_tmp));
  } else {
    timer_set_oc_value(timer_, timer_ocid_fw_, max_duty_);
    timer_set_oc_value(timer_, timer_ocid_bw_, max_duty_);
    //timer_set_oc_value(timer_, timer_ocid_fw_, 0);
    //timer_set_oc_value(timer_, timer_ocid_bw_, 0);
  }
}

uint16_t PwmMotor2Wire::maxDuty() const
{
  return max_duty_;
}

} // namespace btr

#endif // _btr_PwmMotor2Wire_hpp_
