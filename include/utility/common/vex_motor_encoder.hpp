// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_VexMotorEncoder_hpp_
#define _btr_VexMotorEncoder_hpp_

// SYSTEM INCLUDES
#include <stdint.h>

// The default address of an encoder that's just been turned on.
#define I2CENCODER_DEFAULT_ADDRESS      0x30
#define I2CENCODER_ADDRESS_REGISTER     0x4D
#define I2CENCODER_POSITION_REGISTER    0x40
#define I2CENCODER_VELOCITY_REGISTER    0x44
#define I2CENCODER_ZERO_REGISTER        0x4A
#define I2CENCODER_UNTERMINATE_REGISTER 0x4B
#define I2CENCODER_TERMINATE_REGISTER   0x4C

// The addr_ to assign the first encoder. The addr_es after the first is this + (n - 1)
#define I2CENCODER_STARTING_ADDRESS     0x20

// 269 Motor:
// Encoder Revolutions to Output Rotations
// 1 (Output Rotation) / 30.056 (Encoder Revolution)
#define MOTOR_269_ROTATIONS             0.03327122704

// Time-delta to Minutes
// 64 microseconds (per revolution)
// 64 microseconds * 1s/10^6microseconds * 1 minute/60s
#define MOTOR_269_TIME_DELTA            0.00000106666

// 393 Motor configured for torque:
// Encoder Revolutions to Output Rotations
// 1 (Output Rotation) / 39.2 (Encoder Revolution)
#define MOTOR_393_TORQUE_ROTATIONS      0.02551020408	

// 393 Motor configured for speed: Ticks to Rotations
// Encoder Revolutions to Output Rotations
// 1 (Output Rotation) / 24.5 (Encoder Revolution)
#define MOTOR_393_SPEED_ROTATIONS       0.04081632653

// 393 Motor configured for Turbo speed: Ticks to Rotations
// Encoder Revolutions to Output Rotations
// 1 (Output Rotation) / 16.3 (Encoder Revolution)
#define MOTOR_393_TURBO_ROTATIONS       0.06134969325

// Time-delta to Minutes
// 128 microseconds (per revolution)
// 128 microseconds * 1s/10^6microseconds * 1 minute/60s
#define MOTOR_393_TIME_DELTA            0.00000213333

// The default number of ticks per encoder revolution
#define TICKS                           8

namespace btr
{

/**
 * The class provides an interface to VEX motor encoders.
 */
class VexMotorEncoder
{
public:

// LIFECYCLE

  /**
   * Create an encoder and automatically set its address to a next value up from enc_addr_counter.
   */
  VexMotorEncoder();

// OPERATIONS

  /**
   * Initialize.
   *
   * @param rotation_factor
   * @param time_delta
   * @param ticks
   */
  void init(double rotation_factor, double time_delta, int ticks = TICKS);

  /**
   * Sets whether or not the encoder is setup "backwards" or flipped.
   *
   * @param is_reversed - set to reversed if true
   */
  void setReversed(bool is_reversed);

  /**
   * @return the speed of the encoder rotation per minute for the output shaft of the motor.
   *  Assumes 269 motor.
   */
  double getSpeed();

  /**
   * @return the unsigned velocity bits. This is the time-delta between ticks in multiples of
   * 64 microseconds/tick. Stopped is 0xFFFF or 4 seconds. Assumes 269 motor.
   */
  uint16_t getVelocityBits();

  /**
   * @return the position in rotations since power on or last reset
   */
  double getPosition();

  /**
   * @return the position in encoder ticks since power on or last reset
   */
  long getRawPosition();

  /**
   * Zero the position.
   */
  void zero();

  /**
   * Allow access to all I2C devices after this encoder by unterminating it.
   */
  void unTerminate();

  /**
   * Prevent access to all I2C devices after this encoder by temrinating it.
   */
  void terminate();

  /**
   * @return I2C Address of this encoder for manual access.
   */
  int getAddress();

private:

// OPERATIONS

  /**
   * Access I2C register for write.
   *
   * @param reg - the register
   */
  void accessRegister(uint8_t reg);

// ATTRIBUTES

  static uint8_t enc_addr_counter_;
  static VexMotorEncoder* last_encoder_;
  int addr_;
  bool is_reversed_;
  float rotation_factor_;
  float time_delta_;
  int ticks_;
};

} // namespace btr

#endif // _btr_VexMotorEncoder_hpp_
