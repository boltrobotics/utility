// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES

// PROJECT INCLUDES
#include "utility/common/vex_motor_encoder.hpp" // class implemented
#include "utility/common/i2c.hpp"

#if BTR_VEXIMU_ENABLED > 0

namespace btr
{

uint8_t VexMotorEncoder::enc_addr_counter_ = I2CENCODER_STARTING_ADDRESS;
VexMotorEncoder* VexMotorEncoder::last_encoder_ = NULL;

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

VexMotorEncoder::VexMotorEncoder()
  :
    addr_(enc_addr_counter_),
    is_reversed_(false),
    rotation_factor_(0),
    time_delta_(0),
    ticks_(0)
{
  enc_addr_counter_++;
}

//============================================= OPERATIONS =========================================

void VexMotorEncoder::init(double rotation_factor, double time_delta, int ticks)
{
  // Unterminates previous encoder so that messages flow to this one.
  if (last_encoder_) {
    last_encoder_->unTerminate();
  }
  last_encoder_ = this;
  
  rotation_factor_ = rotation_factor;
  time_delta_ = time_delta;
  ticks_ = ticks;

  I2C* i2c = I2C::instance(BTR_VEXIMU_PORT_I2C, true);
  i2c->write(I2CENCODER_DEFAULT_ADDRESS, uint8_t(I2CENCODER_ADDRESS_REGISTER), uint8_t(addr_ << 1));

  zero();
}

void VexMotorEncoder::setReversed(bool is_reversed)
{
  is_reversed_ = is_reversed;
}

double VexMotorEncoder::getSpeed()
{
  uint16_t vb = getVelocityBits();

  if (vb == 0xFFFF) {
    return 0;
  }
  return (rotation_factor_ / (double(vb) * time_delta_));
}

uint16_t VexMotorEncoder::getVelocityBits()
{
  uint16_t speed = 0;
  I2C* i2c = I2C::instance(BTR_VEXIMU_PORT_I2C, false);
  i2c->read(addr_, uint8_t(I2CENCODER_VELOCITY_REGISTER), &speed);
  return speed;
}

double VexMotorEncoder::getPosition()
{
  return (rotation_factor_ / ((double) ticks_) * ((double) getRawPosition()));
}

long VexMotorEncoder::getRawPosition()
{
  uint32_t pos = 0;
  I2C* i2c = I2C::instance(BTR_VEXIMU_PORT_I2C, false);
  i2c->read(addr_, uint8_t(I2CENCODER_POSITION_REGISTER), &pos);

  long position = pos;
  return (is_reversed_ ? -position : position);
}

void VexMotorEncoder::zero()
{
  accessRegister(I2CENCODER_ZERO_REGISTER);
}

void VexMotorEncoder::unTerminate()
{
  accessRegister(I2CENCODER_UNTERMINATE_REGISTER);
}

void VexMotorEncoder::terminate()
{
  accessRegister(I2CENCODER_TERMINATE_REGISTER);
}

int VexMotorEncoder::getAddress()
{
  return addr_;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

void VexMotorEncoder::accessRegister(uint8_t reg)
{
  I2C* i2c = I2C::instance(BTR_VEXIMU_PORT_I2C, false);
  // TODO what is the value supposed to be here?
  i2c->write(addr_, reg, 0);
}

} // namespace btr

#endif // BTR_VL53L0X_ENABLED > 0
