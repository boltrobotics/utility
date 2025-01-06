// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// Based on code by Pololu.

// SYSTEM INCLUDES
#include <stddef.h>

// PROJECT INCLDUES
#include "utility/common/defines.hpp"
#include "utility/common/vl53l0x.hpp"
#include "utility/common/i2c.hpp"
#include "utility/common/time.hpp"

#if BTR_VL53L0X_ENABLED > 0

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

VL53L0X::VL53L0X()
  :
    addr_(BTR_VL53L0X_ADDR_DFLT),
    stop_var_(0),
    timing_budget_us_(0)
{
}

//============================================= OPERATIONS =========================================

int VL53L0X::init(bool io_2v8)
{
  // Sensor uses 1V8 mode for I/O by default.

  if (io_2v8) {
    uint8_t v = readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (v | 0x01)); // set bit 0
  }

  // Set I2C standard mode.
  writeReg(0x88, 0x00);

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  stop_var_ = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks.
  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // Set range signal rate limit to 0.25 MCPS (million counts per second).
  setSignalRateLimit(0.25);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  uint8_t spad_count;
  bool spad_type_is_aperture;

  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) {
    return -1;
  }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++) {
    if (i < first_spad_to_enable || spads_enabled == spad_count) {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
      spads_enabled++;
    }
  }

  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // Apply default tuning settings.

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);

  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);

  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);

  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);

  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);

  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);

  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);

  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);

  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // Set interrupt config to new sample ready.
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  // Active low.
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  timing_budget_us_ = getMeasurementTimingBudget();

  // Disable MSRC and TCC by default.
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // Recalculate timing budget.
  setMeasurementTimingBudget(timing_budget_us_);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);

  if (!performSingleRefCalibration(0x40)) {
    return -1;
  }

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);

  if (!performSingleRefCalibration(0x00)) {
    return -1;
  }

  // Restore the previous sequence config.
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  return 0;
}

void VL53L0X::setAddress(uint8_t addr)
{
  addr_ = addr;
  writeReg(I2C_SLAVE_DEVICE_ADDRESS, addr_ & 0x7F);
}

uint8_t VL53L0X::getAddress()
{ 
  return addr_; 
}

void VL53L0X::writeReg(uint8_t reg, uint8_t value)
{
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->write(addr_, reg, value);
}

void VL53L0X::writeReg16Bit(uint8_t reg, uint16_t value)
{
  uint8_t* buff = reinterpret_cast<uint8_t*>(&value);
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->write(addr_, reg, buff, sizeof(value));
}

void VL53L0X::writeReg32Bit(uint8_t reg, uint32_t value)
{
  uint8_t* buff = reinterpret_cast<uint8_t*>(&value);
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->write(addr_, reg, buff, sizeof(value));
}

uint8_t VL53L0X::readReg(uint8_t reg)
{
  uint8_t value = 0;
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->read(addr_, reg, &value);
  return value;
}

uint16_t VL53L0X::readReg16Bit(uint8_t reg)
{
  uint16_t value = 0;
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->read(addr_, reg, &value);
  return value;
}

uint32_t VL53L0X::readReg32Bit(uint8_t reg)
{
  uint32_t value = 0;
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->read(addr_, reg, &value);
  return value;
}

void VL53L0X::writeMulti(uint8_t reg, uint8_t* src, uint8_t count)
{
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->write(addr_, reg, src, count);
}

void VL53L0X::readMulti(uint8_t reg, uint8_t* dst, uint8_t count)
{
  I2C* i2c = I2C::instance(BTR_VL53L0X_PORT_I2C, false);
  i2c->read(addr_, reg, dst, count);
}

bool VL53L0X::setSignalRateLimit(float limit_mcps)
{
  if (limit_mcps < BTR_VL53L0X_LIMIT_MCPS_MIN || limit_mcps > BTR_VL53L0X_LIMIT_MCPS_MAX) {
    return false;
  }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_mcps * (1 << 7));
  return true;
}

float VL53L0X::getSignalRateLimit()
{
  return (float) readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t const MinTimingBudget    = 20000;

  if (budget_us < MinTimingBudget) {
    return false;
  }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  } else if (enables.msrc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us) {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range) {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    timing_budget_us_ = budget_us;
  }
  return true;
}

uint32_t VL53L0X::getMeasurementTimingBudget()
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  } else if (enables.msrc) {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  timing_budget_us_ = budget_us;
  return budget_us;
}

bool VL53L0X::setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = BTR_VL53L0X_ENCODE_VCSEL(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."

  if (type == VcselPeriodPreRange) {
    // Set phase check limits.
    switch (period_pclks) {
      case 12:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;
      case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;
      case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;
      case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;
      default:
        return false;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // Apply new VCSEL period.
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // Update timeouts.
    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

  } else if (type == VcselPeriodFinalRange) {
    switch (period_pclks) {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;
      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;
      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;
      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;
      default:
        return false;
    }

    // Apply new VCSEL period.
    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // Update timeouts.

    // For the final range timeout, the pre-range timeout must be added. To do this both final
    // and pre-range timeouts must be expressed in macro periods MClks because they have
    // different vcsel periods.
    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range) {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));
  } else {
    return false;
  }

  // Re-apply the timing budget.
  setMeasurementTimingBudget(timing_budget_us_);

  // Perform phase calibration. This is needed after changing VCSEL period.
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  return true;
}

uint8_t VL53L0X::getVcselPulsePeriod(VcselPeriodType type)
{
  if (type == VcselPeriodPreRange) {
    return BTR_VL53L0X_DECODE_VCSEL(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  } else if (type == VcselPeriodFinalRange) {
    return BTR_VL53L0X_DECODE_VCSEL(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  } else {
    return 255;
  }
}

void VL53L0X::startContinuous(uint32_t period)
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_var_);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period != 0) {
    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0) {
      period *= osc_calibrate_val;
    }

    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period);

    // VL53L0X_REG_SYSRANGE_MODE_TIMED
    writeReg(SYSRANGE_START, 0x04);
  } else {
    // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    writeReg(SYSRANGE_START, 0x02);
  }
}

void VL53L0X::stopContinuous()
{
  // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
  writeReg(SYSRANGE_START, 0x01);

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, 0x00);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
}

uint16_t VL53L0X::readRangeContinuousMillimeters()
{
  uint32_t tm = MILLIS();

  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (IS_TIMEOUT(BTR_VL53L0X_TIMEOUT_MS, tm)) {
      set_status(dev::status(), BTR_DEV_ETIMEOUT);
    }
  }

  // Assumptions: Linearity Corrective Gain is 1000 (default). Fractional ranging is not enabled.
  uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return (range + BTR_VL53L0X_COMPENSATE_MM);
}

uint16_t VL53L0X::readRangeSingleMillimeters()
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_var_);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(SYSRANGE_START, 0x01);

  uint32_t tm = MILLIS();

  // Wait until start bit has been cleared.
  while (readReg(SYSRANGE_START) & 0x01) {
    if (IS_TIMEOUT(BTR_VL53L0X_TIMEOUT_MS, tm)) {
      set_status(dev::status(), BTR_DEV_ETIMEOUT);
    }
  }
  return readRangeContinuousMillimeters();
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

bool VL53L0X::getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);

  uint32_t tm = MILLIS();

  while (readReg(0x83) == 0x00) {
    if (IS_TIMEOUT(BTR_VL53L0X_TIMEOUT_MS, tm)) {
      return false;
    }
  }

  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void VL53L0X::getSequenceStepTimeouts(
    SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range) {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

uint16_t VL53L0X::decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t VL53L0X::encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  return 0;
}

uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = BTR_VL53L0X_CALC_PERIOD(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = BTR_VL53L0X_CALC_PERIOD(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte)
{
  // VL53L0X_REG_SYSRANGE_MODE_START_STOP
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte);

  uint32_t tm = MILLIS();

  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (IS_TIMEOUT(BTR_VL53L0X_TIMEOUT_MS, tm)) {
      return false; 
    }
  }

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(SYSRANGE_START, 0x00);
  return true;
}

} // namespace btr

#endif // BTR_VL53L0X_ENABLED > 0
