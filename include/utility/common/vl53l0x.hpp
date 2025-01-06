// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

// Based on code by Pololu.

#ifndef _btr_VL53L0X_hpp_
#define _btr_VL53L0X_hpp_

// PROJECT INCLUDES
#include "utility/common/defines.hpp"

/** When enabling VL53L0X, also set BTR_I2C0_ENABLED. */
#ifndef BTR_VL53L0X_ENABLED
#define BTR_VL53L0X_ENABLED         0
#endif

#ifndef BTR_VL53L0X_PORT_I2C
#if BTR_STM32 > 0
#define BTR_VL53L0X_PORT_I2C             1
#else
#define BTR_VL53L0X_PORT_I2C             0
#endif
#endif // BTR_VL53L0X_PORT_I2C_DEV

#ifndef BTR_VL53L0X_ADDR_DFLT
#define BTR_VL53L0X_ADDR_DFLT       0b0101001
#endif
#ifndef BTR_VL53L0X_COMPENSATE_MM
#define BTR_VL53L0X_COMPENSATE_MM   -10
#endif
#ifndef BTR_VL53L0X_TIMEOUT_MS
#define BTR_VL53L0X_TIMEOUT_MS      1000
#endif
#ifndef BTR_VL53L0X_BUDGET_US
#define BTR_VL53L0X_BUDGET_US       32000
#endif
#ifndef BTR_VL53L0X_LIMIT_MCPS_MIN
#define BTR_VL53L0X_LIMIT_MCPS_MIN  0
#endif
#ifndef BTR_VL53L0X_LIMIT_MCPS_MAX
#define BTR_VL53L0X_LIMIT_MCPS_MAX  511.99
#endif

/** Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register. */
#define BTR_VL53L0X_DECODE_VCSEL(val) (((val) + 1) << 1)
/** Encode VCSEL pulse period register value from period in PCLKs. */
#define BTR_VL53L0X_ENCODE_VCSEL(val) (((val) >> 1) - 1)
/** Calculate macro period in *nanoseconds* from VCSEL period in PCLKs.
 * PLL_period_ps = 1655; macro_period_vclks = 2304. */
#define BTR_VL53L0X_CALC_PERIOD(val) ((((uint32_t) 2304 * (val) * 1655) + 500) / 1000)

namespace btr
{

/**
 * The class provides an interface to VL53L0X device.
 *
 * @see STSW-IMG005 Databrief
 * @see DB2903 VL53L0X datasheet
 * @see UM2039 User Manual
 */
class VL53L0X
{
public:

  /** I2C register addresses. */
  enum RegAddrs {
    SYSRANGE_START                              = 0x00,

    SYSTEM_THRESH_HIGH                          = 0x0C,
    SYSTEM_THRESH_LOW                           = 0x0E,

    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_RANGE_CONFIG                         = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,

    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

    ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

    MSRC_CONFIG_CONTROL                         = 0x60,

    PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

    FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

    SYSTEM_HISTOGRAM_BIN                        = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

    SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    IDENTIFICATION_REVISION_ID                  = 0xC2,

    OSC_CALIBRATE_VAL                           = 0xF8,

    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
  };

  enum VcselPeriodType {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
  };

// LIFECYCLE

  /**
   * Set data members to 0; salve address is set to BTR_VL53LOX_ADDR_DFLT.
   */
  VL53L0X();
  ~VL53L0X() = default;

// OPERATIONS

  /**
   * Initialize VL53L0X device.
   *
   * This function does not perform reference SPAD calibration since the ST performs it on bare
   * modules. The device has to be recalibrated if cover glass is added.
   *
   * @param io_2v8 - configured for 2V8 mode if true, otherwise configure for 1V8 mode
   * @return 0 if initialized, -1 otherwise
   */
  int init(bool io_2v8 = true);

  /**
   * Set new I2C slave address and write the target register.
   *
   * @param new_addr - new I2C slave address
   */
  void setAddress(uint8_t new_addr);

  /**
   * @return slave address
   */
  uint8_t getAddress();

  /**
   * Write 8-bit register.
   *
   * @param reg - register to write
   * @param value - value to write
   */
  void writeReg(uint8_t reg, uint8_t value);

  /**
   * Write 16-bit register.
   *
   * @param reg - register to write
   * @param value - value to write
   */
  void writeReg16Bit(uint8_t reg, uint16_t value);

  /**
   * Write 32-bit register.
   *
   * @param reg - register to write
   * @param value - value to write
   */
  void writeReg32Bit(uint8_t reg, uint32_t value);

  /**
   * Read 8-bit register.
   *
   * @param reg - register to read
   * @return value
   */
  uint8_t readReg(uint8_t reg);

  /**
   * Read 16-bit register.
   *
   * @param reg - register to read
   * @return value
   */
  uint16_t readReg16Bit(uint8_t reg);

  /**
   * Read 32-bit register.
   *
   * @param reg - register to read
   * @return value
   */
  uint32_t readReg32Bit(uint8_t reg);

  /**
   * Write a number of bytes to the sensor, starting at the given register.
   *
   * @param reg - register address
   * @param src - data source
   * @param count - number of bytes
   */
  void writeMulti(uint8_t reg, uint8_t* src, uint8_t count);

  /**
   * Read a number of bytes from the sensor, starting at the given register.
   *
   * @param reg - register address
   * @param dst - destination buffer to write to
   * @param count - the number of bytes to read
   */
  void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

  /**
   * Set the return signal rate limit check value in units of MCPS (mega counts per second).
   *
   * This represents the amplitude of the signal reflected from the target and detected by the
   * device; setting this limit presumably determines the minimum measurement necessary for the
   * sensor to report a valid reading. Setting a lower limit increases the potential range of the
   * sensor but also seems to increase the likelihood of getting an inaccurate reading because of
   * unwanted reflections from objects other than the intended target. Defaults to 0.25 MCPS as
   * initialized by the ST API and this library.
   *
   * @param limit_mcps - the limit
   * @return true if limit is applied, false otherwise
   */
  bool setSignalRateLimit(float limit_mcps);

  /**
   * @return signal rate limit check value in MCPS.
   */
  float getSignalRateLimit();

  /**
   * Set the measurement timing budget, which is the time allowed for one measurement.
   *
   * ST API and this library takes care of splitting the timing budget among the sub-steps in the
   * ranging sequence. A longer timing budget allows for more accurate measurements. Increasing
   * the budget by a factor of N decreases the range measurement standard deviation by a factor of
   * sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
   *
   * @param budget - timing budget in microseconds
   * @return  true if the badget was applied
   */
  bool setMeasurementTimingBudget(uint32_t budget);

  /**
   * @return measurement timing budget in microseconds
   */
  uint32_t getMeasurementTimingBudget();

  /**
   * Set VCSEL (vertical cavity surface emitting laser) pulse period for the given period type
   * (pre-range or final range) to the given value in PCLKs.
   *
   * Longer periods seem to increase the potential range of the sensor. Valid values are (even
   * numbers only):
   *  pre:  12 to 18 (initialized default: 14)
   *  final: 8 to 14 (initialized default: 10)
   *
   * @param type - period type
   * @param period_pclks
   */
  bool setVcselPulsePeriod(VcselPeriodType type, uint8_t period_pclks);

  /**
   * Provide VCSEL pulse period for a given period type.
   *
   * @param type - period type
   * @return pulse period  in PCLKs
   */
  uint8_t getVcselPulsePeriod(VcselPeriodType type);

  /**
   * Start continuous ranging measurements.
   *
   * If period is 0, continuous back-to-back mode is used, i.e. the sensor takes measurements as
   * often as possible. Otherwise, continuous timed mode is used. The given inter-measurement
   * period determines how often the sensor takes a measurement.
   *
   * @param period - optional period in milliseconds
   */
  void startContinuous(uint32_t period = 0);

  /**
   * Stop continuous measurements.
   */
  void stopContinuous();

  /**
   * Provide range while sensor performs continuous mesaurements.
   * @return range in millimeters or UINT16_MAX on timeout
   */
  uint16_t readRangeContinuousMillimeters();

  /**
   * Performs a single-shot range measurement.
   * @return range in millimeters or UINT16_MAX on timeout
   */
  uint16_t readRangeSingleMillimeters();

private:

  struct SequenceStepEnables
  {
    /** Target CentreCheck. */
    bool tcc;
    /** Minimum Signal Rate Check. */
    bool msrc;
    /** Dynamic Spad Selection. */
    bool dss;
    bool pre_range;
    bool final_range;
  };

  struct SequenceStepTimeouts
  {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
  };

// OPERATIONS

  /**
   * Get reference SPAD (single photon avalanche diode) count and type, but only gets reference
   * SPAD count and type.
   *
   * @param count
   * @param type_is_aperture
   * @return spad info
   */
  bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

  /**
   * Get sequence step enables.
   *
   * @param enables
   */
  void getSequenceStepEnables(SequenceStepEnables* enables);

  /**
   * Get sequence step timeouts.
   * Gets all timeouts instead of just the requested one, and also stores intermediate values
   *
   * @param enables
   * @param timeouts
   */
  void getSequenceStepTimeouts(SequenceStepEnables const* enables, SequenceStepTimeouts* timeouts);

  /**
   * Decode sequence step timeout in MCLKs from register value.
   *
   * Note: the original function returned a uint32_t, but the return value is always stored
   * in a uint16_t.
   *
   * @param value - timeout
   */
  static uint16_t decodeTimeout(uint16_t value);

  /**
   * Encode sequence step timeout register value from timeout in MCLKs.
   *
   * @param timeout_mclks
   * @return timeout
   */
  static uint16_t encodeTimeout(uint16_t timeout_mclks);

  /**
   * Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs.
   *
   * @param timeout_period_mclks
   * @param vcsel_period_pclks
   * @return microseconds
   */
  static uint32_t timeoutMclksToMicroseconds(
      uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

  /**
   * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs.
   *
   * @param timeout_period_us
   * @param vcsel_period_pclks
   * @return mclks
   */
  static uint32_t timeoutMicrosecondsToMclks(
      uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

  /**
   * Perform calibration.
   *
   * @param vhv_init_byte
   */
  bool performSingleRefCalibration(uint8_t vhv_init_byte);

// ATTRIBUTES

  /** Slave address. */
  uint8_t addr_;
  uint8_t stop_var_;
  uint32_t timing_budget_us_;
};

} // namespace btr

#endif // _btr_VL53L0X_hpp_
