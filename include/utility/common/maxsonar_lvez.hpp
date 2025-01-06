// Copyright (C) 2017 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _btr_MaxSonarLvEz_hpp_
#define _btr_MaxSonarLvEz_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>

// PROJECT INCLUDES

// Volts per inch for the LV-MaxSonar-EZ sensors
#define DEFAULT_VOLTS 5
#define VPI           512
#define ADC_PRECISION 1024
#define MM_PER_INCH   25.4
#define COMPENSATE_MM 0

namespace btr
{

/**
 * The class calculates range to an object from a MaxBotix LV-MaxSonar-EZ ultrasonic sensor.
 */
class MaxSonarLvEz
{
public:

// LIFECYCLE

    MaxSonarLvEz() = delete;
    ~MaxSonarLvEz() = delete;

// OPERATIONS

    /**
     * Calculate range in millimeters.
     *
     * @param adc_sample - the analog sample from ADC
     * @param vcc - supplied voltage to ADC. Default 5 volts
     * @return the rnage millimeters
     */
    static uint16_t range(uint16_t adc_sample, uint8_t vcc = DEFAULT_VOLTS);

private:

// OPERATIONS

// ATTRIBUTES

}; // class MaxSonarLvEz

////////////////////////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

inline uint16_t MaxSonarLvEz::range(uint16_t adc_sample, uint8_t vcc)
{
  // Compute analog volts
  float volt = vcc * (static_cast<float>(adc_sample) / ADC_PRECISION);
  // Compute volts per inch
  float volt_inch = static_cast<float>(vcc) / VPI;
  uint16_t range_mm = (volt / volt_inch) * MM_PER_INCH + COMPENSATE_MM;
  return range_mm;
}

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================


} // namespace btr

#endif // _btr_MaxSonarLvEz_hpp_
