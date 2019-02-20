// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

// SYSTEM INCLUDES
#include <math.h>

// PROJECT INCLUDES
#include "utility/value_codec.hpp"  // class implemented

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

//============================================= OPERATIONS =========================================

#if BTR_FLOAT_ENABLED > 0

int ValueCodec::encodeFloatToInt(
    Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  if (0 == encodeFloatToInt(buff->write_ptr(), val_bytes, val, dec_places, msb)) {
    buff->write_ptr() += val_bytes;
    return 0;
  }
  return -1;
}

int ValueCodec::encodeFloatToInt(
    uint8_t* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  if (sizeof(uint8_t) == val_bytes) {
    uint8_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint16_t) == val_bytes) {
    uint16_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint32_t) == val_bytes) {
    uint32_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else if (sizeof(uint64_t) == val_bytes) {
    uint64_t output = round(val * pow(10, dec_places));
    encodeFixedInt(buff, output, msb);
  } else {
    errno = EINVAL;
    return -1;
  }
  return 0;
}

int ValueCodec::encodeFloatToIntParts(
    Buff* buff, uint8_t val_bytes, double val, uint8_t dec_places, bool msb)
{
  double ipart_tmp = 0;
  double fpart_tmp = modf(val, &ipart_tmp);

  if (sizeof(uint8_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint8_t>(ipart_tmp), msb);
  } else if (sizeof(uint16_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint16_t>(ipart_tmp), msb);
  } else if (sizeof(uint32_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint32_t>(ipart_tmp), msb);
  } else if (sizeof(uint64_t) == val_bytes) {
    encodeFixedInt(buff, static_cast<uint64_t>(ipart_tmp), msb);
  } else {
    errno = EINVAL;
    return -1;
  }
  return encodeFloatToInt(buff, val_bytes, fpart_tmp, dec_places, msb);
}

#endif // BTR_FLOAT_ENABLED > 0

/////////////////////////////////////////////// PROTECTED //////////////////////////////////////////

//============================================= OPERATIONS =========================================

/////////////////////////////////////////////// PRIVATE ////////////////////////////////////////////

//============================================= OPERATIONS =========================================

} // namespace btr
