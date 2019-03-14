// Copyright (C) 2019 Bolt Robotics <info@boltrobotics.com>
// License: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

/** @file */

#ifndef _btr_Defines_hpp_
#define _btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>
#include <errno.h>

namespace btr
{

/** Macro enables encoding/decoding of floating-point numbers. @see ValueCodec */
#ifndef BTR_FLOAT_ENABLED
#define BTR_FLOAT_ENABLED     1
#endif

} // namespace btr

#endif // _btr_Defines_hpp_
