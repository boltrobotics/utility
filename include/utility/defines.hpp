// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _utility_btr_Defines_hpp_
#define _utility_btr_Defines_hpp_

// SYSTEM INCLUDES
#include <inttypes.h>
#include <errno.h>

namespace btr
{

/** Macro enables encoding/decoding of floating-point numbers. @see ValueCodec */
#ifndef BTR_FLOAT_ENABLED
#if BTR_X86 > 0
#define BTR_FLOAT_ENABLED     1
#else
#define BTR_FLOAT_ENABLED     0
#endif
#endif

/** Enable/disable status-handling code in this library. */
#ifndef BTR_STATUS_ENABLED
#define BTR_STATUS_ENABLED    1
#endif

#define BTR_OK_I(v)             ((v & 0xFFFF0000) == 0)
#define BTR_ERR_I(v)            ((v & 0xFFFF0000) != 0)
#define BTR_IS_SET(s,v)         (v == (s & v))
#define BTR_ESET_I(s,v)         (s |= (v & 0xFFFF0000))

inline bool is_ok(uint32_t v)    { return BTR_OK_I(v); }
inline bool is_err(uint32_t v)   { return BTR_ERR_I(v); }

#if BTR_STATUS_ENABLED > 0
inline bool is_ok(uint32_t* v)   { return BTR_OK_I(*v); }
inline bool is_err(uint32_t* v)  { return BTR_ERR_I(*v); }
inline bool is_set(uint32_t* s, uint32_t v) { return BTR_IS_SET(*s, v); }
inline void set_status(uint32_t* s, uint32_t v) { BTR_ESET_I(*s, v); }
inline void clear_status(uint32_t* s) { *s &= ~(0xFFFF0000); }
#else
inline bool is_ok(uint32_t*)   { return true; }
inline bool is_err(uint32_t*)  { return false; }
inline bool is_set(uint32_t*, uint32_t) { return false; };
inline void set_status(uint32_t*, uint32_t)  {}
inline void clear_status(uint32_t*) {}
#endif

} // namespace btr

#endif // _utility_btr_Defines_hpp_
