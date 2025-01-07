// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

/** @file */

#ifndef _utility_btr_Defines_hpp_
#define _utility_btr_Defines_hpp_

// SYSTEM INCLUDES
#define _STDC_FORMAT_MACROS
#include <inttypes.h>
#include <errno.h>

namespace btr
{

//==================================================================================================
// General {

/** Macro enables encoding/decoding of floating-point numbers. @see ValueCodec */
#ifndef BTR_FLOAT_ENABLED
#if BTR_X86 > 0
#define BTR_FLOAT_ENABLED     1
#else
#define BTR_FLOAT_ENABLED     0
#endif
#endif

#if BTR_AVR > 0 || BTR_ARD > 0
//! @cond Suppress_Doxygen_Warning
#include <avr/io.h>
//! @endcond

// BV was renamed to _BV. _BV is not in C standard. For portability, use old-style BV macro.
#define BV(bit)                 (1 << bit)
#define set_bit(sfr, bit)       (_SFR_BYTE(sfr) |= BV(bit))  // old sbi()
#define clear_bit(sfr, bit)     (_SFR_BYTE(sfr) &= ~BV(bit)) // old cbi()
#define toggle_bit(sfr, bit)    (_SFR_BYTE(sfr) ^= BV(bit))  
#define set_reg(reg, val)       (reg = val)

#endif // BTR_AVR > 0 || BTR_ARD > 0

// } General

//==================================================================================================
// Status {

/** Enable/disable status-handling code in this library. */
#ifndef BTR_STATUS_ENABLED
#define BTR_STATUS_ENABLED    1
#endif

#define BTR_IS_OK(v)            ((v & 0xFFFF0000) == 0)
#define BTR_IS_ERR(v)           ((v & 0xFFFF0000) != 0)
#define BTR_IS_SET(s,v)         (v == (s & v))
#define BTR_SET_STATUS(s,v)     (s |= (v & 0xFFFF0000))
#define BTR_CLEAR_STATUS(s)     (s &= ~(0xFFFF0000))

/** Returns this module's accumulated status or nullptr if BTR_STATUS_ENABLED is 0. The status is 
 * stored in upper 16 bits, the lower 16 bits are reserved for storing system errors. */
uint32_t* status();

inline bool is_ok(uint32_t v)    { return BTR_IS_OK(v); }
inline bool is_err(uint32_t v)   { return BTR_IS_ERR(v); }

#if BTR_STATUS_ENABLED > 0
inline bool is_ok(uint32_t* v)                  { return BTR_IS_OK(*v); }
inline bool is_err(uint32_t* v)                 { return BTR_IS_ERR(*v); }
inline bool is_set(uint32_t* s, uint32_t v)     { return BTR_IS_SET(*s, v); }
inline void set_status(uint32_t* s, uint32_t v) { BTR_SET_STATUS(*s, v); }
inline void clear_status(uint32_t* s)           { BTR_CLEAR_STATUS(*s); }
#else
inline bool is_ok(uint32_t*)                    { return true; }
inline bool is_err(uint32_t*)                   { return false; }
inline bool is_set(uint32_t*, uint32_t)         { return false; }
inline void set_status(uint32_t*, uint32_t)     {}
inline void clear_status(uint32_t*)             {}
#endif

// This library's status codes.
#define BTR_ENOERR          0x00000000
#define BTR_ENODATA         0x00010000
#define BTR_EOVERFLOW       0x00020000
#define BTR_EPARITY         0x00040000
#define BTR_EOVERRUN        0x00080000
#define BTR_EFRAME          0x00100000
#define BTR_ETIMEOUT        0x00110000
#define BTR_ENOTOPEN        0x00120000
#define BTR_ESTART          0x00140000
#define BTR_ESENDBYTE       0x00180000
#define BTR_ERECVBYTE       0x00200000
#define BTR_ENOACK          0x00210000
#define BTR_ENONACK         0x00220000
#define BTR_EINVAL          0x00240000
#define BTR_EINIT           0x00280000
#define BTR_EFAIL           0x00400000
#define BTR_ENOMEM          0x00410000

// } Status

//==================================================================================================
// Log {

#if 0
enum BTR_LOG_LEVEL {
  TRACE     = 0,
  DEBUG     = 1,
  INFO      = 2,
  WARN      = 3,
  ERROR     = 4,
  CRITICAL  = 5,
  LOGS_OFF  = 6
};

#ifndef BTR_LOG_ENABLED
#if BTR_X86 > 0
#define BTR_LOG_ENABLED         1
#else
#define BTR_LOG_ENABLED         0
#endif
#endif

#if !defined(BTR_LOG_PORT_USB) && !defined(BTR_LOG_PORT_USART)
#define BTR_LOG_PORT_USB        1
#endif

#ifndef BTR_LOG_MAX
#define BTR_LOG_MAX             128
#endif
#ifndef BTR_LOG_MAX_STRING
#define BTR_LOG_MAX_STRING      96
#endif
#ifndef BTR_LOG_MAX_HEX
#define BTR_LOG_MAX_HEX         96
#endif
#ifndef BTR_LOG_LEVEL_DFLT
#define BTR_LOG_LEVEL_DFLT      btr::WARN
#endif

#endif

// } Log

//==================================================================================================
// Time {

#ifndef BTR_TIME_ENABLED 
#if BTR_ESP32 > 0 || BTR_STM32 > 0 || BTR_AVR > 0 || BTR_X86 > 0
#define BTR_TIME_ENABLED        1
#else
#define BTR_TIME_ENABLED        0
#endif // #if BTR_ESP32 > 0 || BTR_STM32 > 0 || BTR_AVR > 0 || BTR_X86 > 0
#endif // #ifndef BTR_TIME_ENABLED 

#if BTR_ESP32 > 0 || BTR_STM32 > 0 || BTR_AVR > 0
#define MILLIS()                (Time::millis())
#define SEC()                   (Time::sec())
#define TIME_DIFF(a,b)          (Time::diff(a, b))
#endif // #if BTR_ESP32 > 0 || BTR_STM32 > 0 || BTR_AVR > 0

/** Check if timeout is greater than 0, if so, check if time window has expired. */
#define IS_TIMEOUT(timeout_ms, start_ms) \
  (timeout_ms > 0 && (TIME_DIFF(MILLIS(), start_ms) > timeout_ms))

// } Time

//==================================================================================================
// LED {

#if BTR_STM32 > 0

#if !defined(BTR_BLUE_PILL) && !defined(BTR_BLACK_PILL)
#define BTR_BLUE_PILL         1
#endif

// The LED in BlackPill use GPIOB, GPIO12, in BluePill GPIOC, GPIO13
#ifndef BTR_BUILTIN_LED_CLK
#ifdef BTR_BLACK_PILL
#define BTR_BUILTIN_LED_CLK   RCC_GPIOB
#else
#define BTR_BUILTIN_LED_CLK   RCC_GPIOC
#endif // BTR_BLACK_PILL
#endif // BTR_BUILTIN_LED_CLK

#ifndef BTR_BUILTIN_LED_PORT
#ifdef BTR_BLACK_PILL
#define BTR_BUILTIN_LED_PORT  GPIOB
#else
#define BTR_BUILTIN_LED_PORT  GPIOC
#endif // BTR_BLACK_PILL
#endif // BTR_BUILTIN_LED_PORT

#ifndef BTR_BUILTIN_LED_PIN
#ifdef BTR_BLACK_PILL
#define BTR_BUILTIN_LED_PIN   GPIO12
#else
#define BTR_BUILTIN_LED_PIN   GPIO13
#endif // BTR_BLACK_PILL
#endif // BTR_BUILTIN_LED_PIN

#define LED_ENABLE() \
  do { \
    rcc_periph_clock_enable(BTR_BUILTIN_LED_CLK); \
    gpio_set_mode(BTR_BUILTIN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
        GPIO_CNF_OUTPUT_PUSHPULL, BTR_BUILTIN_LED_PIN); \
  } while (0);

#define LED_ON() (gpio_clear(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_OFF() (gpio_set(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))
#define LED_TOGGLE() (gpio_toggle(BTR_BUILTIN_LED_PORT, BTR_BUILTIN_LED_PIN))

#elif BTR_AVR > 0

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB5
#define BTR_BUILTIN_LED_DDR   DDRB
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_BUILTIN_LED_PORT  PORTB
#define BTR_BUILTIN_LED_PIN   PB7
#define BTR_BUILTIN_LED_DDR   DDRB
#else
#warning AVR device is not supported
#endif

#define LED_ON()  \
  do { \
    BTR_BUILTIN_LED_DDR  |= (1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT |= (1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#define LED_OFF() \
  do { \
    BTR_BUILTIN_LED_DDR  &= ~(1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT &= ~(1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#define LED_TOGGLE() \
  do { \
    BTR_BUILTIN_LED_DDR  ^= (1 << BTR_BUILTIN_LED_PIN); \
    BTR_BUILTIN_LED_PORT ^= (1 << BTR_BUILTIN_LED_PIN); \
  } while (0);
#endif // stm32, avr, ard

// } LED

//==================================================================================================
// I2C {

/** On STM32F103C8T6, I2C0 refers to SCL1/SDA1, I2C1 to SCL2/SDA2. On AVR, only I2C0 is used. */
#ifndef BTR_I2C0_ENABLED
#define BTR_I2C0_ENABLED      0
#endif
#ifndef BTR_I2C1_ENABLED
#define BTR_I2C1_ENABLED      0
#endif

#if BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

//--------------------------------------------------------------------------------------------------
// AVR

#if BTR_AVR > 0

// WRITE/READ constants are taken from /usr/lib/avr/include/util/twi.h
/** I2C write operation. */
#define BTR_I2C_WRITE               0
/** I2C read operation. */
#define BTR_I2C_READ                1

//--------------------------------------------------------------------------------------------------
// ESP32

#elif BTR_ESP32 > 0

/** Default clock doesn't work well when power management is enabled. Change it to fix */
#ifndef BTR_I2C_CLK_SRC 
#define BTR_I2C_CLK_SRC             I2C_CLK_SRC_DEFAULT // I2C_CLK_SRC_APB
#endif
#ifndef BTR_I2C_MASTER_PORT
#define BTR_I2C_MASTER_PORT         0
#endif
#ifndef BTR_I2C_MASTER_SCL_IO       
#define BTR_I2C_MASTER_SCL_IO       12 // Default 22
#endif
#ifndef BTR_I2C_MASTER_SDA_IO
#define BTR_I2C_MASTER_SDA_IO       13 // Default 21
#endif
#ifndef BTR_I2C_GLITCH_IGNORE_COUNT
#define BTR_I2C_GLITCH_IGNORE_COUNT 7
#endif
#ifndef BTR_I2C_INTERNAL_PULLUP
#define BTR_I2C_INTERNAL_PULLUP     true
#endif

/** Dummy I2C write operation. Discarded in ESP32 */
#define BTR_I2C_WRITE               0 
/** Dummy I2C read operation. Discarded in ESP32 */
#define BTR_I2C_READ                1

//--------------------------------------------------------------------------------------------------
// STM32

#elif BTR_STM32 > 0
#include <libopencm3/stm32/i2c.h>

/** Define I2C write operation. */
#define BTR_I2C_WRITE               I2C_WRITE 
/** Define I2C read operation. */
#define BTR_I2C_READ                I2C_READ

//--------------------------------------------------------------------------------------------------
// Other

#else

/** Define I2C write operation. */
#define BTR_I2C_WRITE               0 
/** Define I2C read operation. */
#define BTR_I2C_READ                1

#endif // # AVR, ESP32, STM32, Other

//--------------------------------------------------------------------------------------------------

#ifndef BTR_I2C_IO_TIMEOUT_MS
#define BTR_I2C_IO_TIMEOUT_MS       100
#endif

/** Warn if time-out is zero. */
#if BTR_I2C_IO_TIMEOUT_MS == 0
#error "BTR_I2C_IO_TIMEOUT_MS is 0"
#endif

/** Speed 100kHz or 400kHz. */
#ifndef BTR_I2C_SPEED
#define BTR_I2C_SPEED               100000
#endif

/** Enable internal pull-ups */
#ifndef BTR_I2C_INTERNAL_PULLUP
#define BTR_I2C_INTERNAL_PULLUP     true
#endif

/** Maximum number of devices to scan. Absolute maximum is UINT8_MAX. */
#ifndef BTR_I2C_SCAN_MAX
#define BTR_I2C_SCAN_MAX            128
#endif

#define BTR_I2C_WRITE_ADDR(addr)    (addr << 1)
#define BTR_I2C_READ_ADDR(addr)     ((addr << 1) + 1)

#endif // BTR_I2C0_ENABLED > 0 || BTR_I2C1_ENABLED > 0

// } I2C

//==================================================================================================
// USART/USB {

/** Supported parity types. */
typedef enum
{
  NONE,
  ODD,
  EVEN
} ParityType;

#define PARITY_FROM_INT(v) ((v==1 ? \
      btr::ParityType::ODD : (v==2 ? btr::ParityType::EVEN : btr::ParityType::NONE)))

/** Supported stop bit types. */
typedef enum
{
  ONE,
  ONEPOINTFIVE,
  TWO
} StopBitsType;

#define TO_STOP_BITS(v) ((v==2 ? btr::StopBitsType::TWO : btr::StopBitsType::ONE))

/** Data flow direction. */
typedef enum
{
  IN,
  OUT,
  INOUT
} DirectionType;

//--------------------------------------------------------------------------------------------------

#if BTR_STM32 > 0

#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        2

#elif BTR_AVR > 0 || BTR_ARD > 0

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        0
#elif defined(__AVR__ATmega1280__) || defined(__AVR_ATmega2560__)
#define BTR_USART_MIN_ID        0
#define BTR_USART_MAX_ID        3
#endif // AVR board

#endif // BTR_STM32 > 0

//--------------------------------------------------------------------------------------------------

#ifndef BTR_USART0_PORT
#define BTR_USART0_PORT         "/dev/ttyACM0"
#endif
#ifndef BTR_USART1_PORT
#define BTR_USART1_PORT         "/dev/ttyACM1"
#endif
#ifndef BTR_USART2_PORT
#define BTR_USART2_PORT         "/dev/ttyACM2"
#endif
#ifndef BTR_USART3_PORT
#define BTR_USART3_PORT         "/dev/ttyACM3"
#endif

#ifndef BTR_USB0_ENABLED
#define BTR_USB0_ENABLED        0
#endif
#ifndef BTR_USART0_ENABLED
#define BTR_USART0_ENABLED      0
#endif
#ifndef BTR_USART1_ENABLED
#define BTR_USART1_ENABLED      0
#endif
#ifndef BTR_USART2_ENABLED
#define BTR_USART2_ENABLED      0
#endif
#ifndef BTR_USART3_ENABLED
#define BTR_USART3_ENABLED      0
#endif

#ifndef BTR_USART_RX_DELAY_US
#define BTR_USART_RX_DELAY_US   5000
#endif
#ifndef BTR_USART_TX_DELAY_US
#define BTR_USART_TX_DELAY_US   5000
#endif
#define BTR_USART_TX_DELAY_MS   (BTR_USART_TX_DELAY_US / 1000) 
#define BTR_USART_RX_DELAY_MS   (BTR_USART_RX_DELAY_US / 1000) 

#ifndef BTR_USART_IO_TIMEOUT_MS
#define BTR_USART_IO_TIMEOUT_MS 100
#endif
#ifndef BTR_USART_RX_TIMEOUT_MS
#define BTR_USART_RX_TIMEOUT_MS BTR_USART_IO_TIMEOUT_MS
#endif
#ifndef BTR_USART_TX_TIMEOUT_MS
#define BTR_USART_TX_TIMEOUT_MS BTR_USART_IO_TIMEOUT_MS
#endif

#ifndef BTR_USART_RX_BUFF_SIZE
#define BTR_USART_RX_BUFF_SIZE  64
#endif
#ifndef BTR_USART_TX_BUFF_SIZE
#define BTR_USART_TX_BUFF_SIZE  64
#endif
#ifndef BTR_USART_IR_BUFF_SIZE
#define BTR_USART_IR_BUFF_SIZE  16
#endif
/** USB doesn't work with lower values like 64 bytes. Keep it at 128. */
//#ifndef BTR_USART_CR_BUFF_SIZE
#define BTR_USART_CR_BUFF_SIZE  128
//#endif

#ifndef BTR_USART_USE_2X
#define BTR_USART_USE_2X        1
#endif

#ifndef BTR_USART0_BAUD
#define BTR_USART0_BAUD         115200
#endif
#ifndef BTR_USART1_BAUD
#define BTR_USART1_BAUD         115200
#endif
#ifndef BTR_USART2_BAUD
#define BTR_USART2_BAUD         115200
#endif
#ifndef BTR_USART3_BAUD
#define BTR_USART3_BAUD         115200
#endif

#ifndef BTR_USART0_DATA_BITS
#define BTR_USART0_DATA_BITS    8
#endif
#ifndef BTR_USART1_DATA_BITS
#define BTR_USART1_DATA_BITS    8
#endif
#ifndef BTR_USART2_DATA_BITS
#define BTR_USART2_DATA_BITS    8
#endif
#ifndef BTR_USART3_DATA_BITS
#define BTR_USART3_DATA_BITS    8
#endif

#ifndef BTR_USART0_PARITY
#define BTR_USART0_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART1_PARITY
#define BTR_USART1_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART2_PARITY
#define BTR_USART2_PARITY       ParityType::NONE
#endif
#ifndef BTR_USART3_PARITY
#define BTR_USART3_PARITY       ParityType::NONE
#endif

#ifndef BTR_USART0_STOP_BITS
#define BTR_USART0_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART1_STOP_BITS
#define BTR_USART1_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART2_STOP_BITS
#define BTR_USART2_STOP_BITS    StopBitsType::ONE
#endif
#ifndef BTR_USART3_STOP_BITS
#define BTR_USART3_STOP_BITS    StopBitsType::ONE
#endif

#ifndef BTR_USART0_RTS
#define BTR_USART0_RTS          0
#endif
#ifndef BTR_USART1_RTS
#define BTR_USART1_RTS          0
#endif
#ifndef BTR_USART2_RTS
#define BTR_USART2_RTS          0
#endif
#ifndef BTR_USART3_RTS
#define BTR_USART3_RTS          0
#endif

#ifndef BTR_USART0_CTS
#define BTR_USART0_CTS          0
#endif
#ifndef BTR_USART1_CTS
#define BTR_USART1_CTS          0
#endif
#ifndef BTR_USART2_CTS
#define BTR_USART2_CTS          0
#endif
#ifndef BTR_USART3_CTS
#define BTR_USART3_CTS          0
#endif

#define BTR_USART_CONFIG(parity, stop_bits, data_bits) \
  ( (parity == ParityType::NONE ? 0 : (parity == ParityType::EVEN ? 32 : 48)) \
    + (stop_bits == StopBitsType::ONE ? 0 : 8) + (2 * (data_bits - 5)) )

// } USART/USB

} // namespace btr

#endif // _utility_btr_Defines_hpp_
