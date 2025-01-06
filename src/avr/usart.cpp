// Copyright (C) 2019 Sergey Kapustin <kapucin@gmail.com>

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

// SYSTEM INCLUDES
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

// PROJECT INCLUDES
#include "utility/common/usart.hpp"  // class implemented

#if BTR_USART0_ENABLED > 0 || BTR_USART1_ENABLED > 0 || \
    BTR_USART2_ENABLED > 0 || BTR_USART3_ENABLED > 0

#if BTR_USART1_ENABLED > 0 || BTR_USART2_ENABLED > 0 || BTR_USART2_ENABLED > 0

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
#error USART ports 1-3 are supported by atmega1280 and atmega2560 only
#endif // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif

#if BTR_USART_USE_2X > 0
#define BAUD_CALC(BAUD) (((F_CPU) + 4UL * (BAUD)) /  (8UL * (BAUD)) - 1UL)
#else
#define BAUD_CALC(BAUD) (((F_CPU) + 8UL * (BAUD)) / (16UL * (BAUD)) - 1UL)
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
// Register bits {

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// UCSRnA (status)
#define RXC       RXC0    // Bit 7. Receive complete
#define TXC       TXC0    // Bit 6. Transmit complete
#define UDRE      UDRE0   // Bit 5. Transmit buffer empty
#define FE        FE0     // Bit 4. Frame error
#define DOR       DOR0    // Bit 3. Data OverRun
#define UPE       UPE0    // Bit 2. Parity error
#define U2X       U2X0    // Bit 1. Double transmission speed
#define MPCM      MPCM0   // Bit 0. Multi-processor communication mode
// UCSRnB (control 1)
#define RXCIE     RXCIE0  // Bit 7. Receive complete interrupt enable
#define TXCIE     TXCIE0  // Bit 6. Transmit complete interrupt enable
#define UDRIE     UDRIE0  // Bit 5. Transmit buffer empty interrupt enable
#define RXEN      RXEN0   // Bit 4. Receive enable
#define TXEN      TXEN0   // Bit 3. Transmit enable
#define UCSZ2     UCSZ02  // Bit 2. Character size 2
// UCSRnC (control 2)
#define UCSZ1     UCSZ01  // Bit 2. Character size 1
#define UCSZ0     UCSZ00  // Bit 1. Character size 0
#endif // __AVR

// } Register bits

////////////////////////////////////////////////////////////////////////////////////////////////////
// RS485 { TODO Finish off

#if BTR_RTS_ENABLED > 0

#define RTS_PIN   PB0
#define RTS_DDR   DDRB
#define RTS_PORT  PORTB

#define RTS_INIT \
  do { \
    set_bit(RTS_DDR, RTS_PIN); \
    clear_bit(RTS_PORT, RTS_PIN); \
  } while (0);

#define RTS_HIGH \
  do { \
    set_bit(RTS_PORT, RTS_PIN); \
  } while (0);

#define RTS_LOW \
  do { \
    clear_bit(RTS_PORT, RTS_PIN); \
  } while (0);

#endif // BTR_RTS_ENABLED

// } RS485

////////////////////////////////////////////////////////////////////////////////////////////////////
// Static members {

#if BTR_USART0_ENABLED > 0
#if defined(UBRRH) && defined(UBRRL)
static btr::Usart usart_0(&UBRRH, &UBRRL, &UCSRA, &UCSRB, &UCSRC, &UDR);
#else
static btr::Usart usart_0(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0);
#endif // UBRRH && UBRRL
#endif
#if BTR_USART1_ENABLED > 0
static btr::Usart usart_1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1);
#endif
#if BTR_USART2_ENABLED > 0
static btr::Usart usart_2(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2);
#endif
#if BTR_USART3_ENABLED > 0
static btr::Usart usart_3(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3);
#endif

static void onRecv(btr::Usart* u)
{
  u->rx_error_ = (*(u->ucsr_a_) & ((1 << FE) | (1 << DOR) | (1 << UPE)));
  uint16_t head_next = (u->rx_head_ + 1) % BTR_USART_RX_BUFF_SIZE;

  if (head_next != u->rx_tail_) {
    u->rx_buff_[u->rx_head_] = *(u->udr_);
    u->rx_head_ = head_next;
  } else {
    u->rx_error_ |= (BTR_DEV_EOVERFLOW >> 16);
  }
  LED_TOGGLE();
}

static void onSend(btr::Usart* u)
{
  uint8_t ch = u->tx_buff_[u->tx_tail_];
  u->tx_tail_ = (u->tx_tail_ + 1) % BTR_USART_TX_BUFF_SIZE;
  *(u->udr_) = ch;

  if (u->tx_head_ == u->tx_tail_) {
    clear_bit(*(u->ucsr_b_), UDRIE);
  }
  LED_TOGGLE();
}

// } Static members

////////////////////////////////////////////////////////////////////////////////////////////////////
// ISRs {
// See: http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html

#if defined (__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#if BTR_USART0_ENABLED > 0
ISR(USART_RX_vect)
{
  onRecv(&usart_0);
}
ISR(USART_UDRE_vect)
{
  onSend(&usart_0);
}
#endif

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#if BTR_USART0_ENABLED > 0
ISR(USART0_RX_vect)
{
  onRecv(&usart_0);
}
ISR(USART0_UDRE_vect)
{
  onSend(&usart_0);
}
#endif
#if BTR_USART1_ENABLED > 0
ISR(USART1_RX_vect)
{
  onRecv(&usart_1);
}
ISR(USART1_UDRE_vect)
{
  onSend(&usart_1);
}
#endif
#if BTR_USART2_ENABLED > 0
ISR(USART2_RX_vect)
{
  onRecv(&usart_2);
}
ISR(USART2_UDRE_vect)
{
  onSend(&usart_2);
}
#endif
#if BTR_USART3_ENABLED > 0
ISR(USART3_RX_vect)
{
  onRecv(&usart_3);
}
}
ISR(USART3_UDRE_vect)
{
  onSend(&usart_3);
}
#endif // BTR_USARTx

#endif // __AVR_ATmegaX

// } ISRs

namespace btr
{

/////////////////////////////////////////////// PUBLIC /////////////////////////////////////////////

//============================================= LIFECYCLE ==========================================

Usart::Usart(
    volatile uint8_t* ubrr_h,
    volatile uint8_t* ubrr_l,
    volatile uint8_t* ucsr_a,
    volatile uint8_t* ucsr_b,
    volatile uint8_t* ucsr_c,
    volatile uint8_t* udr)
  :
    ubrr_h_(ubrr_h),
    ubrr_l_(ubrr_l),
    ucsr_a_(ucsr_a),
    ucsr_b_(ucsr_b),
    ucsr_c_(ucsr_c),
    udr_(udr),
    rx_error_(0),
    enable_flush_(false),
    rx_head_(0),
    rx_tail_(0),
    rx_buff_(),
    tx_head_(0),
    tx_tail_(0),
    tx_buff_()
{
  // Taint data.
  rx_buff_[0] = 'U'; // hex 55
  tx_buff_[0] = 'f'; // hex 66
  clear_bit(*ucsr_b_, TXEN);
  clear_bit(*ucsr_b_, RXEN);
  clear_bit(*ucsr_b_, RXCIE);
  clear_bit(*ucsr_b_, UDRIE);
}

//============================================= OPERATIONS =========================================

// static
Usart* Usart::instance(uint32_t id, bool open)
{
  switch (id) {
#if BTR_USART0_ENABLED > 0
    case 0:
      if (open) {
        usart_0.open(BTR_USART0_BAUD, BTR_USART0_DATA_BITS, BTR_USART0_STOP_BITS, BTR_USART0_PARITY);
      }
      return &usart_0;
#endif
#if BTR_USART1_ENABLED > 0
    case 1:
      if (open) {
        usart_1.open(BTR_USART1_BAUD, BTR_USART1_DATA_BITS, BTR_USART1_STOP_BITS, BTR_USART1_PARITY);
      }
      return &usart_1;
#endif
#if BTR_USART2_ENABLED > 0
    case 2:
      if (open) {
        usart_2.open(BTR_USART2_BAUD, BTR_USART2_DATA_BITS, BTR_USART2_STOP_BITS, BTR_USART2_PARITY);
      }
      return &usart_2;
#endif
#if BTR_USART3_ENABLED > 0
    case 3:
      if (open) {
        usart_3.open(BTR_USART3_BAUD, BTR_USART3_DATA_BITS, BTR_USART3_STOP_BITS, BTR_USART3_PARITY);
      }
      return &usart_3;
#endif
    default:
      return nullptr;
  }
}

bool Usart::isOpen()
{
  return (bit_is_set(*ucsr_b_, TXEN) || bit_is_set(*ucsr_b_, RXEN));
}

int Usart::open(
    uint32_t baud, uint8_t data_bits, StopBitsType stop_bits, ParityType parity, const char* port)
{
  (void) port; // not used on AVR

  if (true == isOpen()) {
    return 0;
  }

  uint16_t baud_rate = BAUD_CALC(baud);
  *ubrr_h_ = baud_rate >> 8;
  *ubrr_l_ = baud_rate;
  *ucsr_c_ = BTR_USART_CONFIG(parity, stop_bits, data_bits);

#if BTR_USART_USE_2X > 0
  *ucsr_a_ = (1 << U2X);
#endif

  set_bit(*ucsr_b_, TXEN);
  set_bit(*ucsr_b_, RXEN);
  set_bit(*ucsr_b_, RXCIE);
  clear_bit(*ucsr_b_, UDRIE);

  return 0;
}

void Usart::close()
{
  flush(DirectionType::OUT);
  clear_bit(*ucsr_b_, TXEN);
  clear_bit(*ucsr_b_, RXEN);
  clear_bit(*ucsr_b_, RXCIE);
  clear_bit(*ucsr_b_, UDRIE);
  rx_head_ = rx_tail_;
}

int Usart::available()
{
  uint16_t bytes = BTR_USART_RX_BUFF_SIZE + rx_head_ - rx_tail_;
  return (bytes % BTR_USART_RX_BUFF_SIZE);
}

int Usart::flush(DirectionType queue_selector)
{
  if (false == enable_flush_) {
    return 0;
  }

  (void) queue_selector;

  while (bit_is_set(*ucsr_b_, UDRIE) || bit_is_clear(*ucsr_a_, TXC)) {
    if (bit_is_clear(SREG, SREG_I) && bit_is_set(*ucsr_b_, UDRIE)) {
      if (bit_is_set(*ucsr_a_, UDRE)) {
        // Call manually since global interrupts are disabled.
        onSend(this);
      }
    }
  }
  return 0;
}

uint32_t Usart::send(const char* buff, uint16_t bytes, uint32_t timeout)
{
  enable_flush_ = true;
  uint32_t rc = 0;
  uint32_t delay = 0;

  while (bytes > 0) {
    uint16_t head_next = (tx_head_ + 1) % BTR_USART_TX_BUFF_SIZE;

    // No room in tx buffer, wait until at least one character is drained from it.
    while (head_next == tx_tail_) {
      if (timeout > 0) {
        _delay_us(BTR_USART_TX_DELAY_US);
        delay += BTR_USART_TX_DELAY_US;

        if ((delay / 1000) >= timeout) {
          rc |= BTR_DEV_ETIMEOUT;
          return rc;
        }
      }
    }

    tx_buff_[tx_head_] = *buff++;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      tx_head_ = head_next;
      set_bit(*ucsr_b_, UDRIE);
    }
    ++rc;
    --bytes;
  }
  return rc;
}

uint32_t Usart::recv(char* buff, uint16_t bytes, uint32_t timeout)
{
  uint32_t rc = 0;
  uint32_t delay = 0;

  while (bytes > 0) {
    if (rx_head_ != rx_tail_) {
      *buff++ = rx_buff_[rx_tail_];  
      rx_tail_ = (rx_tail_ + 1) % BTR_USART_RX_BUFF_SIZE;
      delay = 0;
      --bytes;
      ++rc;
    } else {
      if (timeout > 0) {
        _delay_us(BTR_USART_RX_DELAY_US);
        delay += BTR_USART_RX_DELAY_US;

        if ((delay / 1000) >= timeout) {
          rc |= BTR_DEV_ETIMEOUT;
          break;
        }
      }
    }
  }
  rc |= (uint32_t(rx_error_) << 16);
  rx_error_ = 0;
  return rc;
}

} // namespace btr

#endif // BTR_USARTn_ENABLED > 0
