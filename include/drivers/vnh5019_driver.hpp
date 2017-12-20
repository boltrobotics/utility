/* Copyright (C) 2017 Bolt Robotics <info@boltrobotics.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifndef _drivers_VNH5019Driver_h_
#define _drivers_VNH5019Driver_h_

#include <Arduino.h>

namespace drivers {

/**
 * Driver for VNH5019 Arduino motor shield.
 *
 * Original code: https://github.com/pololu/dual-vnh5019-motor-shield
 * Product: https://www.pololu.com/product/2502
 */
class VNH5019Driver {
public:

// LIFECYCLE

    /**
     * Ctor. pwm1 and pwm2 cannot be remapped because the library assumes pwm
     * is on timer1.
     */
    VNH5019Driver(
            uint16_t max_pwm = 400,
            uint8_t ina1 = 2,
            uint8_t inb1 = 4,
            uint8_t en1diag1 = 6,
            uint8_t cs1 = A0,
            uint8_t ina2 = 7,
            uint8_t inb2 = 8,
            uint8_t en2diag2 = 5,
            uint8_t cs2 = A1);

// OPERATIONS

    /**
     * Initialize timer 1 (or timer 3 for mega), set PWM to 20kHz.
     */
    void init();

    /**
     * Set speed for M1.
     *
     * @param speed - the speed is a number betwenn -400 and 400.
     */
    void setM1Speed(int16_t speed);

    /**
     * Set speed for M2.
     *
     * @param speed - the speed is a number betwenn -400 and 400.
     */
    void setM2Speed(int16_t speed);

    /**
     * Set brake for M1.
     *
     * @param brake - the brake is a number between 0 (coast) and 400 (hard).
     */
    void setM1Brake(int16_t brake);

    /**
     * Set brake for M2.
     *
     * @param brake - the brake is a number between 0 (coast) and 400 (hard).
     */
    void setM2Brake(int16_t brake);

    /**
     * @return motor current value in milliamps.
     */
    uint8_t getM1Load();

    /**
     * @return motor current value in milliamps.
     */
    uint8_t getM2Load();

    /**
     * @return fault reading from M1.
     */
    uint8_t getM1Fault();

    /**
     * @return fault reading from M2.
     */
    uint8_t getM2Fault();

private:

// OPERATIONS

    void setSpeed(
            volatile uint16_t* ocr,
            uint8_t ina,
            uint8_t inb,
            uint8_t pwm_pin,
            int16_t speed);

    void setBrake(
            volatile uint16_t* ocr,
            uint8_t ina,
            uint8_t inb,
            uint8_t pwm_pin,
            int16_t speed);

// ATTRIBUTES

#if defined(__AVR_ATmega168__) || \
        defined(__AVR_ATmega328P__)
    static const uint8_t _pwm1 = 9;
    static const uint8_t _pwm2 = 10;
#elif defined(__AVR_ATmega128__) || \
        defined(__AVR_ATmega1280__) || \
        defined(__AVR_ATmega2560__)
    static const uint8_t _pwm1 = 11;
    static const uint8_t _pwm2 = 12;
#endif  

    uint16_t max_pwm_;
    uint8_t ina1_;
    uint8_t inb1_;
    uint8_t en1diag1_;
    uint8_t cs1_;
    uint8_t ina2_;
    uint8_t inb2_;
    uint8_t en2diag2_;
    uint8_t cs2_;

}; // class VNH5019Driver

////////////////////////////////////////////////////////////////////////////////
// INLINE OPERATIONS
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// PUBLIC ///////////////////////////////////

//=================================== LIFECYCLE ================================

inline VNH5019Driver::VNH5019Driver(
        uint16_t max_pwm,
        uint8_t ina1,
        uint8_t inb1,
        uint8_t en1diag1,
        uint8_t cs1, 
        uint8_t ina2,
        uint8_t inb2,
        uint8_t en2diag2,
        uint8_t cs2) :
    max_pwm_(max_pwm),
    ina1_(ina1),
    inb1_(inb1),
    en1diag1_(en1diag1),
    cs1_(cs1),
    ina2_(ina2),
    inb2_(inb2),
    en2diag2_(en2diag2),
    cs2_(cs2) {
}

//=================================== OPERATIONS ===============================

inline void VNH5019Driver::init() {
    // Define pinMode for the pins and set the frequency for timer1.
    pinMode(ina1_, OUTPUT);
    pinMode(inb1_, OUTPUT);
    pinMode(_pwm1, OUTPUT);
    pinMode(en1diag1_, INPUT);
    pinMode(cs1_, INPUT);
    pinMode(ina2_, OUTPUT);
    pinMode(inb2_, OUTPUT);
    pinMode(_pwm2, OUTPUT);
    pinMode(en2diag2_, INPUT);
    pinMode(cs2_, INPUT);

    // See ATmega2560 17.11 Register Description.
    //
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    // Timer 1 configuration
    // prescaler: clockI/O / 1
    // outputs enabled
    // phase-correct pwm
    // top of 400
    //
    // pwm frequency calculation
    // 16MHz / 1 (no prescaling) / 2 (phase-correct) / 400 (top) = 20kHz
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    // ICR provides fixed TOP value so that OCR can be used for PWM.
    ICR1 = 400;
#elif defined(__AVR_ATmega128__) || \
        defined(__AVR_ATmega1280__) || \
        defined(__AVR_ATmega2560__)
    // Mega board specific stuff here - assumes assigning timer3, using
    // pins 3 & 5.
    TCCR1A = 0b10100000; // Mega pin 5
    TCCR1B = 0b00010001; // Mega pin 3
    ICR1 = 400;
#endif
}

inline void VNH5019Driver::setM1Speed(int16_t speed) {
    setSpeed(&OCR1A, ina1_, inb1_, _pwm1, speed);
}

inline void VNH5019Driver::setM2Speed(int16_t speed) {
    setSpeed(&OCR1B, ina2_, inb2_, _pwm2, speed);
}

inline void VNH5019Driver::setM1Brake(int brake) {
    setBrake(&OCR1A, ina1_, inb1_, _pwm1, brake);
}

inline void VNH5019Driver::setM2Brake(int brake) {
    setBrake(&OCR1B, ina2_, inb2_, _pwm2, brake);
}

inline uint8_t VNH5019Driver::getM1Load() {
    // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
    return analogRead(cs1_) * 34;
}

inline uint8_t VNH5019Driver::getM2Load() {
    // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
    return analogRead(cs2_) * 34;
}

inline uint8_t VNH5019Driver::getM1Fault() {
    return !digitalRead(en1diag1_);
}

inline uint8_t VNH5019Driver::getM2Fault() {
    return !digitalRead(en2diag2_);
}

///////////////////////////////////// PRIVATE //////////////////////////////////

//=================================== OPERATIONS ===============================

inline void VNH5019Driver::setSpeed(
        volatile uint16_t* ocr,
        uint8_t ina,
        uint8_t inb,
        uint8_t pwm_pin,
        int16_t speed) {

    uint8_t reverse = 0;

    if (speed < 0) {
        // Make speed a positive quantity.
        speed = -speed;
        // Preserve the direction.
        reverse = 1;
    }

    if (speed > max_pwm_) {
        speed = max_pwm_;
    }

#if defined(__AVR_ATmega168__) || \
        defined(__AVR_ATmega328P__) || \
        defined(__AVR_ATmega128__) || \
        defined(__AVR_ATmega1280__) || \
        defined(__AVR_ATmega2560__)
    *ocr = speed;
#else
    // Default to using analogWrite, mapping 400 to 255.
    analogWrite(pwm_pin, speed * 51 / 80);
#endif

    if (reverse) {
        digitalWrite(ina, LOW);
        digitalWrite(inb, HIGH);
    } else {
        digitalWrite(ina, HIGH);
        digitalWrite(inb, LOW);
    }
}

inline void VNH5019Driver::setBrake(
        volatile uint16_t* ocr,
        uint8_t ina,
        uint8_t inb,
        uint8_t pwm_pin,
        int16_t brake) {

    // Normalize brake.
    //
    if (brake < 0) {
        brake = -brake;
    }

    if (brake > max_pwm_) {
        brake = max_pwm_;
    }

    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);

#if defined(__AVR_ATmega168__) || \
        defined(__AVR_ATmega328P__) || \
        defined(__AVR_ATmega128__) || \
        defined(__AVR_ATmega1280__) || \
        defined(__AVR_ATmega2560__)
    *ocr = brake;
#else
    // Default to using analogWrite, mapping 400 to 255
    analogWrite(pwm_pin, brake * 51 / 80);
#endif
}

} // namespace drivers

#endif // _drivers_VNH5019Driver_h_
