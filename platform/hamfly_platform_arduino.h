/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson
 * Quantum Information Systems Lab, SFU Physics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 *
 * ============================================================================
 * Generic Arduino concrete implementation of hamfly_hal_t.
 * Works on any board that exposes a HardwareSerial UART, including:
 *   - Arduino Mega 2560  (&Serial1, TX1/RX1, pins 18/19)
 *   - Arduino Uno / Nano (&Serial, pins 1/0 — shares USB; not recommended)
 *   - Arduino Leonardo   (&Serial1, pins 1/0)
 *   - Arduino Due        (&Serial1, pins 18/19)
 *   - Arduino Nano Every (&Serial1, pins 0/1 via SERCOM)
 *   - Seeed XIAO SAMD21  (&Serial1, D6/D7 — Uart inherits HardwareSerial)
 *   - ESP32 Arduino      (&Serial1 or &Serial2 with custom pin mapping)
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * Configure the serial port before calling hamfly_init():
 *   Serial1.begin(111111);
 *
 * Baud rate notes by platform:
 *   AVR (Mega, Uno): 16 MHz UBRR gives <0.01 % error at 111111 baud.
 *   ESP32 Arduino:   fractional baud divider hits 111111 exactly.
 *   SAMD21 (XIAO):   SERCOM fractional generator hits 111111 exactly.
 */

#ifndef HAMFLY_PLATFORM_ARDUINO_H
#define HAMFLY_PLATFORM_ARDUINO_H

#include "hamfly_core_hal.h"
#include <Arduino.h>

static void _hamfly_arduino_putc(void *ctx, uint8_t b)
{
    ((HardwareSerial *)ctx)->write(b);
}

static uint32_t _hamfly_arduino_tick_ms(void *ctx)
{
    (void)ctx;
    return (uint32_t)millis();
}

/* Pass a pointer to the HardwareSerial instance you have wired to the
 * gimbal, e.g. &Serial1.  Caller must call Serial1.begin(111111) first. */
static inline hamfly_hal_t hamfly_arduino_hal(HardwareSerial *serial)
{
    hamfly_hal_t h;
    h.ctx         = (void *)serial;
    h.uart_putc   = _hamfly_arduino_putc;
    h.get_tick_ms = _hamfly_arduino_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_ARDUINO_H */
