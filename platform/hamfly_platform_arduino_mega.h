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
 * Arduino Mega concrete implementation of hamfly_hal_t.
 * Targets Arduino Mega 2560 Serial1 (TX1/RX1, pins 18/19).
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * Configure Serial1 before calling hamfly_init():
 *   Serial1.begin(111111);
 *
 * The AVR hardware UART divider for 111111 baud at 16 MHz gives
 * a 0.01 % error, which is within tolerance.
 */

#ifndef HAMFLY_PLATFORM_ARDUINO_MEGA_H
#define HAMFLY_PLATFORM_ARDUINO_MEGA_H

#include "hamfly_core_hal.h"
#include <Arduino.h>

static void _hamfly_arduino_mega_putc(void *ctx, uint8_t b)
{
    ((HardwareSerial *)ctx)->write(b);
}

static uint32_t _hamfly_arduino_mega_tick_ms(void *ctx)
{
    (void)ctx;
    return (uint32_t)millis();
}

/* Pass &Serial1 (or &Serial2/3 if re-routed).
 * Caller must call Serial1.begin(111111) before hamfly_init(). */
static inline hamfly_hal_t hamfly_arduino_mega_hal(HardwareSerial *serial)
{
    hamfly_hal_t h;
    h.ctx         = (void *)serial;
    h.uart_putc   = _hamfly_arduino_mega_putc;
    h.get_tick_ms = _hamfly_arduino_mega_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_ARDUINO_MEGA_H */
