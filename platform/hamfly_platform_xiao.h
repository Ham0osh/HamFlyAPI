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
 * Seeed XIAO SAMD21 (non-Sense) concrete implementation of hamfly_hal_t.
 * Uses Serial1 (hardware UART on D6/D7).
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * Configure Serial1 before calling hamfly_init():
 *   Serial1.begin(111111);
 *
 * Board notes:
 *   - Targets XIAO SAMD21 (ATSAMD21G18) with Arduino SAMD core.
 *   - Serial1 maps to SERCOM4 UART (D6=TX, D7=RX).
 *   - The SAMD21 fractional baud generator achieves 111111 baud
 *     with negligible error from the 48 MHz GCLK.
 *   - For XIAO RP2040: use hamfly_platform_pico.h with uart1.
 *   - For XIAO nRF52840 (Sense): use Arduino Serial1 but note
 *     the nRF52840 UART does not support fractional baud rates;
 *     111111 will fall back to 115200 unless you patch the core.
 */

#ifndef HAMFLY_PLATFORM_XIAO_H
#define HAMFLY_PLATFORM_XIAO_H

#include "hamfly_core_hal.h"
#include <Arduino.h>

static void _hamfly_xiao_putc(void *ctx, uint8_t b)
{
    ((Uart *)ctx)->write(b);
}

static uint32_t _hamfly_xiao_tick_ms(void *ctx)
{
    (void)ctx;
    return (uint32_t)millis();
}

/* Pass &Serial1 (default) or another Uart instance if re-routed.
 * Caller must call Serial1.begin(111111) before hamfly_init(). */
static inline hamfly_hal_t hamfly_xiao_hal(Uart *serial)
{
    hamfly_hal_t h;
    h.ctx         = (void *)serial;
    h.uart_putc   = _hamfly_xiao_putc;
    h.get_tick_ms = _hamfly_xiao_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_XIAO_H */
