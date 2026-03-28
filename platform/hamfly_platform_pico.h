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
 * Raspberry Pi Pico (RP2040) Pico SDK concrete implementation of hamfly_hal_t.
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * Configure the UART before calling hamfly_init(), e.g.:
 *   uart_init(uart1, 111111);
 *   gpio_set_function(4, GPIO_FUNC_UART);   // TX pin
 *   gpio_set_function(5, GPIO_FUNC_UART);   // RX pin
 *
 * tick_ms wraps at ~49.7 days (uint32 milliseconds from time_us_64).
 */

#ifndef HAMFLY_PLATFORM_PICO_H
#define HAMFLY_PLATFORM_PICO_H

#include "hamfly_core_hal.h"
#include <hardware/uart.h>
#include <pico/time.h>

static void _hamfly_pico_putc(void *ctx, uint8_t b)
{
    uart_putc_raw((uart_inst_t *)ctx, (char)b);
}

static uint32_t _hamfly_pico_tick_ms(void *ctx)
{
    (void)ctx;
    return (uint32_t)(time_us_64() / 1000u);
}

/* Pass uart0 or uart1; caller configures baud rate and GPIO pins. */
static inline hamfly_hal_t hamfly_pico_hal(uart_inst_t *uart)
{
    hamfly_hal_t h;
    h.ctx         = (void *)uart;
    h.uart_putc   = _hamfly_pico_putc;
    h.get_tick_ms = _hamfly_pico_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_PICO_H */
