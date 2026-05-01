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
 * ESP32 ESP-IDF (native) concrete implementation of hamfly_hal_t.
 * For ESP32 using the Arduino framework use hamfly_platform_arduino.h.
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * The ESP32 APB clock (80 MHz) divides cleanly to 111111 baud.
 * Configure and install the UART driver before calling hamfly_init():
 *
 *   #define MOVI_UART   UART_NUM_1
 *   #define MOVI_TX_PIN 17
 *   #define MOVI_RX_PIN 16
 *   #define MOVI_RX_BUF 256
 *
 *   uart_config_t cfg = {
 *       .baud_rate  = 111111,
 *       .data_bits  = UART_DATA_8_BITS,
 *       .parity     = UART_PARITY_DISABLE,
 *       .stop_bits  = UART_STOP_BITS_1,
 *       .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
 *       .source_clk = UART_SCLK_APB,
 *   };
 *   uart_param_config(MOVI_UART, &cfg);
 *   uart_set_pin(MOVI_UART, MOVI_TX_PIN, MOVI_RX_PIN,
 *                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 *   uart_driver_install(MOVI_UART, MOVI_RX_BUF, 0, 0, NULL, 0);
 *
 *   uart_port_t port = MOVI_UART;
 *   hamfly_hal_t hal = hamfly_esp32_idf_hal(&port);
 *   hamfly_init(&g, &hal);
 *
 * tick_ms wraps at ~49.7 days (uint32 milliseconds from esp_timer).
 */

#ifndef HAMFLY_PLATFORM_ESP32_IDF_H
#define HAMFLY_PLATFORM_ESP32_IDF_H

#include "hamfly_core_hal.h"
#include <driver/uart.h>
#include <esp_timer.h>

static void _hamfly_esp32_idf_putc(void *ctx, uint8_t b)
{
    uart_write_bytes(*(uart_port_t *)ctx, (const char *)&b, 1);
}

static uint32_t _hamfly_esp32_idf_tick_ms(void *ctx)
{
    (void)ctx;
    /* esp_timer_get_time() returns microseconds as int64_t. */
    return (uint32_t)((uint64_t)esp_timer_get_time() / 1000u);
}

/* Pass a pointer to a uart_port_t variable (e.g. &my_port where
 * my_port = UART_NUM_1).  The variable must remain valid for the
 * lifetime of the hamfly_gimbal_t. */
static inline hamfly_hal_t hamfly_esp32_idf_hal(uart_port_t *port)
{
    hamfly_hal_t h;
    h.ctx         = (void *)port;
    h.uart_putc   = _hamfly_esp32_idf_putc;
    h.get_tick_ms = _hamfly_esp32_idf_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_ESP32_IDF_H */
