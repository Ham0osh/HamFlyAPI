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
 * STM32 HAL concrete implementation of hamfly_hal_t.
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * Set BaudRate = 111111 in your CubeMX UART config or MX_USARTx_UART_Init().
 * STM32 fractional baud dividers achieve this exactly on 72/168/180 MHz clocks.
 *
 * Replace the #include below with the correct header for your series,
 * e.g. "stm32f4xx_hal.h", "stm32h7xx_hal.h", etc.
 */

#ifndef HAMFLY_PLATFORM_STM32_H
#define HAMFLY_PLATFORM_STM32_H

#include "hamfly_core_hal.h"
#include "stm32_hal.h"  /* adjust to your series */

/* Blocking single-byte transmit. Adequate for low-rate gimbal
 * commands; consider DMA or IT variants for high-throughput use. */
static void _hamfly_stm32_putc(void *ctx, uint8_t b)
{
    HAL_UART_Transmit((UART_HandleTypeDef *)ctx, &b, 1u,
                      HAL_MAX_DELAY);
}

static uint32_t _hamfly_stm32_tick_ms(void *ctx)
{
    (void)ctx;
    return (uint32_t)HAL_GetTick();
}

/* Pass pointer to the UART handle initialised by CubeMX, e.g. &huart1.
 * Caller must initialise the UART peripheral before hamfly_init(). */
static inline hamfly_hal_t hamfly_stm32_hal(UART_HandleTypeDef *huart)
{
    hamfly_hal_t h;
    h.ctx         = (void *)huart;
    h.uart_putc   = _hamfly_stm32_putc;
    h.get_tick_ms = _hamfly_stm32_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_STM32_H */
