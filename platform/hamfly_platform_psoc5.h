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
 * PSoC 5 LP concrete implementation of hamfly_hal_t.
 * Only file in HamflyAPI that includes <project.h>.
 */

#ifndef HAMFLY_PLATFORM_PSOC5_H
#define HAMFLY_PLATFORM_PSOC5_H

#include "hamfly_core_hal.h"
#include "project.h"  // PSoC creator specific from its UDB registrations.

// PUTC functions assumes a UART UDB called "UART_MOVI" is present in the PSoC
// 5 design.
static void _hamfly_psoc5_putc(void *ctx, uint8_t b)
{
    (void)ctx;
    UART_MOVI_PutChar((char)b);
}

// Ready to use HAL for the PSoC 5 under these assumptions.
// User can adjust naming as needed.
static inline hamfly_hal_t hamfly_psoc5_hal(void *ctx)
{
    hamfly_hal_t h;
    h.ctx       = ctx;
    h.uart_putc = _hamfly_psoc5_putc;
    return h;
}

#endif /* HAMFLY_PLATFORM_PSOC5_H */
