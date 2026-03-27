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
 * Hardware abstraction layer (HAL) definition between the Hamfly API and the 
 * platforms UART.
 */

#ifndef HAMFLY_CORE_HAL_H
#define HAMFLY_CORE_HAL_H

#include <stdint.h>

// HAL struct for user to implement and pass to hamfly_init().
// HAL is used for platform agnostic communications of UART bytes.
// Implementation by user includes:
//  - uart_putc() to send a byte out the UART (NECESSARY).
//  - get_tick_ms() for timeout handling (OPTIONAL).
//
// ctx is a generic pointer passed to all HAL functions if needed for a
// specific platform (ie. specific UART instance or state).
typedef struct {
    void     *ctx;
    void     (*uart_putc)   (void *ctx, uint8_t b);
    uint32_t (*get_tick_ms) (void *ctx);  // Return 0 if unused.
} hamfly_hal_t;

#endif /* HAMFLY_CORE_HAL_H */
