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
 * ISR-safe RX ring buffer.
 * ISR pushes bytes in via hamfly_rb_push().
 * hamfly_pump() pops bytes out to QX_StreamRxCharSM.
 */

#ifndef HAMFLY_COMM_RB_H
#define HAMFLY_COMM_RB_H

#include <stdint.h>

#ifndef HAMFLY_RB_SIZE
#define HAMFLY_RB_SIZE 256u
#endif

// Boilerplate ring buffer.
typedef struct {
    volatile uint8_t  buf[HAMFLY_RB_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t drops;
} hamfly_rb_t;

void     hamfly_rb_init (hamfly_rb_t *rb);
void     hamfly_rb_clear(hamfly_rb_t *rb);
uint16_t hamfly_rb_count(const hamfly_rb_t *rb);
uint8_t  hamfly_rb_push (hamfly_rb_t *rb, uint8_t b);   // call from ISR
uint8_t  hamfly_rb_pop  (hamfly_rb_t *rb, uint8_t *b);  // call from main

#endif /* HAMFLY_COMM_RB_H */
