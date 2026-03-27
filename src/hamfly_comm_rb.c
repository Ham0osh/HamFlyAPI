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
 * 
 * ISR-safe RX ring buffer.
 * ISR pushes via hamfly_rb_push().
 * Main loop pops via hamfly_rb_pop().
 *
 * This is a part of the Hamfly extension of the FreeflyAPI.
 * This is a simple ring buffer implementation.
 * 
 */

#include "hamfly_comm_rb.h"

// Get next index in a wrapping way.
static inline uint16_t rb_next(uint16_t i)
{
    i++;
    if (i >= HAMFLY_RB_SIZE) i = 0u;
    return i;
}

// Initialize the ring buffer.
void hamfly_rb_init(hamfly_rb_t *rb)
{
    rb->head  = 0u;
    rb->tail  = 0u;
    rb->drops = 0u;  // Count overflow drops.
}

// Reset ring buffer head and tail to empty.
void hamfly_rb_clear(hamfly_rb_t *rb)
{
    rb->head = 0u;
    rb->tail = 0u;
}

// Get number of bytes in the buffer.
uint16_t hamfly_rb_count(const hamfly_rb_t *rb)
{
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (h >= t) ? (h - t) : (uint16_t)(HAMFLY_RB_SIZE - (t - h));
}

// Push a byte into the ring buffer from b. Returns 1 on success, 0 if byte dropped.
uint8_t hamfly_rb_push(hamfly_rb_t *rb, uint8_t b)
{
    uint16_t next = rb_next(rb->head);
    if (next == rb->tail) {
        rb->drops++;
        return 0u;
    }
    rb->buf[rb->head] = b;
    rb->head = next;
    return 1u;
}

// Pop a byte from the ring buffer into *b. Returns 1 on success, 0 if buffer is empty.
uint8_t hamfly_rb_pop(hamfly_rb_t *rb, uint8_t *b)
{
    if (rb->head == rb->tail) return 0u;
    *b       = rb->buf[rb->tail];
    rb->tail = rb_next(rb->tail);
    return 1u;
}
