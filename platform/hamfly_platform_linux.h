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
 * Linux / POSIX concrete implementation of hamfly_hal_t.
 * Useful for host-side testing or embedded Linux targets (RPi, BeagleBone).
 *
 * UART configuration for Freefly MōVI gimbal:
 *   Baud rate : 111111  (non-standard — NOT 115200)
 *   Frame     : 8N1
 *
 * 111111 baud is not in the POSIX standard baud set. On Linux you
 * can set it with the BOTHER / TCSETS2 ioctl (requires kernel >= 2.6):
 *
 *   struct termios2 tio;
 *   ioctl(fd, TCGETS2, &tio);
 *   tio.c_cflag &= ~CBAUD;
 *   tio.c_cflag |=  BOTHER;
 *   tio.c_ispeed = 111111;
 *   tio.c_ospeed = 111111;
 *   ioctl(fd, TCSETS2, &tio);
 *
 * tick_ms wraps at ~49.7 days (uint32 milliseconds from CLOCK_MONOTONIC).
 */

#ifndef HAMFLY_PLATFORM_LINUX_H
#define HAMFLY_PLATFORM_LINUX_H

#include "hamfly_core_hal.h"
#include <unistd.h>
#include <time.h>
#include <stdint.h>

static void _hamfly_linux_putc(void *ctx, uint8_t b)
{
    int fd = *(int *)ctx;
    (void)write(fd, &b, 1u);
}

static uint32_t _hamfly_linux_tick_ms(void *ctx)
{
    struct timespec ts;
    (void)ctx;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u
                    + (uint64_t)ts.tv_nsec / 1000000u);
}

/* Pass a pointer to an open, configured file descriptor.
 * The fd must remain valid for the lifetime of the hamfly_gimbal_t. */
static inline hamfly_hal_t hamfly_linux_hal(int *fd)
{
    hamfly_hal_t h;
    h.ctx         = (void *)fd;
    h.uart_putc   = _hamfly_linux_putc;
    h.get_tick_ms = _hamfly_linux_tick_ms;
    return h;
}

#endif /* HAMFLY_PLATFORM_LINUX_H */
