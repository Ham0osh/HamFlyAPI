/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson, Quantum Information Systems Lab, SFU Physics
 * Author: Hamish Johnson
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
 * Typical usage:
 * 
 * #include "hamfly.h"
 * 
 * static hamfly_gimbal_t g_gimbal;
 * static const hamfly_hal_t HAL = hamfly_psoc5_hal(NULL);
 * hamfly_init(&g_gimbal, &HAL);
 * 
 * // PSoC Specific initiate ISR for Rx for a UART UDB called "UART_MOVI".
 * hamfly_on_rx_byte(&g_gimbal, UART_MOVI_RXDATA_REG);
 * 
 * // Within the main loop you can pump into the gimbal struct
 * hamfly_pump(&g_gimbal);
 * // and also send a control packet that you built:
 * hamfly_send_control(&g_gimbal, &ctl);
 * 
 * // You can also optionally request other telemetry attributes:
 * hamfly_request_attr(&g_gimbal, 22u);
 * g_gimbal.pending_sent_ms = now_ms;
 * // Later when response received into the struct:
 * if (g_gimbal.pending_ready) { ... }
 * 
 */

#ifndef HAMFLY_H
#define HAMFLY_H

#include "hamfly_core_hal.h"
#include "hamfly_core_control.h"
#include "hamfly_core_telemetry.h"
#include "hamfly_core_gimbal.h"

#endif /* HAMFLY_H */
