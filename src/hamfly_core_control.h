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
 * Gimbal control types and send function declarations.
 * This is a part of the Hamfly extension of the FreeflyAPI.
 * Control type enum matches FreeflyAPI ff_api_control_type_e in qx_app.h.
 */

#ifndef HAMFLY_CORE_CONTROL_H
#define HAMFLY_CORE_CONTROL_H

#include <stdint.h>

// Must match ff_api_control_type_e in qx_app.h for posterity.
typedef enum {
    HAMFLY_DEFER    = 0,
    HAMFLY_RATE     = 1,
    HAMFLY_ABSOLUTE = 2
} hamfly_control_mode_t;

// Control packet struct. Gets built by the user and sent to the gimbal on
// hamfly_send_control() in hamfly_core_control.c.
typedef struct {
    float                 pan;
    float                 tilt;
    float                 roll;
    hamfly_control_mode_t pan_mode;
    hamfly_control_mode_t tilt_mode;
    hamfly_control_mode_t roll_mode;
    uint8_t               enable;
    uint8_t               kill;
} hamfly_control_t;

#endif /* HAMFLY_CORE_CONTROL_H */
