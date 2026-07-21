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
 * Offline golden-vector tests for the v2 GCU settings RMW work, sourced from
 * build/re-notes/hamfly_test_vectors.md. No hardware, no HAL beyond an
 * in-memory mock -- every test here runs on a desktop compiler.
 *
 * HOW TO RUN OFFLINE
 * --------------------------------------------------------------------------
 * This file defines hamfly_settings_tests_run() (an ordinary function, safe
 * to link into the embedded firmware image -- it contains no `main` and is
 * dead code unless something calls it) plus a `main()` that is compiled in
 * ONLY when HAMFLY_SETTINGS_TEST_STANDALONE is defined, so it can never
 * collide with the project's real firmware entry point. On a desktop with
 * any C99 compiler (this environment has none -- not build-verified):
 *
 *   cc -std=c99 -DHAMFLY_SETTINGS_TEST_STANDALONE -Isrc \
 *      src/hamfly_settings_tests.c src/hamfly_settings_rmw.c \
 *      src/hamfly_settings_wire.c  src/hamfly_settings_field.c \
 *      src/hamfly_attr_table.c     src/hamfly_core_gimbal.c \
 *      src/hamfly_core_telemetry.c src/hamfly_qx_protocol.c \
 *      src/hamfly_qx_app.c         src/hamfly_comm_rb.c \
 *      -lm -o hamfly_settings_tests && ./hamfly_settings_tests
 *
 * Exit code is the number of failed checks (0 = all pass).
 *
 * NOT build-verified -- no C compiler was available when this was written.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "hamfly.h"
#include "hamfly_attr_table.h"
#include "hamfly_settings_field.h"
#include "hamfly_settings_wire.h"
#include "hamfly_settings_rmw.h"

#ifdef HAMFLY_SETTINGS_TEST_STANDALONE
#include <stdio.h>
#endif

/* ----------------------------------------------------------------------
 * Minimal check harness -- no external framework assumed.
 * ---------------------------------------------------------------------- */
static int s_pass = 0;
static int s_fail = 0;

static void hft_check(int cond, const char *msg)
{
    if (cond) {
        s_pass++;
    } else {
        s_fail++;
#ifdef HAMFLY_SETTINGS_TEST_STANDALONE
        printf("FAIL: %s\n", msg);
#else
        (void)msg;
#endif
    }
}
#define HFT_CHECK(cond, msg) hft_check((cond) ? 1 : 0, msg)

/* ----------------------------------------------------------------------
 * Mock HAL -- captures every byte handed to uart_putc(), and can act as a
 * no-clock or clock-having HAL to exercise both timeout paths in
 * hamfly_settings_rmw.c.
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t  buf[1024];
    uint16_t len;
    uint32_t tick_ms;
} mock_uart_t;

static void mock_putc(void *ctx, uint8_t b)
{
    mock_uart_t *m = (mock_uart_t *)ctx;
    if (m->len < sizeof(m->buf)) m->buf[m->len++] = b;
}

static uint32_t mock_get_tick_ms(void *ctx)
{
    return ((mock_uart_t *)ctx)->tick_ms;
}

static hamfly_hal_t mock_hal_no_clock(mock_uart_t *m)
{
    memset(m, 0, sizeof(*m));
    hamfly_hal_t h;
    h.ctx         = m;
    h.uart_putc   = mock_putc;
    h.get_tick_ms = NULL; /* the state actually shipped today -- see hamfly_psoc5_hal() */
    return h;
}

/* Feed one complete frame's bytes into the gimbal's ISR-facing entry point,
 * one at a time, exactly as the real UART RX ISR would. */
static void feed_bytes(hamfly_gimbal_t *g, const uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) hamfly_on_rx_byte(g, b[i]);
}

/* ----------------------------------------------------------------------
 * T1 -- checksum worked example (hamfly_test_vectors.md sec 0)
 * ---------------------------------------------------------------------- */
static void test_checksum_worked_example(void)
{
    const uint8_t data1[] = {0xD3,0x01,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x01};
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(data1); ++i) sum += data1[i];
    HFT_CHECK((uint8_t)(0xFFu - (sum & 0xFFu)) == 0x80u, "T1: motor kill=1 checksum should be 0x80");

    const uint8_t data0[] = {0xD3,0x01,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x00};
    sum = 0;
    for (size_t i = 0; i < sizeof(data0); ++i) sum += data0[i];
    HFT_CHECK((uint8_t)(0xFFu - (sum & 0xFFu)) == 0x81u, "T1: motor kill=0 checksum should be 0x81");
}

/* ----------------------------------------------------------------------
 * T2 -- QB read request builder (hamfly_test_vectors.md sec 1)
 * ---------------------------------------------------------------------- */
static void test_qb_build_read(void)
{
    uint8_t buf[8];

    uint8_t n = hamfly_qb_build_read(83u, buf, (uint8_t)sizeof buf);
    const uint8_t expect83[] = {0x51,0x42,0x00,0x01,0x53,0xAC};
    HFT_CHECK(n == 6u && memcmp(buf, expect83, 6u) == 0, "T2: QB read attr 83 frame");

    n = hamfly_qb_build_read(104u, buf, (uint8_t)sizeof buf);
    const uint8_t expect104[] = {0x51,0x42,0x00,0x01,0x68,0x97};
    HFT_CHECK(n == 6u && memcmp(buf, expect104, 6u) == 0, "T2: QB read attr 104 frame");

    n = hamfly_qb_build_read(110u, buf, (uint8_t)sizeof buf);
    const uint8_t expect110[] = {0x51,0x42,0x00,0x01,0x6E,0x91};
    HFT_CHECK(n == 6u && memcmp(buf, expect110, 6u) == 0, "T2: QB read attr 110 frame");

    /* A QX attribute must be refused by the QB builder. */
    HFT_CHECK(hamfly_qb_build_read(382u, buf, (uint8_t)sizeof buf) == 0u,
              "T2: QB builder must refuse attr > 255");
}

/* ----------------------------------------------------------------------
 * T3 -- attr 83 field decode, including -44 sign extension
 * (hamfly_test_vectors.md sec 2). Note: "Yaw Rate Control Limit" (offset 3)
 * appears in the test-vector decode walkthrough but has NO row in
 * hamfly_attr_table.h -- an evidence completeness gap, flagged in the
 * delivery report. It is still safely carried through byte-for-byte by any
 * RMW on this struct's OTHER fields; it just cannot be decoded by name.
 * ---------------------------------------------------------------------- */
static void test_decode_attr83(void)
{
    const uint8_t payload[] = {0x00,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x00};
    float v;

    HFT_CHECK(hamfly_settings_get(83u, "Max Control Rate", payload, sizeof payload, &v) && v == 25.0f,
              "T3: Max Control Rate == 25");
    HFT_CHECK(hamfly_settings_get(83u, "Max Tilt Angle", payload, sizeof payload, &v) && v == 45.0f,
              "T3: Max Tilt Angle == 45");
    HFT_CHECK(hamfly_settings_get(83u, "Min Tilt Angle", payload, sizeof payload, &v) && v == -44.0f,
              "T3: Min Tilt Angle == -44 (0xFFD4 sign extension)");
    HFT_CHECK(hamfly_settings_get(83u, "Max Roll Angle", payload, sizeof payload, &v) && v == 45.0f,
              "T3: Max Roll Angle == 45");
    HFT_CHECK(hamfly_settings_get(83u, "Motor Kill", payload, sizeof payload, &v) && v == 0.0f,
              "T3: Motor Kill == 0 (motors live)");

    /* A too-short payload must be refused, never silently mis-decoded. */
    HFT_CHECK(hamfly_settings_get(83u, "Motor Kill", payload, 11u, &v) == false,
              "T3: Rule 3 -- must refuse when payload doesn't reach the field");
}

/* ----------------------------------------------------------------------
 * T4 -- QB write builder reproduces the RMW golden frame byte-for-byte
 * given a correctly-preserved payload (hamfly_test_vectors.md sec 3).
 * ---------------------------------------------------------------------- */
static void test_qb_build_write_rmw(void)
{
    uint8_t payload_kill1[] = {0x01,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x01};
    uint8_t frame[24];
    uint8_t n = hamfly_qb_build_write(83u, payload_kill1, (uint8_t)sizeof payload_kill1,
                                       frame, (uint8_t)sizeof frame);
    const uint8_t expect_kill1[] = {0x51,0x42,0x00,0x0D,0xD3,0x01,0x00,0x19,0x00,0x64,
                                     0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x01,0x80};
    HFT_CHECK(n == sizeof expect_kill1 && memcmp(frame, expect_kill1, sizeof expect_kill1) == 0,
              "T4: RMW write frame, Motor Kill=1, siblings preserved");

    uint8_t payload_kill0[] = {0x01,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x00};
    n = hamfly_qb_build_write(83u, payload_kill0, (uint8_t)sizeof payload_kill0,
                               frame, (uint8_t)sizeof frame);
    const uint8_t expect_kill0[] = {0x51,0x42,0x00,0x0D,0xD3,0x01,0x00,0x19,0x00,0x64,
                                     0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x00,0x81};
    HFT_CHECK(n == sizeof expect_kill0 && memcmp(frame, expect_kill0, sizeof expect_kill0) == 0,
              "T4: RMW write frame, Motor Kill=0, siblings preserved");
}

/* ----------------------------------------------------------------------
 * T5 -- negative/contrast test: a NAIVE (zero-based) payload for the SAME
 * struct reproduces the documented "what a naive write looks like" golden
 * frame exactly. This demonstrates the QB builder is a faithful, dumb
 * serialiser -- the difference between "naive" and "RMW" is entirely in
 * which payload bytes are supplied to it, which is exactly why Rule 2
 * (never write a struct you could not read first) is the load-bearing
 * rule, not the frame encoding itself.
 * ---------------------------------------------------------------------- */
static void test_qb_build_write_naive_contrast(void)
{
    uint8_t naive_payload[12] = {0};
    naive_payload[0] = 0x01;         /* type byte */
    naive_payload[1] = 0x00;         /* Max Control Rate hi */
    naive_payload[2] = 0xFA;         /* Max Control Rate lo -- 250 */
    /* every other byte left at 0 -- this is what "forgot to read first" looks like */

    uint8_t frame[24];
    const uint8_t n = hamfly_qb_build_write(83u, naive_payload, (uint8_t)sizeof naive_payload,
                                             frame, (uint8_t)sizeof frame);
    const uint8_t expect_naive[] = {0x51,0x42,0x00,0x0D,0xD3,0x01,0x00,0xFA,0x00,0x00,
                                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x31};
    HFT_CHECK(n == sizeof expect_naive && memcmp(frame, expect_naive, sizeof expect_naive) == 0,
              "T5: naive zero-based frame matches the documented negative example");

    /* And, for contrast, the byte-correct RMW frame for the SAME struct
     * (Motor Kill=1, from T4) is NOT this frame -- the sibling fields
     * differ, which is precisely the failure Rule 2 exists to prevent. */
    uint8_t rmw_payload[] = {0x01,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x01};
    uint8_t rmw_frame[24];
    const uint8_t rn = hamfly_qb_build_write(83u, rmw_payload, (uint8_t)sizeof rmw_payload,
                                              rmw_frame, (uint8_t)sizeof rmw_frame);
    HFT_CHECK(!(n == rn && memcmp(frame, rmw_frame, n) == 0),
              "T5: naive frame must differ from the sibling-preserving RMW frame");
}

/* ----------------------------------------------------------------------
 * T6 -- QX write builder, three golden/derived vectors
 * (hamfly_test_vectors.md sec 4).
 * ---------------------------------------------------------------------- */
static void test_qx_build_write(void)
{
    uint8_t frame[24];
    const uint8_t gimbal_id = 2u; /* QX_DEV_ID_GIMBAL */

    uint8_t p350[] = {0x00,0x00,0x00,0x04};
    uint8_t n = hamfly_qx_build_write(350u, gimbal_id, p350, (uint8_t)sizeof p350,
                                       frame, (uint8_t)sizeof frame);
    const uint8_t expect350[] = {0x51,0x58,0x0B,0xDE,0x02,0x02,0x0A,0x02,0x00,0x00,
                                  0x00,0x00,0x00,0x04,0x0D};
    HFT_CHECK(n == sizeof expect350 && memcmp(frame, expect350, sizeof expect350) == 0,
              "T6: QX write, attr 350 (golden, encoder-verified)");

    uint8_t p292[] = {0x01};
    n = hamfly_qx_build_write(292u, gimbal_id, p292, (uint8_t)sizeof p292,
                               frame, (uint8_t)sizeof frame);
    const uint8_t expect292[] = {0x51,0x58,0x08,0xA4,0x02,0x02,0x0A,0x02,0x00,0x00,0x01,0x4A};
    HFT_CHECK(n == sizeof expect292 && memcmp(frame, expect292, sizeof expect292) == 0,
              "T6: QX write, attr 292 (golden, encoder-verified)");

    uint8_t p382[] = {0x01};
    n = hamfly_qx_build_write(382u, gimbal_id, p382, (uint8_t)sizeof p382,
                               frame, (uint8_t)sizeof frame);
    const uint8_t expect382[] = {0x51,0x58,0x08,0xFE,0x02,0x02,0x0A,0x02,0x00,0x00,0x01,0xF0};
    HFT_CHECK(n == sizeof expect382 && memcmp(frame, expect382, sizeof expect382) == 0,
              "T6: QX write, attr 382 (derived -- confirm on first hardware contact)");
}

/* ----------------------------------------------------------------------
 * T7 -- attr 110: absent from the table, must never yield a field value,
 * and hamfly_settings_write_start() must refuse it before any I/O.
 * ---------------------------------------------------------------------- */
static void test_attr110_no_usable_data(void)
{
    HFT_CHECK(hamfly_settings_find_field(110u, "Aux Port Function") == NULL,
              "T7: attr 110 must have no rows in the table");
    HFT_CHECK(hamfly_settings_field_count(110u) == 0u,
              "T7: attr 110 field count must be 0");

    mock_uart_t mock;
    hamfly_hal_t hal = mock_hal_no_clock(&mock);
    hamfly_gimbal_t g;
    hamfly_init(&g, &hal);

    hamfly_settings_rmw_t txn;
    hamfly_settings_rmw_state_t st = hamfly_settings_write_start(
        &txn, &g, 110u, "Aux Port Function", 1.0f, false);
    HFT_CHECK(st == HAMFLY_SETTINGS_RMW_ERR_UNKNOWN_FIELD,
              "T7: write_start on attr 110 must refuse as unknown field");
    HFT_CHECK(mock.len == 0u, "T7: refusing attr 110 must send zero bytes");
}

/* ----------------------------------------------------------------------
 * T8 -- attr 121 decode cross-check (hamfly_test_vectors.md sec 5): a
 * real, multi-field, non-writable attribute's golden vector, to sanity
 * check the generic decoder against something outside the writable set.
 * ---------------------------------------------------------------------- */
static void test_decode_attr121(void)
{
    const uint8_t payload[] = {
        0xF5,0xBA,0x90,0x24, 0x00,0x02,0x00,0x06, 0x07,0xE7,0x02,0x15,
        0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,
        0x02,0x03,0x05
    };
    float v;
    HFT_CHECK(hamfly_settings_get(121u, "Device Serial Number", payload, sizeof payload, &v)
              && v == 4122644516.0f,
              "T8: attr 121 serial number == 4122644516");
    HFT_CHECK(hamfly_settings_get(121u, "Build Year", payload, sizeof payload, &v) && v == 2023.0f,
              "T8: attr 121 build year == 2023");
    HFT_CHECK(hamfly_settings_get(121u, "Build Month", payload, sizeof payload, &v) && v == 2.0f,
              "T8: attr 121 build month == 2");
    HFT_CHECK(hamfly_settings_get(121u, "Build Day", payload, sizeof payload, &v) && v == 21.0f,
              "T8: attr 121 build day == 21");
    HFT_CHECK(hamfly_settings_get(121u, "Firmware Major", payload, sizeof payload, &v) && v == 2.0f,
              "T8: attr 121 firmware major == 2");
    HFT_CHECK(hamfly_settings_get(121u, "Firmware Minor", payload, sizeof payload, &v) && v == 3.0f,
              "T8: attr 121 firmware minor == 3");
    HFT_CHECK(hamfly_settings_get(121u, "Firmware Patch", payload, sizeof payload, &v) && v == 5.0f,
              "T8: attr 121 firmware patch == 5");
}

/* ----------------------------------------------------------------------
 * T9 -- typed set(): range clamp and NaN refusal (task step 4).
 * ---------------------------------------------------------------------- */
static void test_encode_clamp_and_nan(void)
{
    int32_t raw;
    float   clamped_p;
    bool    was_clamped;

    /* Max Control Rate: vmin=25, vmax=500. Request 999 -> clamp to 500. */
    HFT_CHECK(hamfly_settings_set(83u, "Max Control Rate", 999.0f, &raw, &clamped_p, &was_clamped)
              && raw == 500 && clamped_p == 500.0f && was_clamped == true,
              "T9: out-of-range request clamps to vmax, never wraps");

    /* Request below vmin -> clamp to vmin. */
    HFT_CHECK(hamfly_settings_set(83u, "Max Control Rate", 1.0f, &raw, &clamped_p, &was_clamped)
              && raw == 25 && clamped_p == 25.0f && was_clamped == true,
              "T9: below-range request clamps to vmin");

    /* In-range request is untouched. */
    HFT_CHECK(hamfly_settings_set(83u, "Max Control Rate", 100.0f, &raw, &clamped_p, &was_clamped)
              && raw == 100 && was_clamped == false,
              "T9: in-range request is not clamped");

    /* NaN must never be encoded. */
    float nan_val;
    memset(&nan_val, 0xFF, sizeof nan_val); /* a NaN bit pattern on IEEE-754 float */
    HFT_CHECK(hamfly_settings_set(83u, "Max Control Rate", nan_val, &raw, &clamped_p, &was_clamped) == false,
              "T9: NaN must be refused, never encoded");

    /* writable=0 field must be refused by set(), even though it decodes fine. */
    HFT_CHECK(hamfly_settings_set(1u, "Battery Voltage", 12.0f, &raw, &clamped_p, &was_clamped) == false,
              "T9: set() must refuse a non-writable field");
}

/* ----------------------------------------------------------------------
 * T10 -- full offline round trip through the REAL wire pipeline: start(),
 * observe the exact QB read request on the wire, feed a synthetic golden
 * reply byte-by-byte through hamfly_on_rx_byte()+hamfly_pump() (exactly as
 * the ISR + main loop would), poll() to completion, and check the exact
 * QB write frame that comes out the other side. This is the central
 * design problem's solution exercised end-to-end, offline.
 * ---------------------------------------------------------------------- */
static void test_rmw_round_trip_motor_kill(void)
{
    mock_uart_t mock;
    hamfly_hal_t hal = mock_hal_no_clock(&mock);
    hamfly_gimbal_t g;
    hamfly_init(&g, &hal);

    hamfly_settings_rmw_t txn;
    hamfly_settings_rmw_state_t st =
        hamfly_settings_write_start(&txn, &g, 83u, "Motor Kill", 1.0f, false);
    HFT_CHECK(st == HAMFLY_SETTINGS_RMW_READING, "T10: start() begins reading");

    const uint8_t expect_read[] = {0x51,0x42,0x00,0x01,0x53,0xAC};
    HFT_CHECK(mock.len == sizeof expect_read && memcmp(mock.buf, expect_read, sizeof expect_read) == 0,
              "T10: start() sent exactly the golden QB read request");

    /* Synthetic reply: DATA = 53 00 00 19 00 64 00 2D FF D4 00 2D 00
     * (attr echo + the same struct as hamfly_test_vectors.md sec 2, Motor
     * Kill currently 0). Checksum hand-computed: sum(DATA)=0x2FD,
     * &0xFF=0xFD, 0xFF-0xFD=0x02. */
    const uint8_t reply[] = {0x51,0x42,0x00,0x0D,
                              0x53,0x00,0x00,0x19,0x00,0x64,0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x00,
                              0x02};
    feed_bytes(&g, reply, (uint16_t)sizeof reply);
    hamfly_pump(&g);

    HFT_CHECK(g.pending_ready == true && g.pending_response_attr == 83u,
              "T10: synthetic reply captured by the existing hamfly_pump() logic");

    st = hamfly_settings_write_poll(&txn);
    HFT_CHECK(st == HAMFLY_SETTINGS_RMW_OK, "T10: poll() completes the write and reaches OK");
    HFT_CHECK(txn.prev_physical == 0.0f, "T10: prev_physical captured as 0 (motors were live)");

    const uint8_t expect_write[] = {0x51,0x42,0x00,0x0D,0xD3,0x01,0x00,0x19,0x00,0x64,
                                     0x00,0x2D,0xFF,0xD4,0x00,0x2D,0x01,0x80};
    const uint16_t write_off = (uint16_t)sizeof expect_read;
    HFT_CHECK(mock.len == write_off + sizeof expect_write, "T10: write frame length on the wire");
    HFT_CHECK(memcmp(&mock.buf[write_off], expect_write, sizeof expect_write) == 0,
              "T10: the write frame on the wire is byte-for-byte the golden Motor Kill=1 frame");
}

/* ----------------------------------------------------------------------
 * T11 -- fail-closed timeout: no reply is ever fed. Must retry exactly
 * HAMFLY_SETTINGS_RMW_READ_TRIES times, land on ERR_NO_READ, and NEVER put
 * a write-shaped frame on the wire.
 * ---------------------------------------------------------------------- */
static void test_rmw_timeout_fails_closed(void)
{
    mock_uart_t mock;
    hamfly_hal_t hal = mock_hal_no_clock(&mock);
    hamfly_gimbal_t g;
    hamfly_init(&g, &hal);

    hamfly_settings_rmw_t txn;
    hamfly_settings_rmw_state_t st =
        hamfly_settings_write_start(&txn, &g, 83u, "Motor Kill", 1.0f, false);
    HFT_CHECK(st == HAMFLY_SETTINGS_RMW_READING, "T11: start() begins reading");

    /* No hamfly_on_rx_byte()/hamfly_pump() calls at all -- pending_ready
     * can never become true. Bound the loop generously above the exact
     * expected iteration count (READ_TRIES * POLL_BUDGET) so the test
     * fails loudly instead of spinning if the budget arithmetic changes. */
    uint32_t iterations = 0u;
    while (!hamfly_settings_write_done(&txn) &&
           iterations < (uint32_t)HAMFLY_SETTINGS_RMW_READ_TRIES * HAMFLY_SETTINGS_RMW_POLL_BUDGET + 10u) {
        st = hamfly_settings_write_poll(&txn);
        iterations++;
    }

    HFT_CHECK(st == HAMFLY_SETTINGS_RMW_ERR_NO_READ,
              "T11: exhausting all retries with no reply fails closed to ERR_NO_READ");

    const uint16_t expect_len = 6u * (uint16_t)HAMFLY_SETTINGS_RMW_READ_TRIES;
    HFT_CHECK(mock.len == expect_len,
              "T11: exactly READ_TRIES 6-byte read frames were sent, nothing else");
    for (uint16_t i = 0; i < mock.len; i += 6u) {
        const uint8_t expect_read[] = {0x51,0x42,0x00,0x01,0x53,0xAC};
        HFT_CHECK(memcmp(&mock.buf[i], expect_read, 6u) == 0,
                  "T11: every frame on the wire is a read retry, never a write");
    }
}

/* ----------------------------------------------------------------------
 * T12 -- single-outstanding-request guard: a second start() while one is
 * already in flight must refuse immediately, with no bytes sent.
 * ---------------------------------------------------------------------- */
static void test_rmw_busy_guard(void)
{
    mock_uart_t mock;
    hamfly_hal_t hal = mock_hal_no_clock(&mock);
    hamfly_gimbal_t g;
    hamfly_init(&g, &hal);

    hamfly_settings_rmw_t txn1;
    hamfly_settings_rmw_state_t st1 =
        hamfly_settings_write_start(&txn1, &g, 83u, "Motor Kill", 1.0f, false);
    HFT_CHECK(st1 == HAMFLY_SETTINGS_RMW_READING, "T12: first transaction begins reading");

    const uint16_t len_after_first = mock.len;

    hamfly_settings_rmw_t txn2;
    hamfly_settings_rmw_state_t st2 =
        hamfly_settings_write_start(&txn2, &g, 83u, "Max Control Rate", 100.0f, false);
    HFT_CHECK(st2 == HAMFLY_SETTINGS_RMW_ERR_BUSY,
              "T12: a second transaction while one is in flight must refuse as busy");
    HFT_CHECK(mock.len == len_after_first,
              "T12: refusing the second transaction must send zero additional bytes");
}

/* ----------------------------------------------------------------------
 * Runner
 * ---------------------------------------------------------------------- */
int hamfly_settings_tests_run(void)
{
    s_pass = 0;
    s_fail = 0;

    test_checksum_worked_example();
    test_qb_build_read();
    test_decode_attr83();
    test_qb_build_write_rmw();
    test_qb_build_write_naive_contrast();
    test_qx_build_write();
    test_attr110_no_usable_data();
    test_decode_attr121();
    test_encode_clamp_and_nan();
    test_rmw_round_trip_motor_kill();
    test_rmw_timeout_fails_closed();
    test_rmw_busy_guard();

#ifdef HAMFLY_SETTINGS_TEST_STANDALONE
    printf("hamfly_settings_tests: %d passed, %d failed\n", s_pass, s_fail);
#endif
    return s_fail;
}

#ifdef HAMFLY_SETTINGS_TEST_STANDALONE
int main(void)
{
    return hamfly_settings_tests_run();
}
#endif
