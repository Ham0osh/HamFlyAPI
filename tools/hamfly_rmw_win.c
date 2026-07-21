/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson
 * Quantum Information Systems Lab, SFU Physics
 *
 * ============================================================================
 * Windows host harness for the v2 read-modify-write engine — FIRST HARDWARE
 * WRITE TEST.
 *
 * Runs the real hamfly_settings_write_start/_poll state machine over a real
 * serial link, with a real hal.get_tick_ms (which also exercises the
 * millisecond-deadline branch that the offline suite never covered — its test
 * HAL deliberately pins get_tick_ms = NULL).
 *
 * MODES
 *   read                     dump attr 83's six fields, change nothing
 *   write <field> <value>    guarded cycle: read -> RMW write -> verify
 *                            read-back -> RESTORE original -> verify restore
 *
 * WHY THE RESTORE MATTERS
 *   Attr 83 is a six-field struct: Max Control Rate, (unnamed offset 3),
 *   Max/Min Tilt, Max Roll, Motor Kill. A botched RMW zeroes the siblings —
 *   i.e. silently destroys the tilt and roll limits. This harness prints every
 *   sibling before and after so that failure is visible immediately, and always
 *   attempts to put the original value back.
 *
 * BUILD (from repo root):
 *   gcc -std=c99 -Isrc tools/hamfly_rmw_win.c src/hamfly_settings_rmw.c \
 *       src/hamfly_settings_wire.c src/hamfly_settings_field.c \
 *       src/hamfly_attr_table.c src/hamfly_core_gimbal.c \
 *       src/hamfly_core_telemetry.c src/hamfly_qx_protocol.c \
 *       src/hamfly_qx_app.c src/hamfly_comm_rb.c -lm -o hamfly_rmw.exe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>

#include "hamfly.h"
#include "hamfly_settings_wire.h"
#include "hamfly_settings_field.h"
#include "hamfly_settings_rmw.h"
#include "hamfly_attr_table.h"

#define ATTR 83u

static HANDLE g_port = INVALID_HANDLE_VALUE;

/* ---- HAL ---------------------------------------------------------------- */

static void ser_putc(void *ctx, uint8_t b)
{
    (void)ctx;
    DWORD w = 0;
    WriteFile(g_port, &b, 1, &w, NULL);
}

static uint32_t ser_ticks(void *ctx)
{
    (void)ctx;
    return (uint32_t)GetTickCount();
}

static HANDLE open_port(const char *port, uint32_t baud)
{
    char path[32];
    snprintf(path, sizeof path, "\\\\.\\%s", port);
    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) return h;

    DCB dcb; memset(&dcb, 0, sizeof dcb); dcb.DCBlength = sizeof dcb;
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }
    dcb.BaudRate = baud; dcb.ByteSize = 8;
    dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT; dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    COMMTIMEOUTS to; memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout = MAXDWORD;      /* return immediately */
    to.ReadTotalTimeoutConstant = 0;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

/* Drain whatever the port has into the gimbal's RX ring, then pump. */
static void service(hamfly_gimbal_t *g)
{
    uint8_t buf[256];
    DWORD got = 0;
    if (ReadFile(g_port, buf, (DWORD)sizeof buf, &got, NULL) && got) {
        for (DWORD i = 0; i < got; ++i) hamfly_on_rx_byte(g, buf[i]);
    }
    hamfly_pump(g);
}

/* ---- attr 83 reporting -------------------------------------------------- */

static const char *FIELDS[] = {
    "Max Control Rate", "Max Tilt Angle", "Min Tilt Angle",
    "Max Roll Angle",   "Motor Kill"
};
#define NFIELDS (sizeof FIELDS / sizeof FIELDS[0])

/* Blocking-ish read of attr 83 into buf. Returns payload length, 0 on failure.
 * Rule 1: retries, because a busy gimbal misses the first request. */
static uint8_t read_attr83(hamfly_gimbal_t *g, uint8_t *out, uint8_t out_max)
{
    for (int attempt = 1; attempt <= 4; ++attempt) {
        g->pending_attr  = 0u;
        g->pending_ready = false;
        if (hamfly_settings_send_qb_read(g, ATTR) != HAMFLY_OK) continue;

        DWORD deadline = GetTickCount() + 400u;
        while (GetTickCount() < deadline) {
            service(g);
            if (g->pending_ready) {
                uint8_t n = g->pending_payload_len;
                if (n > out_max) n = out_max;
                memcpy(out, g->pending_payload, n);
                return n;
            }
            Sleep(2);
        }
        printf("      (read attempt %d timed out — Rule 1 retry)\n", attempt);
    }
    return 0u;
}

static void dump83(const uint8_t *pay, uint8_t len)
{
    printf("      raw payload (%u): ", len);
    for (uint8_t i = 0; i < len; ++i) printf("%02X ", pay[i]);
    printf("\n");
    for (size_t i = 0; i < NFIELDS; ++i) {
        float v;
        if (hamfly_settings_get(ATTR, FIELDS[i], pay, len, &v))
            printf("      %-18s = %g\n", FIELDS[i], (double)v);
        else
            printf("      %-18s = <not present in this payload>\n", FIELDS[i]);
    }
}

static const char *st_name(hamfly_settings_rmw_state_t s)
{
    switch (s) {
    case HAMFLY_SETTINGS_RMW_OK:                return "OK";
    case HAMFLY_SETTINGS_RMW_READING:           return "READING";
    case HAMFLY_SETTINGS_RMW_VERIFYING:         return "VERIFYING";
    case HAMFLY_SETTINGS_RMW_IDLE:              return "IDLE";
    case HAMFLY_SETTINGS_RMW_ERR_ARG:           return "ERR_ARG";
    case HAMFLY_SETTINGS_RMW_ERR_UNKNOWN_FIELD: return "ERR_UNKNOWN_FIELD";
    case HAMFLY_SETTINGS_RMW_ERR_NOT_WRITABLE:  return "ERR_NOT_WRITABLE";
    case HAMFLY_SETTINGS_RMW_ERR_RANGE:         return "ERR_RANGE";
    case HAMFLY_SETTINGS_RMW_ERR_BUSY:          return "ERR_BUSY";
    case HAMFLY_SETTINGS_RMW_ERR_NO_READ:       return "ERR_NO_READ (failed closed)";
    case HAMFLY_SETTINGS_RMW_ERR_TOO_SHORT:     return "ERR_TOO_SHORT";
    case HAMFLY_SETTINGS_RMW_ERR_TX:            return "ERR_TX";
    case HAMFLY_SETTINGS_RMW_ERR_UNVERIFIED:    return "ERR_UNVERIFIED";
    default:                                    return "?";
    }
}

/* Drive one RMW transaction to completion. */
static hamfly_settings_rmw_state_t do_write(hamfly_gimbal_t *g,
                                            const char *field, float value)
{
    hamfly_settings_rmw_t txn;
    memset(&txn, 0, sizeof txn);

    hamfly_settings_rmw_state_t s =
        hamfly_settings_write_start(&txn, g, ATTR, field, value, true);
    printf("      write_start -> %s\n", st_name(s));

    DWORD deadline = GetTickCount() + 5000u;
    while (!hamfly_settings_write_done(&txn) && GetTickCount() < deadline) {
        service(g);
        s = hamfly_settings_write_poll(&txn);
        Sleep(2);
    }
    printf("      final state -> %s\n", st_name(hamfly_settings_write_poll(&txn)));
    return hamfly_settings_write_poll(&txn);
}

int main(int argc, char **argv)
{
    const char *port  = (argc > 1) ? argv[1] : "COM6";
    uint32_t    baud  = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 111111u;
    const char *mode  = (argc > 3) ? argv[3] : "read";
    const char *field = (argc > 4) ? argv[4] : "Max Control Rate";
    float       newv  = (argc > 5) ? (float)atof(argv[5]) : 0.0f;

    g_port = open_port(port, baud);
    if (g_port == INVALID_HANDLE_VALUE) {
        printf("could not open %s (err %lu)\n", port, (unsigned long)GetLastError());
        return 2;
    }

    hamfly_gimbal_t g;
    hamfly_hal_t hal;
    memset(&hal, 0, sizeof hal);
    hal.uart_putc   = ser_putc;
    hal.get_tick_ms = ser_ticks;   /* exercises the real-clock timeout branch */
    hamfly_init(&g, &hal);

    printf("HamFly RMW harness — %s @ %lu, attr %u\n\n",
           port, (unsigned long)baud, ATTR);

    uint8_t before[64];
    printf("[1] READ current state\n");
    uint8_t n = read_attr83(&g, before, (uint8_t)sizeof before);
    if (n == 0) { printf("      READ FAILED — aborting, nothing written.\n"); goto done; }
    dump83(before, n);

    if (strcmp(mode, "read") == 0) {
        printf("\nread-only mode: nothing written.\n");
        goto done;
    }

    float orig;
    if (!hamfly_settings_get(ATTR, field, before, n, &orig)) {
        printf("      cannot read '%s' — aborting.\n", field);
        goto done;
    }

    printf("\n[2] WRITE %s: %g -> %g\n", field, (double)orig, (double)newv);
    hamfly_settings_rmw_state_t s = do_write(&g, field, newv);

    printf("\n[3] READ BACK\n");
    uint8_t after[64];
    uint8_t n2 = read_attr83(&g, after, (uint8_t)sizeof after);
    if (n2) dump83(after, n2);

    printf("\n[4] RESTORE %s -> %g\n", field, (double)orig);
    hamfly_settings_rmw_state_t r = do_write(&g, field, orig);

    printf("\n[5] VERIFY RESTORE\n");
    uint8_t fin[64];
    uint8_t n3 = read_attr83(&g, fin, (uint8_t)sizeof fin);
    if (n3) dump83(fin, n3);

    printf("\n=== SUMMARY ===\n");
    printf("  write : %s\n", st_name(s));
    printf("  restore: %s\n", st_name(r));
    if (n3 == n && memcmp(before, fin, n) == 0)
        printf("  STRUCT IDENTICAL to start — no siblings disturbed. \n");
    else
        printf("  *** STRUCT DIFFERS FROM START — INSPECT ABOVE ***\n");

done:
    CloseHandle(g_port);
    return 0;
}
