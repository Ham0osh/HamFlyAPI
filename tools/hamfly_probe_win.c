/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson
 * Quantum Information Systems Lab, SFU Physics
 *
 * ============================================================================
 * READ-ONLY hardware probe — Windows host harness for HamFlyAPI.
 *
 * PURPOSE
 *   Answer finding F13 empirically: does the MoVI Pro reply in QB framing for
 *   attribute ids <= 255? If it does, telemetry reads via hamfly_request_attr()
 *   have never worked on hardware, because QX_RxMsg() drops every QB frame
 *   (QX_ParseHeader_Legacy is NULL — see build/review/findings.md F13).
 *
 * SAFETY — this tool CANNOT write to the gimbal.
 *   It calls hamfly_qb_build_read() / hamfly_qx_build_read_like() only. The
 *   write builders (hamfly_qb_build_write, hamfly_qx_build_write) are never
 *   referenced. A QX/QB READ cannot change gimbal state. There is no code path
 *   here that emits a WRITE_ABS type byte.
 *
 * WHAT IT DOES
 *   1. Opens the given COM port at each candidate baud in turn.
 *   2. Sends a READ request for attr 121 (Device Serial Number / firmware) —
 *      chosen because it is read-only AND we know the expected answer from the
 *      golden vectors: serial 4122644516, firmware 2.3.5. That makes the reply
 *      self-verifying.
 *   3. Dumps every received byte as raw hex FIRST (ground truth, independent of
 *      our parser being correct), then reports the framing it observed.
 *
 * BUILD (from repo root):
 *   gcc -std=c99 -Isrc tools/hamfly_probe_win.c src/hamfly_settings_wire.c \
 *       src/hamfly_settings_field.c src/hamfly_attr_table.c \
 *       src/hamfly_core_gimbal.c src/hamfly_core_telemetry.c \
 *       src/hamfly_qx_protocol.c src/hamfly_qx_app.c src/hamfly_comm_rb.c \
 *       -lm -o hamfly_probe.exe
 *
 * RUN:
 *   ./hamfly_probe.exe COM6            (sweep candidate bauds)
 *   ./hamfly_probe.exe COM6 115200     (pin one baud)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>

#include "hamfly_settings_wire.h"

static const uint32_t CANDIDATE_BAUDS[] = {
    115200u, 57600u, 230400u, 38400u, 19200u, 9600u
};
#define N_BAUDS (sizeof CANDIDATE_BAUDS / sizeof CANDIDATE_BAUDS[0])

/* Attribute to probe. Read-only, and its expected content is known from the
 * golden vectors, so a correct reply proves framing AND decode in one shot. */
#define PROBE_ATTR 121u

#define RX_WAIT_MS 700
#define RX_CAP     512

static HANDLE open_port(const char *port, uint32_t baud)
{
    char path[32];
    snprintf(path, sizeof path, "\\\\.\\%s", port);

    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) return h;

    DCB dcb;
    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    /* No flow control — the GCU serial link is plain 8N1. */
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    COMMTIMEOUTS to;
    memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout         = 50;
    to.ReadTotalTimeoutConstant    = RX_WAIT_MS;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.WriteTotalTimeoutConstant   = 500;
    SetCommTimeouts(h, &to);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

static void hexdump(const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        printf("%02X ", b[i]);
        if ((i % 16) == 15) printf("\n                ");
    }
    printf("\n");
}

/* Report what framing the reply used, without trusting our own parser. */
static void classify(const uint8_t *b, size_t n)
{
    if (n == 0) { printf("      framing : (no bytes)\n"); return; }

    size_t q = (size_t)-1;
    for (size_t i = 0; i + 1 < n; ++i) {
        if (b[i] == 0x51u && (b[i+1] == 0x58u || b[i+1] == 0x42u)) { q = i; break; }
    }
    if (q == (size_t)-1) {
        printf("      framing : NO 'Q'+('X'|'B') sync found — likely wrong baud or noise\n");
        return;
    }
    printf("      framing : sync at offset %zu -> 'Q'%c  ==> %s\n",
           q, (char)b[q+1],
           (b[q+1] == 0x42u) ? "*** QB (legacy header) ***" : "QX");
    if (b[q+1] == 0x42u) {
        printf("                CONFIRMS F13: attr %u replies in QB. QX_RxMsg() would\n"
               "                drop this frame (QX_ParseHeader_Legacy == NULL).\n", PROBE_ATTR);
    }
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "COM6";
    uint32_t pinned  = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 0u;

    uint8_t req[32];
    uint8_t req_len = hamfly_qb_build_read(PROBE_ATTR, req, (uint8_t)sizeof req);
    if (req_len == 0u) {
        printf("FATAL: hamfly_qb_build_read() produced no frame\n");
        return 2;
    }

    printf("HamFly read-only probe — port %s, attr %u\n", port, PROBE_ATTR);
    printf("READ-ONLY: no write builder is linked into this path.\n\n");
    printf("request (%u bytes): ", req_len);
    hexdump(req, req_len);
    printf("\n");

    for (size_t i = 0; i < N_BAUDS; ++i) {
        uint32_t baud = CANDIDATE_BAUDS[i];
        if (pinned && baud != pinned) continue;

        printf("  baud %6lu : ", (unsigned long)baud);
        fflush(stdout);

        HANDLE h = open_port(port, baud);
        if (h == INVALID_HANDLE_VALUE) {
            printf("could not open port (err %lu)\n", (unsigned long)GetLastError());
            continue;
        }

        DWORD wrote = 0;
        if (!WriteFile(h, req, req_len, &wrote, NULL) || wrote != req_len) {
            printf("write failed (err %lu)\n", (unsigned long)GetLastError());
            CloseHandle(h);
            continue;
        }

        uint8_t rx[RX_CAP];
        DWORD got = 0;
        ReadFile(h, rx, (DWORD)sizeof rx, &got, NULL);
        CloseHandle(h);

        if (got == 0) { printf("no reply\n"); continue; }

        printf("%lu bytes\n", (unsigned long)got);
        printf("      raw     : ");
        hexdump(rx, (size_t)got);
        classify(rx, (size_t)got);
        printf("\n");
    }

    printf("Done. Nothing was written to the gimbal.\n");
    return 0;
}
