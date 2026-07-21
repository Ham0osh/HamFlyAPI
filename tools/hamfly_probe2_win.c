/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 Hamish Johnson
 * Quantum Information Systems Lab, SFU Physics
 *
 * ============================================================================
 * READ-ONLY hardware probe #2 — does the EXISTING telemetry path work?
 *
 * Probe #1 proved: a QB-framed request gets a QB-framed reply, which
 * QX_RxMsg() discards. It did NOT test the path the firmware actually uses.
 *
 * hamfly_request_attr() emits QX framing (QX_InitMsg zeroes Legacy_Header).
 * If the gimbal replies in the same framing it was asked in, then QX request
 * -> QX reply -> parses fine, and existing telemetry reads work. That would
 * make F13 real but narrow: it would only affect QB requests, i.e. only the
 * new settings module.
 *
 * This tool runs the REAL code path end to end:
 *   1. hamfly_request_attr(attr)  -> capture the exact bytes via a HAL whose
 *      uart_putc appends to a buffer. These are the bytes the firmware sends.
 *   2. Transmit them on the serial port.
 *   3. Feed every received byte back through hamfly_on_rx_byte() + hamfly_pump()
 *      -- the genuine RX pipeline, unmodified.
 *   4. Report whether the telemetry struct actually populated.
 *
 * Then repeat with a QB request for comparison.
 *
 * SAFETY: READ-ONLY. Only read builders are used. No write builder is
 * referenced. A QX/QB read cannot change gimbal state.
 *
 * BUILD (from repo root):
 *   gcc -std=c99 -Isrc tools/hamfly_probe2_win.c src/hamfly_settings_wire.c \
 *       src/hamfly_settings_field.c src/hamfly_attr_table.c \
 *       src/hamfly_core_gimbal.c src/hamfly_core_telemetry.c \
 *       src/hamfly_qx_protocol.c src/hamfly_qx_app.c src/hamfly_comm_rb.c \
 *       -lm -o hamfly_probe2.exe
 *
 * RUN:  ./hamfly_probe2.exe COM6 111111
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>

#include "hamfly.h"
#include "hamfly_settings_wire.h"

#define TXCAP 256
#define RXCAP 512
#define RX_WAIT_MS 700

static uint8_t  g_tx[TXCAP];
static uint16_t g_tx_len;

/* HAL uart_putc that captures instead of transmitting, so we can see exactly
 * what the real code path emits before putting it on the wire. */
static void cap_putc(void *ctx, uint8_t b)
{
    (void)ctx;
    if (g_tx_len < TXCAP) g_tx[g_tx_len++] = b;
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
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = RX_WAIT_MS;
    to.WriteTotalTimeoutConstant = 500;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

static void hexdump(const char *label, const uint8_t *b, size_t n)
{
    printf("    %s (%zu): ", label, n);
    for (size_t i = 0; i < n; ++i) {
        printf("%02X ", b[i]);
        if ((i % 16) == 15 && i + 1 < n) printf("\n                  ");
    }
    printf("\n");
}

static const char *framing_of(const uint8_t *b, size_t n)
{
    for (size_t i = 0; i + 1 < n; ++i) {
        if (b[i] == 0x51u && b[i+1] == 0x58u) return "QX";
        if (b[i] == 0x51u && b[i+1] == 0x42u) return "QB";
    }
    return "(none)";
}

/* Run one exchange. build_qb = 0 -> real hamfly_request_attr (QX);
 *                    build_qb = 1 -> hamfly_qb_build_read (QB).            */
static void exchange(const char *port, uint32_t baud, uint16_t attr, int build_qb)
{
    hamfly_gimbal_t g;
    hamfly_hal_t hal;
    memset(&hal, 0, sizeof hal);
    hal.ctx = NULL;
    hal.uart_putc = cap_putc;
    hal.get_tick_ms = NULL;

    hamfly_init(&g, &hal);
    g_tx_len = 0;

    printf("  --- attr %u via %s ---\n", attr, build_qb ? "QB (settings module)"
                                                        : "QX (hamfly_request_attr)");

    if (build_qb) {
        uint8_t buf[64];
        uint8_t n = hamfly_qb_build_read(attr, buf, (uint8_t)sizeof buf);
        memcpy(g_tx, buf, n);
        g_tx_len = n;
    } else {
        hamfly_request_attr(&g, attr);   /* REAL path; cap_putc records bytes */
    }

    if (g_tx_len == 0) { printf("    no bytes produced\n\n"); return; }
    hexdump("request ", g_tx, g_tx_len);
    printf("    req framing: %s\n", framing_of(g_tx, g_tx_len));

    HANDLE h = open_port(port, baud);
    if (h == INVALID_HANDLE_VALUE) {
        printf("    could not open %s (err %lu)\n\n", port, (unsigned long)GetLastError());
        return;
    }
    DWORD wrote = 0;
    WriteFile(h, g_tx, g_tx_len, &wrote, NULL);

    uint8_t rx[RXCAP]; DWORD got = 0;
    ReadFile(h, rx, (DWORD)sizeof rx, &got, NULL);
    CloseHandle(h);

    if (got == 0) { printf("    reply: NONE\n\n"); return; }
    hexdump("reply   ", rx, (size_t)got);
    printf("    rep framing: %s\n", framing_of(rx, (size_t)got));

    /* Feed the reply through the genuine RX pipeline. */
    for (DWORD i = 0; i < got; ++i) hamfly_on_rx_byte(&g, rx[i]);
    hamfly_pump(&g);

    printf("    after hamfly_pump(): rx_packets=%lu rx_bad_checksum=%lu pending_ready=%s\n",
           (unsigned long)g.statistics.rx_packets,
           (unsigned long)g.statistics.rx_bad_checksum,
           g.pending_ready ? "TRUE" : "false");
    printf("    telemetry valid flags: sysstat=%d baro=%d gps=%d att=%d mag=%d sysecho=%d qx287=%d\n",
           g.telemetry.sysstat_valid, g.telemetry.baro_valid, g.telemetry.gps_valid,
           g.telemetry.att_valid, g.telemetry.mag_valid, g.telemetry.sysecho_valid,
           g.telemetry.valid);
    if (g.telemetry.sysstat_valid) {
        printf("    >>> DECODED sysstat: batt=%.2fV sats=%u temp=%.1fC cpu=%u%%\n",
               (double)g.telemetry.sysstat_batt_v, g.telemetry.sysstat_gps_sats,
               (double)g.telemetry.sysstat_temp_c, g.telemetry.sysstat_cpu_pct);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "COM6";
    uint32_t baud    = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 111111u;

    printf("HamFly probe #2 — real-path telemetry test\n");
    printf("port %s, baud %lu. READ-ONLY.\n\n", port, (unsigned long)baud);

    /* attr 1 = SysStat: a real sensor attribute with a decoder, id <= 255. */
    exchange(port, baud, 1u, 0);   /* QX via real hamfly_request_attr */
    exchange(port, baud, 1u, 1);   /* QB via settings module          */

    /* attr 121 = identity, for cross-check against the golden vector. */
    exchange(port, baud, 121u, 0);

    printf("Done. Nothing was written to the gimbal.\n");
    return 0;
}
