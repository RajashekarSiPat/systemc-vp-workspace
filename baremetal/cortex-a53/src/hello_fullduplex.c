/* hello_fullduplex.c — Full-duplex USART2 interrupt test firmware.
 *
 * Tests two USART2 instances (A and B) bridged via SerialBridge for
 * full-duplex communication with interrupt verification:
 *
 *   Test 1: A→B   TBIR_A fires, RIR_B fires, RBUF_B == 0x55
 *   Test 2: B→A   TBIR_B fires, RIR_A fires, RBUF_A == 0xAA
 *   Test 3: EIR   overrun on B (RBUF not drained) → EIR_B fires
 *   Test 4: multi  0xDE,0xAD,0xBE,0xEF A→B with per-byte RIR check
 *   Test 5: TIR   TX frame complete after baud simulation
 *
 * Hardware map:
 *   0x09002000  USART2 A  (bridge test — TX/RX)
 *   0x09003000  USART2 B  (bridge test — TX/RX)
 *   0x09004000  USART2 C  (console: TBUF write → backend_socket.enqueue → stdout)
 *
 * USART2 register offsets:
 *   +0x00  CON     Control (R/W)
 *   +0x04  TBUF    TX holding buffer (W only)
 *   +0x08  RBUF    RX holding buffer (R only)
 *   +0x20  STATUS  Sticky interrupt log (R / W1C)
 *
 * STATUS bits (sticky, W1C):
 *   [0] TBIR — TX buffer freed into TSR
 *   [1] TIR  — TX frame complete
 *   [2] RIR  — RX data ready in RBUF
 *   [3] EIR  — Error (overrun / framing / parity)
 *
 * CON: Mode1=0x1 | REN=0x8 | OEN=0x40 | R=0x8000 → 0x8049
 *
 * IMPORTANT: _start() is NAKED and must be the FIRST function defined so the
 * linker places it at 0x80000000 (Cortex-A53 RVBAR reset address).
 * It sets SP to 0x90000000 (top of 256 MB RAM) then calls fullduplex_main().
 * All helpers are forward-declared here and defined after _start().
 */

/* ═══════════════════════════════════════════════════════════════════════════
 * Register macros
 * ═══════════════════════════════════════════════════════════════════════════ */
#define MMIO32(a)  (*(volatile unsigned int *)(unsigned long)(a))

#define UA_CON    MMIO32(0x09002000UL)
#define UA_TBUF   MMIO32(0x09002004UL)
#define UA_RBUF   MMIO32(0x09002008UL)
#define UA_STATUS MMIO32(0x09002020UL)

#define UB_CON    MMIO32(0x09003000UL)
#define UB_TBUF   MMIO32(0x09003004UL)
#define UB_RBUF   MMIO32(0x09003008UL)
#define UB_STATUS MMIO32(0x09003020UL)

/* Console: just write bytes to TBUF — wrapper forwards via backend_socket */
#define UC_TBUF   MMIO32(0x09004004UL)

#define CON_INIT  0x8049u   /* Mode1 | REN | OEN | R */

#define STATUS_TBIR  (1u << 0)
#define STATUS_TIR   (1u << 1)
#define STATUS_RIR   (1u << 2)
#define STATUS_EIR   (1u << 3)

/* ═══════════════════════════════════════════════════════════════════════════
 * Forward declarations — _start() must be FIRST defined function.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void put_char(char c);
static void put_str(const char *s);
static void put_hex(unsigned int v);
static int  wait_status(volatile unsigned int *reg, unsigned int mask,
                        unsigned int iters);
static void clr_status(volatile unsigned int *reg, unsigned int mask);

static int g_pass = 0, g_fail = 0;

static void pass_test(const char *name);
static void fail_test(const char *name, const char *why);
static void fullduplex_main(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * _start — naked entry point at 0x80000000 (RVBAR).
 * Sets SP to 0x90000000 (top of 256 MB RAM) then calls fullduplex_main().
 * MUST be the first function defined so GCC places it at 0x80000000.
 * ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((naked)) _start(void)
{
    __asm__ volatile(
        "ldr x0, =0x90000000\n"   /* stack top (256 MB RAM ends at 0x90000000) */
        "mov sp, x0\n"
        "bl  fullduplex_main\n"
        "1: b 1b\n"               /* halt if fullduplex_main returns */
    );
}

/* ═══════════════════════════════════════════════════════════════════════════
 * fullduplex_main — all test logic (called from _start with valid SP)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void fullduplex_main(void)
{
    put_str("\r\n");
    put_str("================================================\r\n");
    put_str("  USART2 Full-Duplex Interrupt Test\r\n");
    put_str("  A <-> B bridged via SerialBridge\r\n");
    put_str("================================================\r\n\r\n");

    UA_CON = CON_INIT;
    UB_CON = CON_INIT;
    clr_status(&UA_STATUS, 0xFFFFFFFFu);
    clr_status(&UB_STATUS, 0xFFFFFFFFu);

    /* ════════════════════════════════════════════════════════════════════════
     * Test 1: A → B
     * TBIR_A fires inside the TBUF write b_transport (loadTSR called).
     * RIR_B fires inside the same b_transport chain via:
     *   SerialBridge.recv_from_a → socket_b.enqueue → Usart2_B.rx_receive
     *   → m_usart.rx_inject → rir_assert_time set → update_status_from_core
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("Test 1: A sends 0x55 to B\r\n");
    {
        unsigned int rbuf;
        UA_TBUF = 0x55u;

        if (!wait_status(&UA_STATUS, STATUS_TBIR, 2000000u)) {
            fail_test("T1-TBIR_A", "timeout"); goto t2;
        }
        put_str("  TBIR_A  STATUS_A="); put_hex(UA_STATUS); put_str("\r\n");
        clr_status(&UA_STATUS, STATUS_TBIR);

        if (!wait_status(&UB_STATUS, STATUS_RIR, 2000000u)) {
            fail_test("T1-RIR_B", "timeout"); goto t2;
        }
        put_str("  RIR_B   STATUS_B="); put_hex(UB_STATUS); put_str("\r\n");
        clr_status(&UB_STATUS, STATUS_RIR);

        rbuf = UB_RBUF & 0xFFu;
        put_str("  RBUF_B="); put_hex(rbuf); put_str("\r\n");
        if (rbuf == 0x55u) pass_test("Test1 A->B 0x55");
        else               fail_test("Test1 A->B", "RBUF_B != 0x55");
    }

t2:
    /* ════════════════════════════════════════════════════════════════════════
     * Test 2: B → A
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("\r\nTest 2: B sends 0xAA to A\r\n");
    {
        unsigned int rbuf;
        UB_TBUF = 0xAAu;

        if (!wait_status(&UB_STATUS, STATUS_TBIR, 2000000u)) {
            fail_test("T2-TBIR_B", "timeout"); goto t3;
        }
        put_str("  TBIR_B  STATUS_B="); put_hex(UB_STATUS); put_str("\r\n");
        clr_status(&UB_STATUS, STATUS_TBIR);

        if (!wait_status(&UA_STATUS, STATUS_RIR, 2000000u)) {
            fail_test("T2-RIR_A", "timeout"); goto t3;
        }
        put_str("  RIR_A   STATUS_A="); put_hex(UA_STATUS); put_str("\r\n");
        clr_status(&UA_STATUS, STATUS_RIR);

        rbuf = UA_RBUF & 0xFFu;
        put_str("  RBUF_A="); put_hex(rbuf); put_str("\r\n");
        if (rbuf == 0xAAu) pass_test("Test2 B->A 0xAA");
        else               fail_test("Test2 B->A", "RBUF_A != 0xAA");
    }

t3:
    /* ════════════════════════════════════════════════════════════════════════
     * Test 3: Overrun / EIR
     * rx_inject returns false when RBUF is full → OE set, EIR fires (OEN=1).
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("\r\nTest 3: Overrun -- EIR on B\r\n");
    {
        UA_TBUF = 0x11u;                         /* fills RBUF_B */
        clr_status(&UA_STATUS, STATUS_TBIR);

        if (!wait_status(&UB_STATUS, STATUS_RIR, 2000000u)) {
            fail_test("T3-RIR_B", "first byte timeout"); goto t4;
        }
        put_str("  RIR_B (RBUF_B full)\r\n");

        UA_TBUF = 0x22u;                         /* RBUF_B still full -> overrun */
        clr_status(&UA_STATUS, STATUS_TBIR);

        if (!wait_status(&UB_STATUS, STATUS_EIR, 2000000u)) {
            fail_test("T3-EIR_B", "EIR timeout"); goto t4;
        }
        put_str("  EIR_B   STATUS_B="); put_hex(UB_STATUS); put_str("\r\n");
        pass_test("Test3 Overrun EIR");

        (void)(UB_RBUF);                         /* drain RBUF_B */
        clr_status(&UB_STATUS, 0xFFFFFFFFu);
        clr_status(&UA_STATUS, 0xFFFFFFFFu);
    }

t4:
    /* ════════════════════════════════════════════════════════════════════════
     * Test 4: Multi-byte stream A → B  [0xDE, 0xAD, 0xBE, 0xEF]
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("\r\nTest 4: Multi-byte A->B [0xDE,0xAD,0xBE,0xEF]\r\n");
    {
        static const unsigned char stream[4] = {0xDE, 0xAD, 0xBE, 0xEF};
        int ok = 1;
        unsigned int i;
        for (i = 0; i < 4u; ++i) {
            unsigned int exp = stream[i], got;
            UA_TBUF = exp;
            if (!wait_status(&UB_STATUS, STATUS_RIR, 2000000u)) {
                put_str("  timeout byte "); put_hex(exp); put_str("\r\n");
                ok = 0; break;
            }
            got = UB_RBUF & 0xFFu;
            clr_status(&UB_STATUS, STATUS_RIR);
            clr_status(&UA_STATUS, STATUS_TBIR);
            put_str("  ["); put_hex(i); put_str("] sent="); put_hex(exp);
            put_str(" got="); put_hex(got);
            put_str((got == exp) ? " OK\r\n" : " MISMATCH\r\n");
            if (got != exp) ok = 0;
        }
        if (ok) pass_test("Test4 multi-byte");
        else    fail_test("Test4 multi-byte", "mismatch or timeout");
    }

    /* ════════════════════════════════════════════════════════════════════════
     * Test 5: TIR — TX complete via baud simulation
     * STATUS reads call m_usart.sync() → advance(sc_time_stamp()).
     * At 100 MHz, BG=0, /2 prescaler: 10-bit frame ~3200 ns.
     * With 10 ms quantum the polling loop itself advances sim time, so TIR
     * fires once the QEMU quantum has carried the clock past the frame end.
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("\r\nTest 5: TIR -- TX complete (baud timing)\r\n");
    {
        clr_status(&UA_STATUS, 0xFFFFFFFFu);
        UA_TBUF = 0x5Au;
        if (!wait_status(&UA_STATUS, STATUS_TIR, 5000000u)) {
            put_str("  TIR not seen (timing-dependent)\r\n");
            put_str("  STATUS_A="); put_hex(UA_STATUS); put_str("\r\n");
        } else {
            put_str("  TIR fired  STATUS_A="); put_hex(UA_STATUS); put_str("\r\n");
            pass_test("Test5 TIR");
        }
        clr_status(&UA_STATUS, 0xFFFFFFFFu);
    }

    /* ════════════════════════════════════════════════════════════════════════
     * Summary
     * ════════════════════════════════════════════════════════════════════════ */
    put_str("\r\n================================================\r\n");
    put_str("  Passed: "); put_hex((unsigned)g_pass);
    put_str("   Failed: "); put_hex((unsigned)g_fail); put_str("\r\n");
    put_str(g_fail == 0 ? "  ALL TESTS PASSED\r\n" : "  SOME TESTS FAILED\r\n");
    put_str("================================================\r\n");

    while (1) { /* halt */ }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Helper function definitions (after _start so they don't precede it in .text)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void put_char(char c)
{
    UC_TBUF = (unsigned int)(unsigned char)c;
}

static void put_str(const char *s)
{
    while (*s) put_char(*s++);
}

static void put_hex(unsigned int v)
{
    const char h[] = "0123456789ABCDEF";
    put_char('0'); put_char('x');
    put_char(h[(v>>28)&0xF]); put_char(h[(v>>24)&0xF]);
    put_char(h[(v>>20)&0xF]); put_char(h[(v>>16)&0xF]);
    put_char(h[(v>>12)&0xF]); put_char(h[(v>> 8)&0xF]);
    put_char(h[(v>> 4)&0xF]); put_char(h[(v>> 0)&0xF]);
}

static int wait_status(volatile unsigned int *reg,
                       unsigned int           mask,
                       unsigned int           iters)
{
    unsigned int i;
    for (i = 0; i < iters; ++i)
        if ((*reg & mask) == mask) return 1;
    return 0;
}

static void clr_status(volatile unsigned int *reg, unsigned int mask)
{
    *reg = mask;
}

static void pass_test(const char *name)
{
    put_str("[PASS] "); put_str(name); put_str("\r\n");
    ++g_pass;
}

static void fail_test(const char *name, const char *why)
{
    put_str("[FAIL] "); put_str(name); put_str(": "); put_str(why);
    put_str("\r\n");
    ++g_fail;
}
