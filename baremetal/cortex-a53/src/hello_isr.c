/* hello_isr.c — AArch64 ISR-based USART2 interrupt test
 *
 * Platform: conf_isr.lua  (Cortex-A53 EL1, GICv3, two bridged USART2s)
 *
 * Each USART2 has a combined irq output wired to a GIC SPI:
 *   usart2_a.irq → GIC SPI 0 → IRQ ID 32
 *   usart2_b.irq → GIC SPI 1 → IRQ ID 33
 *
 * Interrupt delivery chain:
 *   USART event (TBIR/TIR/RIR/EIR) fires in advance() inside b_transport.
 *   Usart core asserts m_sig_xxx → irq_method() (SC_METHOD) fires →
 *   irq output port writes true → QemuTargetSignalSocket notifies QEMU →
 *   GIC SPI asserts (edge-triggered, GICD_ICFGR) →
 *   CPU takes AArch64 IRQ exception at EL1 (VBAR_EL1 + 0x280) →
 *   irq_entry() saves context, calls irq_handler() →
 *   irq_handler() reads GICC_IAR, reads + W1C USART STATUS, logs each set bit,
 *   writes GICC_EOIR → ERET back to interrupted code.
 *
 * ISR event log records one entry per STATUS bit per interrupt call.
 * Because advance() may process many baud ticks at once (lazy model), a
 * single TBUF write can fire TBIR and TIR together in one IRQ; the ISR
 * splits them into two log entries.
 *
 * Tests:
 *   1. A→B 0x55 :  TBIR_A fires, RIR_B fires, RBUF_B == 0x55
 *   2. B→A 0xAA :  TBIR_B fires, RIR_A fires, RBUF_A == 0xAA
 *   3. Overrun  :  RIR_B (first byte), EIR_B (second byte, RBUF full)
 *   4. Multi    :  0xDE,0xAD,0xBE,0xEF A→B, each byte TBIR_A + RIR_B
 *   5. TIR      :  TX-complete via baud timing (may appear in same ISR as TBIR)
 *
 * Selecting tests at compile time:
 *   TEST_MASK bit N-1 enables Test N.  Default 0x1F runs all five.
 *   To run only Test 3:  -DTEST_MASK=0x04
 *   To run Tests 1 and 2: -DTEST_MASK=0x03
 *
 * GICv3 register addresses (from conf_isr.lua):
 *   GICD base: 0x08000000   GICR base: 0x080A0000
 *   CPU interface: AArch64 system registers (ICC_*_EL1)
 *
 * USART2 register map:
 *   +0x00 CON   +0x04 TBUF   +0x08 RBUF   +0x20 STATUS (sticky W1C)
 *   STATUS bits: [0] TBIR  [1] TIR  [2] RIR  [3] EIR
 *
 * Memory layout:
 *   0x80000000  _start (code)
 *   BSS region  g_vt[512] aligned to 2 KB — exception vector table
 */

/* Test selection mask — placed at a fixed address (0x80FFFF00) so
 * conf_isr.lua can patch it at run time via TEST_MASK env var.
 * No recompile needed to select a different test combination.      */
volatile unsigned int g_test_mask
    __attribute__((section(".test_cfg"))) = 0x1Fu;

/* ── MMIO helpers ─────────────────────────────────────────────────────────── */
#define MMIO32(a)  (*(volatile unsigned int *)(unsigned long)(a))

/* ── USART register addresses ────────────────────────────────────────────── */
#define UA_CON    MMIO32(0x09002000UL)
#define UA_TBUF   MMIO32(0x09002004UL)
#define UA_RBUF   MMIO32(0x09002008UL)
#define UA_STATUS MMIO32(0x09002020UL)

#define UB_CON    MMIO32(0x09003000UL)
#define UB_TBUF   MMIO32(0x09003004UL)
#define UB_RBUF   MMIO32(0x09003008UL)
#define UB_STATUS MMIO32(0x09003020UL)

#define UC_TBUF   MMIO32(0x09004004UL)

/* Exiter peripheral: any write calls sc_stop() in SystemC, ending the VP. */
#define EXITER    MMIO32(0x09010000UL)

/* CON_INIT: Mode1 | REN | OEN | R (8N1, receive enable, overrun-enable, run) */
#define CON_INIT     0x8049u

#define STATUS_TBIR  (1u << 0)
#define STATUS_TIR   (1u << 1)
#define STATUS_RIR   (1u << 2)
#define STATUS_EIR   (1u << 3)

/* ── GICv3 register addresses ────────────────────────────────────────────── */
/* Distributor (GICD) at 0x08000000, Redistributor (GICR) at 0x080A0000.    */
#define GICD_BASE  0x08000000UL
#define GICR_BASE  0x080A0000UL   /* 1-CPU GICR: RD frame at +0, SGI at +0x10000 */

/* Distributor MMIO registers */
#define GICD_CTLR        MMIO32(GICD_BASE + 0x000U) /* Distributor control        */
#define GICD_IGROUPR1    MMIO32(GICD_BASE + 0x084U) /* Group bits IRQ 32-63 (1=Grp1=IRQ) */
#define GICD_ISENABLER1  MMIO32(GICD_BASE + 0x104U) /* IRQ 32-63 enable bits      */
#define GICD_ICFGR2      MMIO32(GICD_BASE + 0xC08U) /* Config IRQ 32-47 (edge/lvl)*/
#define GICD_IPRIORITYR8 MMIO32(GICD_BASE + 0x420U) /* Priority bytes IRQ 32-35   */

/* GICD_IROUTER: 64-bit per-SPI affinity routing (GICv3 with ARE enabled).
 * IROUTER(n) at GICD_BASE + 0x6000 + n*8.  Value 0x0 = CPU affinity 0.0.0.0. */
#define MMIO64(a)  (*(volatile unsigned long long *)(unsigned long)(a))
#define GICD_IROUTER32  MMIO64(GICD_BASE + 0x6100UL)  /* SPI 0 → IRQ 32 */
#define GICD_IROUTER33  MMIO64(GICD_BASE + 0x6108UL)  /* SPI 1 → IRQ 33 */

/* Redistributor MMIO (RD frame): GICR_WAKER for power-management handshake. */
#define GICR_WAKER  MMIO32(GICR_BASE + 0x0014U)  /* ProcessorSleep[1], ChildrenAsleep[2] */

/* GICv3 CPU interface: accessed via AArch64 system registers.
 * ICC_SRE_EL1     = S3_0_C12_C12_5  (enable system-register interface)
 * ICC_PMR_EL1     = S3_0_C4_C6_0    (priority mask)
 * ICC_IGRPEN1_EL1 = S3_0_C12_C12_7  (group-1 interrupt enable)
 * ICC_IAR1_EL1    = S3_0_C12_C12_0  (acknowledge: returns INTID)
 * ICC_EOIR1_EL1   = S3_0_C12_C12_1  (end-of-interrupt + deactivate)
 * INTID field: bits [23:0]; spurious = 1023 (0x3FF).                        */

#define IRQ_USART_A   32u    /* GIC SPI 0 — usart2_a combined irq */
#define IRQ_USART_B   33u    /* GIC SPI 1 — usart2_b combined irq */
#define IRQ_SPURIOUS  1023u

/* ── ISR event log ────────────────────────────────────────────────────────── */
/* Each log entry records: which IRQ line fired and which STATUS bit was set. */

#define LOG_SIZE 64u

typedef struct {
    unsigned int irq_id;    /* 32 = USART_A, 33 = USART_B              */
    unsigned int status_bit;/* exactly one of STATUS_TBIR/TIR/RIR/EIR  */
} IrqEvent;

static volatile IrqEvent     g_log[LOG_SIZE];
static volatile unsigned int g_log_count; /* incremented by ISR after each entry */

/* ── Exception vector table ───────────────────────────────────────────────── */
/* 512 words = 2048 bytes, aligned to 2 KB (required by VBAR_EL1).
 * Populated at runtime in setup_vectors().                                    */
static unsigned int g_vt[512] __attribute__((aligned(2048)));

/* ── Pass / fail counters ─────────────────────────────────────────────────── */
static int g_pass;
static int g_fail;

/* ── Forward declarations ─────────────────────────────────────────────────── */
static void setup_vectors(void);
static void setup_gic(void);
static void enable_irq(void);
static int  wait_n(unsigned int target);
static int  find_event(unsigned int from, unsigned int irq_id,
                        unsigned int status_bit);
static unsigned int test_begin(void);
static void pass_test(const char *name);
static void fail_test(const char *name, const char *why);
static void put_char(char c);
static void put_str(const char *s);
static void put_hex(unsigned int v);
static void isr_main(void);

/* Individual test functions — each is self-contained and can run standalone. */
static void test1(void);
static void test2(void);
static void test3(void);
static void test4(void);
static void test5(void);

/* Naked functions — defined before helpers so they land first in .text.
 * irq_handler() is a normal C function called from irq_entry().            */
void __attribute__((naked)) _start(void);
void __attribute__((naked)) irq_entry(void);
static void irq_handler(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * _start — entry point at 0x80000000 (Cortex-A53 RVBAR reset vector)
 * ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((naked)) _start(void)
{
    __asm__ volatile(
        "ldr x0, =0x90000000\n"   /* stack top — 256 MB RAM ends at 0x90000000 */
        "mov sp, x0\n"
        "bl  isr_main\n"
        "1: b 1b\n"               /* halt if isr_main ever returns             */
    );
}

/* ═══════════════════════════════════════════════════════════════════════════
 * irq_entry — low-level IRQ exception entry stub (AArch64 EL1 SPx)
 *
 * This code lives at the VBAR_EL1 + 0x280 exception vector slot, installed
 * dynamically by setup_vectors() via a LDR x0, [pc, #8] ; BR x0 stub.
 *
 * The AArch64 CPU automatically masks D/A/I/F on exception entry.
 * ELR_EL1 and SPSR_EL1 are saved / restored so ERET returns cleanly.
 *
 * Caller-saved registers (x0-x18, x30) and the system registers are saved
 * on the EL1 stack (SP_EL1).  x19-x29 are callee-saved and not touched by
 * irq_handler(), so they need not be saved here.
 * ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((naked)) irq_entry(void)
{
    __asm__ volatile(
        /* ── save context ─────────────────────────────────────────────────── */
        "sub  sp,  sp,  #256\n"
        "stp  x0,  x1,  [sp, #0]\n"
        "stp  x2,  x3,  [sp, #16]\n"
        "stp  x4,  x5,  [sp, #32]\n"
        "stp  x6,  x7,  [sp, #48]\n"
        "stp  x8,  x9,  [sp, #64]\n"
        "stp  x10, x11, [sp, #80]\n"
        "stp  x12, x13, [sp, #96]\n"
        "stp  x14, x15, [sp, #112]\n"
        "stp  x16, x17, [sp, #128]\n"
        "stp  x18, x30, [sp, #144]\n"   /* x18 (platform), x30 (LR) */
        "mrs  x0,  elr_el1\n"
        "mrs  x1,  spsr_el1\n"
        "stp  x0,  x1,  [sp, #160]\n"
        /* ── call C handler ────────────────────────────────────────────────── */
        "bl   irq_handler\n"
        /* ── restore context ──────────────────────────────────────────────── */
        "ldp  x0,  x1,  [sp, #160]\n"
        "msr  elr_el1,  x0\n"
        "msr  spsr_el1, x1\n"
        "ldp  x0,  x1,  [sp, #0]\n"
        "ldp  x2,  x3,  [sp, #16]\n"
        "ldp  x4,  x5,  [sp, #32]\n"
        "ldp  x6,  x7,  [sp, #48]\n"
        "ldp  x8,  x9,  [sp, #64]\n"
        "ldp  x10, x11, [sp, #80]\n"
        "ldp  x12, x13, [sp, #96]\n"
        "ldp  x14, x15, [sp, #112]\n"
        "ldp  x16, x17, [sp, #128]\n"
        "ldp  x18, x30, [sp, #144]\n"
        "add  sp,  sp,  #256\n"
        "eret\n"
    );
}

/* ═══════════════════════════════════════════════════════════════════════════
 * irq_handler — C-level interrupt service routine
 *
 * Called from irq_entry() with full register context saved on stack.
 * Reads GICC_IAR (acknowledge), reads and W1C-clears USART STATUS, logs
 * one entry per set STATUS bit, then writes GICC_EOIR (end of interrupt).
 *
 * The USART2 irq output is a combined signal (OR of TBIR/TIR/RIR/EIR).
 * Because advance() processes all elapsed ticks lazily, a single TBUF
 * write may complete the entire TX frame, firing both TBIR and TIR; the
 * ISR captures both by iterating over all four STATUS bits.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void irq_handler(void)
{
    /* GICv3 CPU interface via system registers.
     * ICC_IAR1_EL1 (S3_0_C12_C12_0): acknowledge interrupt; returns INTID.
     * ICC_EOIR1_EL1 (S3_0_C12_C12_1): end-of-interrupt + deactivate.        */
    unsigned long iar_long;
    __asm__ volatile("mrs %0, S3_0_C12_C12_0" : "=r"(iar_long));
    unsigned int iar    = (unsigned int)iar_long;
    unsigned int id     = iar & 0xFFFFFFu;   /* INTID: 24 bits in GICv3       */
    unsigned int status = 0u;
    unsigned int i;

    static const unsigned int BITS[4] = {
        STATUS_TBIR, STATUS_TIR, STATUS_RIR, STATUS_EIR
    };

    if (id == IRQ_SPURIOUS) {
        __asm__ volatile("msr S3_0_C12_C12_1, %0" :: "r"((unsigned long)iar));
        return;
    }

    /* Read and atomically clear all STATUS bits from the triggering USART. */
    if (id == IRQ_USART_A) {
        status    = UA_STATUS;
        UA_STATUS = status;          /* W1C: clear everything we just read    */
    } else if (id == IRQ_USART_B) {
        status    = UB_STATUS;
        UB_STATUS = status;
    }

    /* Log one entry per set STATUS bit so tests can check each IRQ type
     * individually, even when multiple bits fire in a single ISR call.     */
    for (i = 0u; i < 4u; ++i) {
        if (status & BITS[i]) {
            unsigned int idx = g_log_count;
            if (idx < LOG_SIZE) {
                g_log[idx].irq_id     = id;
                g_log[idx].status_bit = BITS[i];
                /* DMB before incrementing count: ensures main thread sees the
                 * full record before it observes the new count.             */
                __asm__ volatile("dmb sy" ::: "memory");
                g_log_count = idx + 1u;
            }
        }
    }

    /* ICC_EOIR1_EL1: signal EOI and deactivate (EOImode=0, default). */
    __asm__ volatile("msr S3_0_C12_C12_1, %0" :: "r"((unsigned long)iar));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * test_begin — prepare for a new test
 *
 * Clears both STATUS registers and returns the current log count as the base
 * for find_event() searches.  Does NOT read RBUF — the Usart model blocks on
 * a read of an empty RBUF, so draining must only happen when data is known to
 * be present (test1/test2 read RBUF after confirming RIR; test3 drains at end
 * after the overrun; test4 drains within its loop).
 * ═══════════════════════════════════════════════════════════════════════════ */
static unsigned int test_begin(void)
{
    UA_STATUS = 0xFFFFFFFFu;    /* W1C: clear all sticky bits */
    UB_STATUS = 0xFFFFFFFFu;
    __asm__ volatile("" ::: "memory");
    return g_log_count;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 1: A → B  (0x55)
 * Expected: TBIR_A fires (USART_A INT), then RIR_B fires (USART_B INT).
 * TIR_A fires at T+frame_duration via tir_auto_method (SC_METHOD on m_sig_tir.pos()).
 * RIR_B fires at T+frame_duration via deferred m_rxd_done_ev.
 * Expected sequence: TBIR_A (T+0) → TIR_A (T+3.2µs) → RIR_B (T+3.2µs+δ).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test1(void)
{
    unsigned int base, data;

    put_str("Test 1: A sends 0x55, expect TBIR_A + TIR_A + RIR_B\r\n");
    base = test_begin();
    UA_TBUF = 0x55u;

    /* Wait for 3 events: TBIR_A (T+0), TIR_A (T+3.2µs), RIR_B (T+3.2µs+δ). */
    if (!wait_n(base + 3u)) {
        fail_test("T1", "IRQ timeout"); return;
    }
    if (find_event(base, IRQ_USART_A, STATUS_TBIR) < 0) {
        fail_test("T1-TBIR_A", "not in ISR log"); return;
    }
    put_str("  [ISR] TBIR_A confirmed\r\n");

    if (find_event(base, IRQ_USART_A, STATUS_TIR) < 0) {
        fail_test("T1-TIR_A", "not in ISR log"); return;
    }
    put_str("  [ISR] TIR_A  confirmed\r\n");

    if (find_event(base, IRQ_USART_B, STATUS_RIR) < 0) {
        fail_test("T1-RIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] RIR_B  confirmed\r\n");

    data = UB_RBUF & 0xFFu;
    put_str("  RBUF_B="); put_hex(data); put_str("\r\n");
    if (data == 0x55u) pass_test("Test1 A->B 0x55");
    else               fail_test("Test1", "RBUF_B != 0x55");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 2: B → A  (0xAA)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test2(void)
{
    unsigned int base, data;

    put_str("Test 2: B sends 0xAA, expect TBIR_B + TIR_B + RIR_A\r\n");
    base = test_begin();
    UB_TBUF = 0xAAu;

    if (!wait_n(base + 3u)) {
        fail_test("T2", "IRQ timeout"); return;
    }
    if (find_event(base, IRQ_USART_B, STATUS_TBIR) < 0) {
        fail_test("T2-TBIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] TBIR_B confirmed\r\n");

    if (find_event(base, IRQ_USART_B, STATUS_TIR) < 0) {
        fail_test("T2-TIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] TIR_B  confirmed\r\n");

    if (find_event(base, IRQ_USART_A, STATUS_RIR) < 0) {
        fail_test("T2-RIR_A", "not in ISR log"); return;
    }
    put_str("  [ISR] RIR_A  confirmed\r\n");

    data = UA_RBUF & 0xFFu;
    put_str("  RBUF_A="); put_hex(data); put_str("\r\n");
    if (data == 0xAAu) pass_test("Test2 B->A 0xAA");
    else               fail_test("Test2", "RBUF_A != 0xAA");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 3: Overrun / EIR
 *   Step 1: send 0x11 → fills RBUF_B (RIR_B fires)
 *   Step 2: send 0x22 while RBUF_B still full → overrun → EIR_B fires
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test3(void)
{
    unsigned int base;

    put_str("Test 3: Overrun — EIR_B after second byte\r\n");
    base = test_begin();
    UA_TBUF = 0x11u;

    /* Wait for 3 events: TBIR_A (T+0), TIR_A (T+3.2µs), RIR_B (T+3.2µs+δ). */
    if (!wait_n(base + 3u)) {
        fail_test("T3-RIR_B", "first byte timeout"); return;
    }
    if (find_event(base, IRQ_USART_B, STATUS_RIR) < 0) {
        fail_test("T3-RIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] RIR_B (RBUF_B full)\r\n");

    /* Do NOT drain RBUF_B — the overrun only fires if RBUF is still full. */
    base = g_log_count;
    UA_TBUF = 0x22u;

    /* Wait for 3 events: TBIR_A + TIR_A + EIR_B. */
    if (!wait_n(base + 3u)) {
        fail_test("T3-EIR_B", "EIR timeout"); return;
    }
    if (find_event(base, IRQ_USART_B, STATUS_EIR) < 0) {
        fail_test("T3-EIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] EIR_B  confirmed (overrun)\r\n");
    pass_test("Test3 Overrun EIR");
    /* Drain the 0x11 byte left in RBUF_B from step 1 so test4 starts clean. */
    (void)UB_RBUF;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 4: Multi-byte stream A → B  [0xDE, 0xAD, 0xBE, 0xEF]
 * Each TBUF write fires TBIR_A + RIR_B (and possibly TIR_A).
 * RBUF_B is drained after each byte so the next byte's RIR fires cleanly.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test4(void)
{
    static const unsigned int stream[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
    unsigned int base, i;
    int ok = 1;

    put_str("Test 4: Multi-byte A->B [0xDE, 0xAD, 0xBE, 0xEF]\r\n");
    (void)test_begin();     /* drain/clear; first per-byte base set in loop  */

    for (i = 0u; i < 4u; ++i) {
        unsigned int exp  = stream[i];
        unsigned int data;

        base = g_log_count;
        UA_TBUF = exp;

        /* 3 events per byte: TBIR_A (T+0), TIR_A (T+3.2µs), RIR_B (T+3.2µs+δ). */
        if (!wait_n(base + 3u)) {
            put_str("  timeout byte "); put_hex(exp); put_str("\r\n");
            ok = 0; break;
        }
        if (find_event(base, IRQ_USART_A, STATUS_TBIR) < 0) {
            put_str("  TBIR_A missing for byte "); put_hex(exp); put_str("\r\n");
            ok = 0; break;
        }
        if (find_event(base, IRQ_USART_A, STATUS_TIR) < 0) {
            put_str("  TIR_A  missing for byte "); put_hex(exp); put_str("\r\n");
            ok = 0; break;
        }
        if (find_event(base, IRQ_USART_B, STATUS_RIR) < 0) {
            put_str("  RIR_B  missing for byte "); put_hex(exp); put_str("\r\n");
            ok = 0; break;
        }

        data = UB_RBUF & 0xFFu;
        put_str("  [ISR] byte "); put_hex(i);
        put_str(" sent="); put_hex(exp);
        put_str(" got=");  put_hex(data);
        put_str((data == exp) ? " OK\r\n" : " MISMATCH\r\n");
        if (data != exp) { ok = 0; break; }
    }
    if (ok) pass_test("Test4 multi-byte");
    else    fail_test("Test4 multi-byte", "mismatch or timeout");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 5: TIR — TX-frame complete interrupt
 *
 * TIR is now delivered automatically via tir_auto_method (SC_METHOD on
 * m_sig_tir.pos()) at T+frame_duration, in the same delta epoch as RIR_B.
 * No UA_CON polling loop needed — just wait for 3 events: TBIR_A, TIR_A,
 * RIR_B.  This test exercises the same path as Tests 1-4 but is kept
 * separate to explicitly verify TIR standalone.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test5(void)
{
    unsigned int base;

    put_str("Test 5: TIR — TX-frame complete (ISR, auto-delivered)\r\n");
    base = test_begin();
    UA_TBUF = 0x5Au;

    /* Wait for TBIR_A + TIR_A + RIR_B (3 events). */
    if (!wait_n(base + 3u)) {
        fail_test("T5", "IRQ timeout"); return;
    }
    if (find_event(base, IRQ_USART_A, STATUS_TBIR) < 0) {
        fail_test("T5-TBIR_A", "not in ISR log"); return;
    }
    put_str("  [ISR] TBIR_A confirmed\r\n");

    if (find_event(base, IRQ_USART_A, STATUS_TIR) < 0) {
        fail_test("T5-TIR_A", "not in ISR log"); return;
    }
    put_str("  [ISR] TIR_A  confirmed\r\n");

    if (find_event(base, IRQ_USART_B, STATUS_RIR) < 0) {
        fail_test("T5-RIR_B", "not in ISR log"); return;
    }
    put_str("  [ISR] RIR_B  confirmed\r\n");

    pass_test("Test5 TIR TX-complete");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * isr_main — test driver, called from _start with a valid SP
 *
 * Runs whichever tests are enabled by TEST_MASK (bit N-1 = Test N).
 * Each test function is self-contained: test_begin() ensures clean state
 * regardless of what ran before.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void isr_main(void)
{
    put_str("\r\n");
    put_str("================================================\r\n");
    put_str("  USART2 ISR Interrupt Test (GICv3)\r\n");
    put_str("  A <-> B bridged via SerialBridge\r\n");
    put_str("  Interrupts via AArch64 EL1 exception vectors\r\n");
    put_str("  TEST_MASK="); put_hex(g_test_mask); put_str("\r\n");
    put_str("================================================\r\n\r\n");

    /* Install exception vectors, configure GIC, enable IRQs at EL1. */
    setup_vectors();

    UA_CON    = CON_INIT;
    UB_CON    = CON_INIT;
    UA_STATUS = 0xFFFFFFFFu;   /* W1C: clear any stale bits before arming   */
    UB_STATUS = 0xFFFFFFFFu;

    setup_gic();
    enable_irq();

    if (g_test_mask & 0x01u) { put_str("\r\n"); test1(); }
    if (g_test_mask & 0x02u) { put_str("\r\n"); test2(); }
    if (g_test_mask & 0x04u) { put_str("\r\n"); test3(); }
    if (g_test_mask & 0x08u) { put_str("\r\n"); test4(); }
    if (g_test_mask & 0x10u) { put_str("\r\n"); test5(); }

    put_str("\r\n================================================\r\n");
    put_str("  Passed: "); put_hex((unsigned int)g_pass);
    put_str("   Failed: "); put_hex((unsigned int)g_fail);
    put_str("\r\n");
    put_str(g_fail == 0 ? "  ALL TESTS PASSED\r\n" : "  SOME TESTS FAILED\r\n");
    put_str("================================================\r\n");

    EXITER = 0u;   /* triggers sc_stop() — simulation ends cleanly */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * setup_vectors — install the AArch64 exception vector table
 *
 * g_vt is a 2 KB-aligned array in BSS (zero-initialized by the ELF loader).
 * We fill every slot with B . (0x14000000 = branch to self) to catch
 * unexpected exceptions, then install the "Current EL, SPx, IRQ" handler
 * at offset 0x280 using a two-instruction indirect-branch stub:
 *
 *   slot+0: 0x58000040  LDR x0, [pc, #8]  ; load 64-bit fn pointer
 *   slot+1: 0xD61F0000  BR  x0             ; indirect branch
 *   slot+2: irq_entry[31:0]
 *   slot+3: irq_entry[63:32]
 *
 * After filling the table: clean D-cache (DC CIVAC) to push writes to
 * point of coherency, invalidate I-cache (IC IALLU), then set VBAR_EL1.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void setup_vectors(void)
{
    unsigned int        i;
    unsigned int        slot;
    unsigned long       fn;
    unsigned char      *p;

    /* Fill every 32-instruction slot with infinite-loop (B .) instructions. */
    for (i = 0u; i < 512u; ++i)
        g_vt[i] = 0x14000000u;   /* B . */

    /* Install IRQ handler at "Current EL with SPx, IRQ" offset 0x280.
     * LDR x0, [pc, #8]:  imm19 = 2 (8 bytes ahead), Rt = 0 → 0x58000040
     * BR  x0:             unconditional register branch       → 0xD61F0000  */
    slot      = 0x280u / 4u;                            /* word index = 160  */
    fn        = (unsigned long)(void *)irq_entry;
    g_vt[slot + 0] = 0x58000040u;                      /* LDR x0, [pc, #8]  */
    g_vt[slot + 1] = 0xD61F0000u;                      /* BR  x0            */
    g_vt[slot + 2] = (unsigned int)(fn & 0xFFFFFFFFu); /* fn low  32 bits   */
    g_vt[slot + 3] = (unsigned int)(fn >> 32);         /* fn high 32 bits   */

    /* Cache coherency: clean D-cache lines covering the vector table, then
     * invalidate I-cache so the CPU fetches the new instructions.           */
    p = (unsigned char *)g_vt;
    for (i = 0u; i < 2048u; i += 64u)
        __asm__ volatile("dc civac, %0" :: "r"(p + i) : "memory");

    __asm__ volatile(
        "dsb sy\n"      /* ensure DC CIVAC completions are globally visible  */
        "ic  iallu\n"   /* invalidate all I-cache lines (inner-shareable)    */
        "dsb sy\n"      /* wait for IC IALLU to complete                     */
        "isb\n"         /* flush instruction pipeline                        */
        ::: "memory"
    );

    /* Point VBAR_EL1 at the newly populated vector table. */
    {
        unsigned long vbar = (unsigned long)(void *)g_vt;
        __asm__ volatile("msr vbar_el1, %0\nisb\n" :: "r"(vbar));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * setup_gic — program GICv3 for USART interrupts
 *
 * GICv3 differs from GICv2 in three areas:
 *
 *  a) CPU interface: uses AArch64 system registers (ICC_*_EL1) instead of
 *     MMIO.  ICC_SRE_EL1 must have SRE=1; without EL3/EL2, QEMU starts with
 *     it set, but we write it explicitly.  ICC_IGRPEN1_EL1 enables Group-1
 *     (non-secure) interrupt delivery (AArch64 IRQ vector).
 *
 *  b) SPI routing: GICD_IROUTER(n) (64-bit) routes each SPI to a specific
 *     CPU by affinity rather than the GICv2 ITARGETSR byte masks.  We route
 *     IRQ 32 and IRQ 33 to CPU 0 (affinity 0.0.0.0, direct mode).
 *
 *  c) Redistributor (GICR): the GICR_WAKER register's ProcessorSleep bit
 *     must be cleared so the redistributor accepts interrupts.  We then
 *     wait for ChildrenAsleep to de-assert before proceeding.
 *
 *  d) GICD_CTLR: GICv3 uses ARE_NS (bit 4) for affinity routing and
 *     EnableGrp1NS (bit 1) instead of GICv2's simple bit-0 enable.
 *     Value 0x12 = ARE_NS(4) | EnableGrp1NS(1).
 *
 * Edge-triggered (GICD_ICFGR2 = 0x0A) is still critical: the USART irq
 * pulse lasts only 2 clock periods.  Level-sensitive would cause the GIC to
 * re-assert after every EOI while the pulse is high.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void setup_gic(void)
{
    /* ── a) Enable GICv3 system-register CPU interface (ICC_SRE_EL1) ─────── */
    /* SRE=1(bit0), DFB=1(bit1), DIB=1(bit2) — disable legacy FIQ/IRQ bypass */
    __asm__ volatile("msr S3_0_C12_C12_5, %0\nisb\n" :: "r"(7UL));

    /* ── c) Wake redistributor ───────────────────────────────────────────── */
    /* Clear ProcessorSleep (bit 1); wait for ChildrenAsleep (bit 2) to drop.*/
    {
        unsigned int w = GICR_WAKER;
        w &= ~(1u << 1);
        GICR_WAKER = w;
        while (GICR_WAKER & (1u << 2)) { /* spin */ }
    }

    /* ── GICD: disable before reconfiguring ─────────────────────────────── */
    GICD_CTLR = 0u;
    __asm__ volatile("dsb sy\n" ::: "memory");

    /* ── Group 1 for IRQ 32-63: required for IRQ (not FIQ) delivery ────── */
    /* GICv3 default group is 0 (→ FIQ signal).  Set GICD_IGROUPR1 = all-1
     * so all SPIs 32-63 are Group 1 (→ IRQ signal, handled by our ISR).    */
    GICD_IGROUPR1 = 0xFFFFFFFFu;

    /* ── Edge-triggered for IRQ 32 and IRQ 33 ───────────────────────────── */
    /* GICD_ICFGR2 (IRQ 32-47): 2-bit field, bit[1]=1 → edge.
     * IRQ32 bits[1:0]=0b10; IRQ33 bits[3:2]=0b10 → 0x0A.                   */
    GICD_ICFGR2 = 0x0000000Au;

    /* ── Priority 0xA0 for IRQ 32-35 (one byte per IRQ) ─────────────────── */
    GICD_IPRIORITYR8 = 0xA0A0A0A0u;

    /* ── b) Route SPIs to CPU 0 (affinity 0.0.0.0, direct mode) ────────── */
    /* GICD_IROUTER(n) = 0: Aff0/1/2/3 all 0, IRM (bit31) = 0 (direct).    */
    GICD_IROUTER32 = 0x0ULL;
    GICD_IROUTER33 = 0x0ULL;

    /* ── Enable SPI 0 and SPI 1 (bits 0 and 1 of ISENABLER[1]) ─────────── */
    GICD_ISENABLER1 = 0x00000003u;

    /* ── d) Enable distributor: ARE_NS(bit4) + EnableGrp1NS(bit1) ───────── */
    GICD_CTLR = 0x00000012u;
    __asm__ volatile("dsb sy\n" ::: "memory");

    /* ── CPU interface: set priority mask, then enable Group-1 IRQs ─────── */
    /* ICC_PMR_EL1 (S3_0_C4_C6_0): 0xFF = allow all priorities.             */
    __asm__ volatile("msr S3_0_C4_C6_0, %0\n" :: "r"(0xFFUL));
    /* ICC_IGRPEN1_EL1 (S3_0_C12_C12_7): bit 0 = enable Group-1.            */
    __asm__ volatile("msr S3_0_C12_C12_7, %0\nisb\n" :: "r"(1UL));

    __asm__ volatile("dsb sy\nisb\n" ::: "memory");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * enable_irq — unmask IRQ at EL1 by clearing PSTATE.I (the I bit in DAIF)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void enable_irq(void)
{
    __asm__ volatile("msr daifclr, #2\nisb\n" ::: "memory");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * wait_n — spin until at least `target` ISR events have been logged.
 *
 * Returns 1 on success, 0 on timeout.
 * The memory clobber prevents the compiler from hoisting g_log_count reads
 * out of the loop.  Because we are in multithread-unconstrained mode, QEMU
 * and SystemC run on separate OS threads; the ISR fires between QEMU
 * instructions and increments g_log_count without any scheduler cooperation.
 *
 * Each iteration executes WFI (Wait For Interrupt) so the vCPU idles until
 * the GIC delivers the next IRQ, at which point the ISR runs and increments
 * g_log_count before returning here.  This is reliable regardless of timing:
 * the CPU genuinely sleeps until the interrupt arrives rather than racing a
 * tight spin against the SystemC IRQ-delivery pipeline.
 *
 * The limit of 40 WFI iterations is a generous timeout (40 distinct interrupt
 * events) — in practice tests need only 1-2 events each.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int wait_n(unsigned int target)
{
    unsigned int i;
    for (i = 0u; i < 40u; ++i) {
        if (g_log_count >= target) return 1;
        __asm__ volatile("wfi" ::: "memory");
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * find_event — scan g_log[from..g_log_count-1] for a matching entry.
 *
 * Returns the log index on success, -1 if not found.
 * ═══════════════════════════════════════════════════════════════════════════ */
static int find_event(unsigned int from,
                       unsigned int irq_id,
                       unsigned int status_bit)
{
    unsigned int i;
    unsigned int count = g_log_count;   /* snapshot; ISR may increment later */
    for (i = from; i < count; ++i) {
        if (g_log[i].irq_id == irq_id &&
            g_log[i].status_bit == status_bit)
            return (int)i;
    }
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Helper functions — defined after _start / irq_entry so they do not
 * precede the entry point in the .text section.
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
    static const char h[] = "0123456789ABCDEF";
    put_char('0'); put_char('x');
    put_char(h[(v >> 28) & 0xFu]);
    put_char(h[(v >> 24) & 0xFu]);
    put_char(h[(v >> 20) & 0xFu]);
    put_char(h[(v >> 16) & 0xFu]);
    put_char(h[(v >> 12) & 0xFu]);
    put_char(h[(v >>  8) & 0xFu]);
    put_char(h[(v >>  4) & 0xFu]);
    put_char(h[(v >>  0) & 0xFu]);
}

static void pass_test(const char *name)
{
    put_str("[PASS] "); put_str(name); put_str("\r\n");
    ++g_pass;
}

static void fail_test(const char *name, const char *why)
{
    put_str("[FAIL] "); put_str(name);
    put_str(": ");      put_str(why);
    put_str("\r\n");
    ++g_fail;
}
