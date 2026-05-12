/* hello_i2c_isr.c — AArch64 I2C interrupt stress test
 *
 * Platform: conf_i2c_isr.lua  (Cortex-A53 EL1, GICv3, I2cStm32 peripheral)
 *
 * GIC wiring:
 *   i2c_stm32_0.ev_irq → GIC SPI 2 → IRQ ID 34
 *   i2c_stm32_0.er_irq → GIC SPI 3 → IRQ ID 35
 *
 * I2C register map (base 0x09005000):
 *   +0x00 CR1   +0x04 CR2   +0x10 DR   +0x14 SR1   +0x18 SR2
 *
 * SR1 bits: SB(0) ADDR(1) BTF(2) STOPF(4) RxNE(6) TxE(7) AF(10)
 *
 * Tests:
 *   1. TX  1 byte  : SB+ADDR+TxE            = 3 EV interrupts
 *   2. TX  8 bytes : SB+ADDR+8xTxE          = 10 EV interrupts
 *   3. RX  4 bytes : SB+ADDR+4xRxNE         = 6 EV interrupts (ADDR clears in ISR, 1st RxNE on SR2 read)
 *   4. Bad address : SB+AF                  = 1 EV + 1 ER interrupt
 *   5. TX 32 bytes : SB+ADDR+32xTxE         = 34 EV interrupts (stress test)
 *   6. Repeat start: TX2+RX2 without STOP   = (SB+ADDR+2xTxE)+(SB+ADDR+2xRxNE) = 8 EV interrupts
 *   7. BTF on TX completion
 *   8. ITBUFEN masks TxE/RxNE interrupt delivery but not flags
 *   9. STOP and SWRST register/state behavior
 *  10. AF error flag clears by SR1 write-0
 *
 * Total interrupts across all tests: ~63
 *
 * Sync mechanism under test: frame-level sc_time scheduling (I2cStm32 peripheral)
 */

/* ── MMIO helpers ─────────────────────────────────────────────────────────── */
#define MMIO32(a)  (*(volatile unsigned int  *)(unsigned long)(a))
#define MMIO64(a)  (*(volatile unsigned long long *)(unsigned long)(a))

/* ── Console USART (usart2_c, no interrupts — direct TBUF writes) ─────────── */
#define UC_TBUF  MMIO32(0x09004004UL)

/* ── I2C registers ─────────────────────────────────────────────────────────── */
#define I2C_BASE   0x09005000UL
#define I2C_CR1    MMIO32(I2C_BASE + 0x00UL)
#define I2C_CR2    MMIO32(I2C_BASE + 0x04UL)
#define I2C_OAR1   MMIO32(I2C_BASE + 0x08UL)
#define I2C_DR     MMIO32(I2C_BASE + 0x10UL)
#define I2C_SR1    MMIO32(I2C_BASE + 0x14UL)
#define I2C_SR2    MMIO32(I2C_BASE + 0x18UL)
#define I2C_CCR    MMIO32(I2C_BASE + 0x1CUL)
#define I2C_TRISE  MMIO32(I2C_BASE + 0x20UL)

#define CR1_PE     (1u << 0)
#define CR1_START  (1u << 8)
#define CR1_STOP   (1u << 9)
#define CR1_ACK    (1u << 10)
#define CR1_SWRST  (1u << 15)
#define CR2_ITEVTEN (1u << 9)
#define CR2_ITBUFEN (1u << 10)
#define CR2_ITERREN (1u << 8)
#define SR1_SB     (1u << 0)
#define SR1_ADDR   (1u << 1)
#define SR1_BTF    (1u << 2)
#define SR1_RxNE   (1u << 6)
#define SR1_TxE    (1u << 7)
#define SR1_BERR   (1u << 8)
#define SR1_ARLO   (1u << 9)
#define SR1_AF     (1u << 10)
#define SR1_OVR    (1u << 11)
#define SR1_PECERR (1u << 12)
#define SR1_TIMEOUT (1u << 14)
#define SR1_SMBALERT (1u << 15)

#define SR2_MSL    (1u << 0)
#define SR2_BUSY   (1u << 1)
#define SR2_TRA    (1u << 2)

/* ── Exiter ───────────────────────────────────────────────────────────────── */
#define EXITER  MMIO32(0x09010000UL)

/* ── GICv3 register addresses ─────────────────────────────────────────────── */
#define GICD_BASE  0x08000000UL
#define GICR_BASE  0x080A0000UL

#define GICD_CTLR        MMIO32(GICD_BASE + 0x000U)
#define GICD_IGROUPR1    MMIO32(GICD_BASE + 0x084U)  /* group bits IRQ 32-63 */
#define GICD_ISENABLER1  MMIO32(GICD_BASE + 0x104U)  /* enable bits IRQ 32-63 */
#define GICD_ICFGR2      MMIO32(GICD_BASE + 0xC08U)  /* edge/level IRQ 32-47 */
#define GICD_IPRIORITYR8 MMIO32(GICD_BASE + 0x420U)  /* priority bytes IRQ 32-35 */
/* IROUTER for SPIs: GICD_BASE + 0x6000 + intid*8 */
#define GICD_IROUTER32  MMIO64(GICD_BASE + 0x6100UL)
#define GICD_IROUTER33  MMIO64(GICD_BASE + 0x6108UL)
#define GICD_IROUTER34  MMIO64(GICD_BASE + 0x6110UL)
#define GICD_IROUTER35  MMIO64(GICD_BASE + 0x6118UL)

#define GICR_WAKER  MMIO32(GICR_BASE + 0x0014U)

#define IRQ_I2C_EV  34u
#define IRQ_I2C_ER  35u
#define IRQ_SPURIOUS 1023u

#define SLAVE_ADDR 0x50u

/* ── ISR log ──────────────────────────────────────────────────────────────── */
typedef struct {
    unsigned int irq_id;  /* 34 = EV, 35 = ER */
    unsigned int sr1;     /* SR1 snapshot (before ADDR clear) */
} I2cIrqEvent;

#define LOG_SIZE 192u

static volatile I2cIrqEvent  g_log[LOG_SIZE];
static volatile unsigned int g_log_count;

/* ── Pass / fail counters ─────────────────────────────────────────────────── */
static int g_pass;
static int g_fail;

/* ── Exception vector table ───────────────────────────────────────────────── */
static unsigned int g_vt[512] __attribute__((aligned(2048)));

/* ── Forward declarations ─────────────────────────────────────────────────── */
static void setup_vectors(void);
static void setup_gic(void);
static void enable_irq(void);
static int  i2c_wait_n(unsigned int target);
static int  i2c_wait_sr1(unsigned int mask);
static int  i2c_find(unsigned int from, unsigned int irq_id, unsigned int sr1_mask);
static void i2c_init(void);
static void i2c_recover(void);
static void pass_test(const char *name);
static void fail_test(const char *name, const char *why);
static void put_char(char c);
static void put_str(const char *s);
static void put_hex(unsigned int v);
static void put_dec(unsigned int v);
static void i2c_main(void);

static void test1(void);
static void test2(void);
static void test3(void);
static void test4(void);
static void test5(void);
static void test6(void);
static void test7(void);
static void test8(void);
static void test9(void);
static void test10(void);

void __attribute__((naked)) _start(void);
void __attribute__((naked)) irq_entry(void);
static void irq_handler(void);

/* ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((naked)) _start(void)
{
    __asm__ volatile(
        "ldr x0, =0x90000000\n"
        "mov sp, x0\n"
        "bl  i2c_main\n"
        "1: b 1b\n"
    );
}

/* ═══════════════════════════════════════════════════════════════════════════
 * IRQ entry stub — installed at VBAR_EL1 + 0x280 (EL1 SPx IRQ)
 * ═══════════════════════════════════════════════════════════════════════════ */
void __attribute__((naked)) irq_entry(void)
{
    __asm__ volatile(
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
        "stp  x18, x30, [sp, #144]\n"
        "mrs  x0,  elr_el1\n"
        "mrs  x1,  spsr_el1\n"
        "stp  x0,  x1,  [sp, #160]\n"
        "bl   irq_handler\n"
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
 * irq_handler — handles EV (34) and ER (35) I2C interrupts
 * ═══════════════════════════════════════════════════════════════════════════ */
static void irq_handler(void)
{
    unsigned long iar_long;
    __asm__ volatile("mrs %0, S3_0_C12_C12_0" : "=r"(iar_long));
    unsigned int iar = (unsigned int)iar_long;
    unsigned int id  = iar & 0xFFFFFFu;

    if (id == IRQ_SPURIOUS) {
        __asm__ volatile("msr S3_0_C12_C12_1, %0" :: "r"((unsigned long)iar));
        return;
    }

    if (id == IRQ_I2C_EV) {
        unsigned int sr1 = I2C_SR1;
        /* ADDR clear: read SR2 — the peripheral updates m_dr_rx for first RxNE */
        if (sr1 & SR1_ADDR) { (void)I2C_SR2; }
        unsigned int idx = g_log_count;
        if (idx < LOG_SIZE) {
            g_log[idx].irq_id = id;
            g_log[idx].sr1    = sr1;
            __asm__ volatile("dmb sy" ::: "memory");
            g_log_count = idx + 1u;
        }
    } else if (id == IRQ_I2C_ER) {
        unsigned int sr1 = I2C_SR1;
        I2C_SR1 = 0u;  /* clear error bits */
        /* Generate STOP to recover bus */
        I2C_CR1 |= CR1_STOP;
        unsigned int idx = g_log_count;
        if (idx < LOG_SIZE) {
            g_log[idx].irq_id = id;
            g_log[idx].sr1    = sr1;
            __asm__ volatile("dmb sy" ::: "memory");
            g_log_count = idx + 1u;
        }
    }

    __asm__ volatile("msr S3_0_C12_C12_1, %0" :: "r"((unsigned long)iar));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GICv3 setup
 * ═══════════════════════════════════════════════════════════════════════════ */
static void setup_vectors(void)
{
    unsigned int i;
    unsigned int *vt = g_vt;

    /* Fill all 512 slots (32 vectors × 16 instructions) with infinite loops */
    for (i = 0u; i < 512u; ++i)
        vt[i] = 0x14000000u;  /* B . */

    /* Install irq_entry at slot 0x280 / 4 = 160 (EL1 SPx IRQ offset) */
    unsigned long irq_addr  = (unsigned long)irq_entry;
    unsigned int  slot      = 0x280u / 4u;
    vt[slot]     = 0x58000040u;  /* LDR x0, [pc, #8]  */
    vt[slot + 1] = 0xD61F0000u;  /* BR  x0            */
    vt[slot + 2] = (unsigned int)(irq_addr & 0xFFFFFFFFu);
    vt[slot + 3] = (unsigned int)(irq_addr >> 32u);

    __asm__ volatile("msr vbar_el1, %0" :: "r"((unsigned long)vt));
    __asm__ volatile("isb");
}

static void setup_gic(void)
{
    /* ── Wake GICR (redistributor) ─────────────────────────────────────── */
    GICR_WAKER &= ~(1u << 1);  /* clear ProcessorSleep */
    while (GICR_WAKER & (1u << 2)) {}  /* wait ChildrenAsleep=0 */

    /* ── GICD: enable ARE, affinity routing ────────────────────────────── */
    GICD_CTLR = (1u << 4) | (1u << 1) | (1u << 0);  /* ARE_NS | EnableGrp1NS | EnableGrp1 */

    /* ── Assign all 4 SPIs (IRQ 32-35) to Group 1 (Non-Secure = normal IRQ) */
    GICD_IGROUPR1 = 0xFFFFFFFFu;

    /* ── All 4 at same priority (0x80) ─────────────────────────────────── */
    GICD_IPRIORITYR8 = 0x80808080u;

    /* ── Edge-triggered (ICFGR2 covers IRQ 32-47, 2 bits per IRQ) ─────── */
    /* bits [1:0]=IRQ32, [3:2]=IRQ33, [5:4]=IRQ34, [7:6]=IRQ35; b10=edge */
    GICD_ICFGR2 = 0xAAAAAAAAu;

    /* ── Route all 4 to CPU 0 (affinity 0.0.0.0, IRM=0) ──────────────── */
    GICD_IROUTER32 = 0ULL;
    GICD_IROUTER33 = 0ULL;
    GICD_IROUTER34 = 0ULL;
    GICD_IROUTER35 = 0ULL;

    /* ── Enable SPIs 0-3 (bits 0-3 of ISENABLER1 = IRQ 32-35) ─────────── */
    GICD_ISENABLER1 = 0xFu;

    /* ── CPU interface via system registers ─────────────────────────────── */
    __asm__ volatile("msr S3_0_C12_C12_5, %0" :: "r"(1UL));  /* ICC_SRE_EL1 = 1 */
    __asm__ volatile("isb");
    __asm__ volatile("msr S3_0_C4_C6_0,  %0" :: "r"(0xFFUL)); /* ICC_PMR_EL1 */
    __asm__ volatile("msr S3_0_C12_C12_7,%0" :: "r"(1UL));   /* ICC_IGRPEN1_EL1 = 1 */
    __asm__ volatile("isb");
}

static void enable_irq(void)
{
    __asm__ volatile("msr daifclr, #2");  /* clear I bit → enable IRQ */
    __asm__ volatile("isb");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */
static int i2c_wait_n(unsigned int target)
{
    unsigned int timeout = 20000000u;
    while (g_log_count < target && --timeout > 0u) {
        __asm__ volatile("wfi");
        __asm__ volatile("" ::: "memory");
    }
    return (int)(g_log_count >= target);
}

static int i2c_wait_sr1(unsigned int mask)
{
    unsigned int timeout = 20000000u;
    while (--timeout > 0u) {
        if ((I2C_SR1 & mask) == mask) return 1;
        __asm__ volatile("" ::: "memory");
    }
    return 0;
}

/* Find a log entry from index 'from' matching irq_id and sr1_mask */
static int i2c_find(unsigned int from, unsigned int irq_id, unsigned int sr1_mask)
{
    unsigned int i;
    for (i = from; i < g_log_count; ++i)
        if (g_log[i].irq_id == irq_id && (g_log[i].sr1 & sr1_mask))
            return (int)i;
    return -1;
}

static void i2c_init(void)
{
    I2C_CR1 = CR1_SWRST;       /* software reset */
    I2C_CR1 = 0u;               /* release reset */
    I2C_CR2 = 36u;              /* FREQ = 36 (APB MHz, not used for timing in TLM) */
    I2C_CCR = 4000u;            /* slow frame-level bus for deterministic ISR tests */
    I2C_TRISE = 37u;
    I2C_CR1 = CR1_PE;           /* enable */
    I2C_CR2 |= CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
}

static void i2c_recover(void)
{
    I2C_CR1 |= CR1_STOP;
    i2c_init();
}

static void pass_test(const char *name)
{
    ++g_pass;
    put_str("[PASS] "); put_str(name); put_str("\r\n");
}

static void fail_test(const char *name, const char *why)
{
    ++g_fail;
    put_str("[FAIL] "); put_str(name); put_str(" — "); put_str(why); put_str("\r\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 1: TX 1 byte → SB + ADDR + TxE  (3 EV interrupts)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test1(void)
{
    unsigned int base = g_log_count;
    put_str("Test 1: TX 1 byte, expect SB+ADDR+TxE\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T1", "SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 0u;  /* write mode */
    if (!i2c_wait_n(base + 2u)) { fail_test("T1", "ADDR timeout"); i2c_recover(); return; }

    I2C_DR = 0xABu;
    if (!i2c_wait_n(base + 3u)) { fail_test("T1", "TxE timeout"); i2c_recover(); return; }

    I2C_CR1 |= CR1_STOP;

    if (i2c_find(base, IRQ_I2C_EV, SR1_SB)   < 0) { fail_test("T1", "no SB");   return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_ADDR) < 0) { fail_test("T1", "no ADDR"); return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_TxE)  < 0) { fail_test("T1", "no TxE");  return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_BTF)  < 0) { fail_test("T1", "no BTF");  return; }

    put_str("  SB+ADDR+TxE confirmed\r\n");
    pass_test("Test1 TX-1-byte");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 2: TX 8 bytes → SB + ADDR + 8×TxE  (10 EV interrupts)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test2(void)
{
    static const unsigned char data[8] = {0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17};
    unsigned int base = g_log_count;
    unsigned int i;
    put_str("Test 2: TX 8 bytes, expect SB+ADDR+8xTxE\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T2", "SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 0u;
    if (!i2c_wait_n(base + 2u)) { fail_test("T2", "ADDR timeout"); i2c_recover(); return; }

    for (i = 0u; i < 8u; ++i) {
        I2C_DR = data[i];
        if (!i2c_wait_n(base + 3u + i)) {
            put_str("  TxE timeout at byte "); put_dec(i); put_str("\r\n");
            fail_test("T2", "TxE timeout"); i2c_recover(); return;
        }
    }
    I2C_CR1 |= CR1_STOP;

    if (i2c_find(base, IRQ_I2C_EV, SR1_SB)   < 0) { fail_test("T2", "no SB");   return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_ADDR) < 0) { fail_test("T2", "no ADDR"); return; }
    for (i = 0u; i < 8u; ++i)
        if (i2c_find(base + 2u + i, IRQ_I2C_EV, SR1_TxE) < 0) {
            fail_test("T2", "TxE missing"); return;
        }
    for (i = 0u; i < 8u; ++i)
        if (i2c_find(base + 2u + i, IRQ_I2C_EV, SR1_BTF) < 0) {
            fail_test("T2", "BTF missing"); return;
        }

    put_str("  SB+ADDR+8xTxE confirmed\r\n");
    pass_test("Test2 TX-8-bytes");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 3: RX 4 bytes → SB + ADDR + 4×RxNE  (6 EV interrupts)
 *
 * ISR reads SR2 on ADDR (which triggers first RxNE in the peripheral);
 * each DR read triggers the next RxNE.  Firmware reads DR after each RxNE.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test3(void)
{
    unsigned int base = g_log_count;
    unsigned int i;
    unsigned int bytes[4];
    put_str("Test 3: RX 4 bytes, expect SB+ADDR+4xRxNE\r\n");

    I2C_CR1 |= CR1_ACK | CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T3", "SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 1u;  /* read mode */
    /* ADDR interrupt: ISR reads SR2 → peripheral loads first byte → first RxNE fires */
    if (!i2c_wait_n(base + 2u)) { fail_test("T3", "ADDR timeout"); i2c_recover(); return; }

    /* Wait for first RxNE (loaded by peripheral on SR2 read in ISR) */
    if (!i2c_wait_n(base + 3u)) { fail_test("T3", "RxNE[0] timeout"); i2c_recover(); return; }

    /* Read 4 bytes; each DR read triggers next RxNE */
    for (i = 0u; i < 4u; ++i) {
        bytes[i] = I2C_DR & 0xFFu;
        if (i < 3u) {
            /* Each DR read fires another RxNE */
            if (!i2c_wait_n(base + 4u + i)) {
                put_str("  RxNE timeout byte "); put_dec(i+1u); put_str("\r\n");
                fail_test("T3", "RxNE timeout"); i2c_recover(); return;
            }
        }
    }
    I2C_CR1 = (I2C_CR1 & ~CR1_ACK) | CR1_STOP;

    /* Verify slave data: slave_mem[i] = 0xA0+i (wrapping) */
    for (i = 0u; i < 4u; ++i) {
        unsigned int expected = (0xA0u + i) & 0xFFu;
        if (bytes[i] != expected) {
            put_str("  byte "); put_dec(i);
            put_str(" got 0x"); put_hex(bytes[i]);
            put_str(" expected 0x"); put_hex(expected); put_str("\r\n");
            fail_test("T3", "wrong data"); return;
        }
    }

    if (i2c_find(base, IRQ_I2C_EV, SR1_SB)   < 0) { fail_test("T3", "no SB");   return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_ADDR) < 0) { fail_test("T3", "no ADDR"); return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_RxNE) < 0) { fail_test("T3", "no RxNE"); return; }

    put_str("  SB+ADDR+4xRxNE confirmed, data correct\r\n");
    pass_test("Test3 RX-4-bytes");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 4: Bad address (0x7E) → SB (EV) + AF (ER)  (2 interrupts)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test4(void)
{
    unsigned int base = g_log_count;
    put_str("Test 4: Bad address 0x7E, expect SB+AF\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T4", "SB timeout"); i2c_recover(); return; }

    I2C_DR = (0x7Eu << 1u) | 0u;  /* bad slave address */
    if (!i2c_wait_n(base + 2u)) { fail_test("T4", "AF timeout"); i2c_recover(); return; }

    /* ISR already set STOP via error handler */

    if (i2c_find(base,     IRQ_I2C_EV, SR1_SB) < 0) { fail_test("T4", "no SB"); return; }
    if (i2c_find(base + 1u, IRQ_I2C_ER, SR1_AF) < 0) { fail_test("T4", "no AF"); return; }

    put_str("  SB+AF confirmed\r\n");
    pass_test("Test4 bad-addr-AF");
    i2c_init();  /* re-init after error recovery */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 5: STRESS — TX 32 bytes → SB + ADDR + 32×TxE  (34 EV interrupts)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test5(void)
{
    unsigned int base = g_log_count;
    unsigned int i;
    put_str("Test 5: STRESS TX 32 bytes, expect 34 EV interrupts\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T5", "SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 0u;
    if (!i2c_wait_n(base + 2u)) { fail_test("T5", "ADDR timeout"); i2c_recover(); return; }

    for (i = 0u; i < 32u; ++i) {
        I2C_DR = (unsigned int)i;
        if (!i2c_wait_n(base + 3u + i)) {
            put_str("  TxE timeout byte "); put_dec(i); put_str("\r\n");
            fail_test("T5", "TxE timeout"); i2c_recover(); return;
        }
    }
    I2C_CR1 |= CR1_STOP;

    unsigned int ev_count = g_log_count - base;
    put_str("  Total EV interrupts: "); put_dec(ev_count); put_str("\r\n");

    if (ev_count < 34u) { fail_test("T5", "too few interrupts"); return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_SB)   < 0) { fail_test("T5", "no SB");   return; }
    if (i2c_find(base, IRQ_I2C_EV, SR1_ADDR) < 0) { fail_test("T5", "no ADDR"); return; }

    pass_test("Test5 STRESS-TX-32");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 6: Repeated START — TX 2 bytes then RX 2 bytes (8 EV interrupts)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test6(void)
{
    unsigned int base = g_log_count;
    unsigned int b0, b1;
    put_str("Test 6: Repeated START TX2+RX2, expect 8 EV interrupts\r\n");

    /* ── TX phase ─────────────────────────────────────────────────────── */
    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T6", "TX SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 0u;
    if (!i2c_wait_n(base + 2u)) { fail_test("T6", "TX ADDR timeout"); i2c_recover(); return; }

    I2C_DR = 0xCAu;
    if (!i2c_wait_n(base + 3u)) { fail_test("T6", "TX TxE[0] timeout"); i2c_recover(); return; }

    I2C_DR = 0xFEu;
    if (!i2c_wait_n(base + 4u)) { fail_test("T6", "TX TxE[1] timeout"); i2c_recover(); return; }

    /* ── Repeated START (no STOP between TX and RX) ────────────────────── */
    I2C_CR1 |= CR1_ACK | CR1_START;
    if (!i2c_wait_n(base + 5u)) { fail_test("T6", "RS SB timeout"); i2c_recover(); return; }

    I2C_DR = (SLAVE_ADDR << 1u) | 1u;  /* read mode */
    if (!i2c_wait_n(base + 6u)) { fail_test("T6", "RX ADDR timeout"); i2c_recover(); return; }

    /* First RxNE loaded by peripheral on SR2 read inside ISR */
    if (!i2c_wait_n(base + 7u)) { fail_test("T6", "RxNE[0] timeout"); i2c_recover(); return; }

    b0 = I2C_DR & 0xFFu;
    if (!i2c_wait_n(base + 8u)) { fail_test("T6", "RxNE[1] timeout"); i2c_recover(); return; }

    b1 = I2C_DR & 0xFFu;
    I2C_CR1 = (I2C_CR1 & ~CR1_ACK) | CR1_STOP;

    unsigned int ev_count = g_log_count - base;
    put_str("  RX bytes: 0x"); put_hex(b0); put_str(" 0x"); put_hex(b1);
    put_str(", total IRQs: "); put_dec(ev_count); put_str("\r\n");

    if (ev_count < 8u) { fail_test("T6", "too few interrupts"); return; }
    pass_test("Test6 Repeated-START");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 7: TX byte completion sets both TxE and BTF.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test7(void)
{
    unsigned int base = g_log_count;
    put_str("Test 7: TX completion sets TxE+BTF\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T7", "SB timeout"); i2c_recover(); return; }
    I2C_DR = (SLAVE_ADDR << 1u) | 0u;
    if (!i2c_wait_n(base + 2u)) { fail_test("T7", "ADDR timeout"); i2c_recover(); return; }
    I2C_DR = 0x5Au;
    if (!i2c_wait_n(base + 3u)) { fail_test("T7", "TxE/BTF timeout"); i2c_recover(); return; }
    I2C_CR1 |= CR1_STOP;

    if (i2c_find(base + 2u, IRQ_I2C_EV, SR1_TxE | SR1_BTF) < 0) {
        fail_test("T7", "missing combined TxE+BTF"); return;
    }
    pass_test("Test7 TX-BTF");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 8: ITBUFEN masks buffer interrupts while flags still update.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test8(void)
{
    unsigned int base = g_log_count;
    put_str("Test 8: ITBUFEN masks RxNE interrupt but RxNE flag still sets\r\n");

    I2C_CR2 = 36u | CR2_ITEVTEN | CR2_ITERREN;  /* ITBUFEN intentionally off */
    I2C_CR1 |= CR1_ACK | CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T8", "SB timeout"); i2c_recover(); return; }
    I2C_DR = (SLAVE_ADDR << 1u) | 1u;
    if (!i2c_wait_n(base + 2u)) { fail_test("T8", "ADDR timeout"); i2c_recover(); return; }
    if (!i2c_wait_sr1(SR1_RxNE)) { fail_test("T8", "RxNE flag timeout"); i2c_recover(); return; }
    if (g_log_count != base + 2u) { fail_test("T8", "RxNE IRQ was not masked"); i2c_recover(); return; }
    I2C_CR1 |= CR1_STOP;
    i2c_init();
    pass_test("Test8 ITBUFEN-mask");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 9: STOP clears bus state; SWRST clears user-visible registers.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test9(void)
{
    unsigned int base = g_log_count;
    unsigned int sr2;
    put_str("Test 9: STOP/SWRST state behavior\r\n");

    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T9", "SB timeout"); i2c_recover(); return; }
    I2C_CR1 |= CR1_STOP;
    sr2 = I2C_SR2;
    if (sr2 & (SR2_MSL | SR2_BUSY | SR2_TRA)) { fail_test("T9", "STOP left bus busy"); i2c_recover(); return; }
    if (I2C_CR1 & (CR1_START | CR1_STOP)) { fail_test("T9", "START/STOP did not self-clear"); i2c_recover(); return; }

    I2C_CR1 = CR1_SWRST;
    if (I2C_CR1 != 0u || I2C_CR2 != 0u || I2C_SR1 != 0u || I2C_SR2 != 0u) {
        fail_test("T9", "SWRST did not clear registers"); i2c_recover(); return;
    }
    i2c_init();
    pass_test("Test9 STOP-SWRST");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test 10: AF remains visible until software clears it by writing 0 to SR1.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void test10(void)
{
    unsigned int base = g_log_count;
    put_str("Test 10: AF flag clears by SR1 write-0\r\n");

    I2C_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN;  /* ITERREN off: poll AF */
    I2C_CR1 |= CR1_START;
    if (!i2c_wait_n(base + 1u)) { fail_test("T10", "SB timeout"); i2c_recover(); return; }
    I2C_DR = (0x7Eu << 1u) | 0u;
    if (!i2c_wait_sr1(SR1_AF)) { fail_test("T10", "AF flag timeout"); i2c_recover(); return; }
    I2C_SR1 = 0u;
    if (I2C_SR1 & SR1_AF) { fail_test("T10", "AF did not clear"); i2c_recover(); return; }
    I2C_CR1 |= CR1_STOP;
    i2c_init();
    pass_test("Test10 AF-clear");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Character output helpers (write direct to USART TBUF — no IRQ needed)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void put_char(char c)   { UC_TBUF = (unsigned int)c; }
static void put_str(const char *s) { while (*s) put_char(*s++); }

static void put_hex(unsigned int v)
{
    static const char hex[] = "0123456789ABCDEF";
    int i;
    put_char('0'); put_char('x');
    for (i = 28; i >= 0; i -= 4)
        put_char(hex[(v >> i) & 0xFu]);
}

static void put_dec(unsigned int v)
{
    char buf[12];
    int  i = 0;
    if (v == 0u) { put_char('0'); return; }
    while (v > 0u) { buf[i++] = (char)('0' + v % 10u); v /= 10u; }
    while (--i >= 0) put_char(buf[i]);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════════════════ */
static void i2c_main(void)
{
    setup_vectors();
    setup_gic();
    enable_irq();
    i2c_init();

    put_str("\r\n================================================\r\n");
    put_str("  I2C STM32 ISR Interrupt Stress Test\r\n");
    put_str("  Slave 0x50, EV=IRQ34, ER=IRQ35\r\n");
    put_str("  Frame-level sc_time transfer model under test\r\n");
    put_str("================================================\r\n\r\n");

    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
    test8();
    test9();
    test10();

    put_str("\r\n================================================\r\n");
    put_str("  Passed: "); put_dec((unsigned int)g_pass);
    put_str("   Failed: "); put_dec((unsigned int)g_fail);
    put_str("\r\n");
    put_str("  Total IRQ log entries: "); put_dec(g_log_count);
    put_str("\r\n");
    if (g_fail == 0)
        put_str("  ALL TESTS PASSED\r\n");
    else
        put_str("  SOME TESTS FAILED\r\n");
    put_str("================================================\r\n");

    EXITER = 0u;
}
