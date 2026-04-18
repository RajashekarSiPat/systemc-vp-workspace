/*
 * hello_usart2.c — Baremetal AArch64 firmware for the full USART model.
 *
 * Memory map (must match platforms/cortex-a53-virt/conf_usart2.lua):
 *   RAM:    0x80000000  (256 MB, stack at top = 0x90000000)
 *   USART2: 0x09002000  (peripherals/usart2 — full CON/TBUF/RBUF/BG/FDR model)
 *
 * IMPORTANT: _start MUST be the first function so the linker places it at the
 * start of .text (= 0x80000000) and the CPU finds it at the reset vector
 * (rvbar = 0x80000000).
 *
 * Register layout (matches peripherals/usart2/include/Usart_regdefs.h):
 *   0x09002000  CON   [2:0]=M  [3]=REN  [15]=R  (run)
 *   0x09002004  TBUF  write = TX byte  (9-bit slot; bits [8:0])
 *   0x09002008  RBUF  read  = RX byte
 *   0x0900200C  BG    write = baud-gen reload; read = live counter
 *   0x09002010  FDR   fractional divider
 *
 * Init for Mode 1 (8N1):
 *   CON = CON_M_8N1 | CON_R  →  0x8001
 *
 * TX path in the virtual platform:
 *   The Usart2 QBOX wrapper intercepts every TBUF write and forwards
 *   the byte directly to char_backend_stdio — no busy-wait needed.
 *   The Usart core's b_transport is also called to keep internal state
 *   (advance, BG counter, IRQ timestamps) consistent.
 */

#define USART2_BASE  0x09002000UL

#define USART2_CON   (*(volatile unsigned int *)(USART2_BASE + 0x00))
#define USART2_TBUF  (*(volatile unsigned int *)(USART2_BASE + 0x04))
#define USART2_RBUF  (*(volatile unsigned int *)(USART2_BASE + 0x08))
#define USART2_BG    (*(volatile unsigned int *)(USART2_BASE + 0x0C))
#define USART2_FDR   (*(volatile unsigned int *)(USART2_BASE + 0x10))

/* CON bit fields */
#define CON_M_8N1   (1U << 0)   /* Mode 1 — async 8N1 (8 data, no parity, 1/2 stop) */
#define CON_R       (1U << 15)  /* Run bit — enable peripheral clock */

#define PSCI_SYSTEM_OFF 0x84000008UL

/* ---- Entry point — MUST be first function to land at 0x80000000 ---- */
void __attribute__((naked)) _start(void)
{
    __asm__ volatile(
        "ldr x0, =0x90000000\n"   /* stack top */
        "mov sp, x0\n"
        "bl  main\n"
        "1: b 1b\n"               /* halt if main returns */
    );
}

/* ---- PSCI system-off (AArch64 HVC) ---- */
static inline void __attribute__((noreturn)) psci_system_off(void)
{
    register unsigned long x0 __asm__("x0") = PSCI_SYSTEM_OFF;
    __asm__ volatile("hvc #0" : : "r"(x0));
    __builtin_unreachable();
}

/* ---- USART2 driver ---- */
static void usart2_init(void)
{
    USART2_BG  = 0;                     /* max baud rate (timing not needed in VP) */
    USART2_CON = CON_M_8N1 | CON_R;    /* Mode 1 (8N1) + enable clock */
}

static void usart2_putc(char c)
{
    /* In the virtual platform the Usart2 wrapper intercepts the TBUF write and
     * forwards the byte immediately to char_backend_stdio — no polling needed. */
    USART2_TBUF = (unsigned int)c;
}

static void usart2_puts(const char *s)
{
    while (*s)
        usart2_putc(*s++);
}

/* ---- Application ---- */
void main(void)
{
    usart2_init();
    usart2_puts("Hello from Usart2 (full model)!\r\n");
    usart2_puts("Peripheral:  peripherals/usart2/\r\n");
    usart2_puts("Register map: CON / TBUF / RBUF / BG / FDR\r\n");
    usart2_puts("Modes: 8N1, 9-bit, 7+P, 8+P, 8N2, synchronous\r\n");
    psci_system_off();
}
