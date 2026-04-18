/*
 * hello_usart.c — Baremetal AArch64 firmware exercising the custom USART.
 *
 * Memory map (must match platforms/cortex-a53-virt/conf.lua):
 *   RAM:   0x80000000  (256 MB, stack at top = 0x90000000)
 *   USART: 0x09001000
 *
 * IMPORTANT: _start MUST be the first function in this file so the linker
 * places it at the start of .text (= 0x80000000) and the CPU finds it at
 * the reset vector (rvbar = 0x80000000).
 *
 * Register layout (must match peripherals/usart/include/usart.h):
 *   0x09001000  SR   [7]=TXE  [5]=RXNE
 *   0x09001004  DR   write=TX, read=RX
 *   0x09001008  BRR  ignored in simulation
 *   0x0900100C  CR1  [13]=UE [3]=TE [2]=RE
 */

#define USART_BASE  0x09001000UL
#define USART_SR    (*(volatile unsigned int *)(USART_BASE + 0x00))
#define USART_DR    (*(volatile unsigned int *)(USART_BASE + 0x04))
#define USART_BRR   (*(volatile unsigned int *)(USART_BASE + 0x08))
#define USART_CR1   (*(volatile unsigned int *)(USART_BASE + 0x0C))

#define USART_SR_TXE    (1U << 7)
#define USART_CR1_UE    (1U << 13)
#define USART_CR1_TE    (1U <<  3)
#define USART_CR1_RE    (1U <<  2)

#define PSCI_SYSTEM_OFF 0x84000008UL

/* ---- Entry point — MUST be first to land at 0x80000000 ---- */
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

/* ---- Minimal USART driver ---- */
static void usart_init(void)
{
    USART_BRR = 0x683;                          /* 115200 @ 16 MHz (ignored in sim) */
    USART_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void usart_putc(char c)
{
    while (!(USART_SR & USART_SR_TXE))
        ;
    USART_DR = (unsigned int)c;
}

static void usart_puts(const char *s)
{
    while (*s)
        usart_putc(*s++);
}

/* ---- Application ---- */
void main(void)
{
    usart_init();
    usart_puts("Hello from custom USART!\r\n");
    usart_puts("Peripheral: peripherals/usart/\r\n");
    usart_puts("Platform:   platforms/cortex-a53-virt/\r\n");
    psci_system_off();
}
