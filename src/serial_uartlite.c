#include "common.h"
#ifdef UARTLITE_BASE
#include "sbi/riscv_asm.h"
#include "sbi/riscv_io.h"
#include <stdint.h>

#define UARTLITE_MMIO 0x40600000UL
#define UARTLITE_RX_FIFO 0x0U
#define UARTLITE_TX_FIFO 0x4U
#define UARTLITE_STAT_REG 0x8U
#define UARTLITE_CTRL_REG 0xCU

#define UARTLITE_RST_FIFO 0x3U
#define UARTLITE_TX_FULL 0x8U
#define UARTLITE_TX_VALID 0x1U

static inline void putchar(char c) {
    while (readl((void *)(UARTLITE_MMIO + UARTLITE_STAT_REG)) &
           UARTLITE_TX_FULL)
        ;
    writel(c, (void *)(UARTLITE_MMIO + UARTLITE_TX_FIFO));
}

int puts(const char *s) {
    while (*s)
        putchar(*s++);
    return 0;
}

#endif // UARTLITE_BASE