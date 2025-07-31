#include "common.h"
#ifdef NS16550A_BASE /* Must defined after 'common.h' */
#include "sbi/riscv_io.h"
#include "serial.h"
#include <stdint.h>

// UART 寄存器偏移量（标准 8250/16550 定义）
#define UART_THR 0x00          // 发送保持寄存器
#define UART_LSR 0x05          // 线状态寄存器
#define UART_LSR_THRE (1 << 5) // 发送寄存器空标志位

#define UART_BASE (void *)(NS16550A_BASE)

/* 初始化 UART（简化版，仅启用 FIFO 和基本配置） */
void uart8250_init(uint32_t baud_rate) {
    // 假设硬件已默认配置时钟频率（如 1.8432MHz）
    // 如果需要设置波特率，需计算分频值并写入 DLL/DLM 寄存器
    // 此处仅启用 FIFO（如果支持）
    writeb(0x1, UART_BASE + 0x02);
}

static inline void putchar(char ch) {
    while ((readb(UART_BASE + UART_LSR) & UART_LSR_THRE) == 0)
        ;
    writeb(ch, UART_BASE + UART_THR);
}
int puts(const char *s) {
    static int initialized = 0;
    if (!initialized) {
        initialized = 1;
        uart8250_init(115200);
    }

    while (*s)
        putchar(*s++);
    return 0;
}
#endif // NS16550A_BASE