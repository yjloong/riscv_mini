#include "common.h"
#include "sbi/riscv_asm.h"
#include "sbi/riscv_encoding.h"
#include "sbi/riscv_io.h"
#include "sbi/sbi_bitops.h"
#include "sbi/sbi_types.h"
#include "serial.h"
#include <stddef.h>
#include <stdint.h>

#define PC __builtin_return_address(0)
#define ALIGN_UP(x, align) (((x) + ((align) - 1)) & ~((align) - 1))

void m_main();
void s_main();
void m2s();
__attribute__((section(".text.start"))) void _start() {
    asm volatile("li sp, %0" ::"i"(STACK_TOP));
    asm volatile("la t0, trap_entry_m\r\n csrw mtvec, t0");
    asm volatile("la t0, trap_entry_s\r\n csrw stvec, t0");
    asm("jal m_main");
}

void handle_trap_m() {
    uint64_t cause = csr_read(CSR_MCAUSE);
    uint64_t pending = csr_swap(CSR_MIP, 0);
    (void)pending;
    printf("trap_m: %lx\n", cause);
    dead();
}

void handle_trap_s() {
    uint64_t cause = csr_read(CSR_SCAUSE);
    uint64_t pending = csr_swap(CSR_SIP, 0);
    (void)pending;
    printf("trap_s: %lx\n", cause);
    dead();
}

__aligned(8) void s_main() {
    puts("Hello, World!\r\n");
    while (1)
        asm("wfi");
}

void m_main() {
    m2s();
    while (1)
        asm("wfi");
}

void m2s() {
    csr_set(CSR_MSTATUS, MSTATUS_SIE);
    writeq(~0x0UL, (void *)(CLINT_BASE + CLINT_MTIMECMP));
    csr_write(CSR_MIDELEG, ~0x0);
    csr_write(CSR_MEDELEG, ~0x0);
    csr_write(CSR_MIP, 0);
    csr_write(CSR_MIE, ~0x0);

    csr_write(CSR_MEPC, s_main);
    uint64_t status = csr_read(CSR_MSTATUS);
    status = INSERT_FIELD(status, MSTATUS_MPP, PRV_S);
    // when mret: mstatus.MIE = mstatus.MPIE; mstatus.MPIE = 1;
    status = INSERT_FIELD(status, MSTATUS_MPIE, 0);
    csr_write(CSR_MSTATUS, status);

    csr_write(CSR_SATP, 0);
    csr_write(CSR_SIE, ~0x0); // mret doesn't affect CSR_SIE;

    pmp_set(0, 0x7, 0, 0x40);

    asm volatile("mret");
}