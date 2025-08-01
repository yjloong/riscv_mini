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
void s2vs();
__attribute__((section(".text.start"))) void _start() {
    asm volatile("li sp, %0" ::"i"(STACK_TOP));
    asm volatile("la t0, trap_entry_m\r\n csrw mtvec, t0");
    asm volatile("la t0, trap_entry_s\r\n csrw stvec, t0");
    asm volatile("la t0, trap_entry_vs\r\n csrw vstvec, t0");
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

__aligned(8) void handle_trap_vs() {
    uint64_t cause = csr_read(CSR_SCAUSE);
    uint64_t pending = csr_swap(CSR_SIP, 0);
    uint64_t pc = csr_read(CSR_SEPC);
    
    (void)pending;
    (void)pc;
    printf("trap_vs: %lx\n", cause);
    dead();
}

__aligned(8) void vs_main() {
    puts("vs:Hello, World!\r\n");
    while (1)
        asm("wfi");
}

__aligned(8) void s_main() {
    puts("s_main\r\n");
    s2vs();
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

void s2vs() {
    csr_write(CSR_HIDELEG, ~0x0);
    csr_write(CSR_HEDELEG, ~0x0);
    csr_write(CSR_VSIP, 0);
    csr_write(CSR_VSIE, ~0x0);

    //csr_write(CSR_VSEPC, vs_main);
    csr_write(CSR_SEPC, vs_main);
    
    /** Table 36. Value of hstatus field SPV and sstatus field SPP after a trap into HS-mode
     * Previous Mode | SPV | SPP
     * U-mode        | 0   | 0
     * HS-mode       | 0   | 1
     * VU-mode       | 1   | 0
     * VS-mode       | 1   | 1
     */
    /** Table 37. Value of vsstatus field SPP after a trap into VS-mode
     * Previous Mode | SPP
     * VU-mode       | 0
     * VS-mode       | 1
     */
    csr_set(CSR_HSTATUS, HSTATUS_SPV | HSTATUS_SPVP);
    csr_set(CSR_SSTATUS, SSTATUS_SPP);

    csr_write(CSR_VSATP, 0);
    csr_write(CSR_HGATP, 0);
    /**
    19.6.4. Trap Return
    The SRET instruction is used to return from a trap taken into HS-mode or VS-mode. Its behavior depends on the current virtualization mode.
    When executed in M-mode or HS-mode(V=0), SRET first determines what the new privilege mode will be according to 
    the values in hstatus.SPV and sstatus.SPP. SRET then sets hstatus.SPV=0, and in sstatus sets SPP=0, SIE=SPIE, and SPIE=1. Lastly, SRET
    sets the privilege mode as previously determined, and sets pc=sepc.
     */
    asm volatile("sret");
}