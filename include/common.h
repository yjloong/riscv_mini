#ifndef _COMMON_H
#define _COMMON_H
#include "cpu_conf.h"

/* machine configuration */
#ifdef MACHINE_NEMU
#define CLINT_BASE 0x38000000UL
#define UARTLITE_BASE 0x40600000UL
#elifdef MACHINE_VIRT
#define CLINT_BASE 0x2000000UL
#define NS16550A_BASE 0x10000000UL
#endif
#define CLINT_MTIMECMP 0x4000

/* public function declarations */
int printf(const char *restrict, ...);

/* public macro declarations */
#define dead()                                                                 \
  do {                                                                         \
    while (1)                                                                  \
      asm("wfi");                                                              \
  } while(0)

#endif