#include "cpu_conf.h"

OUTPUT_ARCH( "riscv" )
ENTRY( _start )
MEMORY
{
  rom(rwx) : ORIGIN = RAM_BASE, LENGTH = RAM_SIZE
}

SECTIONS
{
    .text :
    {
        *(.text.start)
        *(.text .text.*)
        . = ALIGN(4);
    } > rom

    .rodata :
    {
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
    } > rom

    .data :
    {
        *(.data)
        *(.data.*)
        . = ALIGN(4);
    } > rom

    .bss :
    {
        *(.bss)
        *(.bss.*)
        . = ALIGN(16);
    } > rom

}