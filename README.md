[简体中文](README_CN.md)

# Key Features
- **Smart Dependency Chaining**: Precise incremental builds  
- **OpenSBI Integration**: Full access to OpenSBI headers and low-level APIs  
- **Developer-Friendly**: Auto-generated `compile_commands.json` for VSCode + clangd  
- **Multi-Machine Support**: Flexible `MACHINE` targeting (virt/nemu)  
- **Privilege Levels**: Supports both Machine (M) and Supervisor (S) modes  
- **Minimalist Design**: Clean architecture for easy debugging  

# Ideal Use Cases
✅ Lightweight CPU feature validation  
✅ Rapid prototyping without OpenSBI overhead  

# Project Structure
```text
src/
├── main.c                # Entry point
├── opensbi_helper/       # OpenSBI compatibility layer
│   └── riscv_asm.c       # Prebuilt critical assembly interfaces
├── printf.c              # Minimal printf implementation
├── serial_ns16550a.c     # Industry-standard NS16550A UART driver
├── serial_uartlite.c     # Lightweight UARTLite driver
└── trap.S                # Trap/exception entry points
```

# Quick Start
```bash
# 1. Build (select target)
make MACHINE=MACHINE_VIRT   # QEMU virt platform
make MACHINE=MACHINE_NEMU   # NEMU simulator

# 2. Run (virt example)
qemu-system-riscv64 -machine virt -cpu rv64 -nographic -bios output/hello.elf
```
