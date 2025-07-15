# 项目特点
- **模块化依赖管理**：基于链表实现按需编译，减少冗余构建  
- **OpenSBI 深度集成**：完整支持 OpenSBI 头文件及底层功能  
- **开发友好**：自动生成 `compile_commands.json`，支持 VSCode + clangd 智能补全  
- **多硬件支持**：通过 `MACHINE` 参数灵活适配不同设备（virt/nemu）  
- **特权级完备**：支持 Machine mode (M) 和 Supervisor mode (S)  
- **极简设计**：代码精简，结构清晰，便于调试  

# 适用场景
✅ 轻量级 CPU 功能验证  
✅ 替代 OpenSBI 进行快速原型开发  

# 代码结构
```text
src/
├── main.c                # 主程序入口
├── opensbi_helper/       # OpenSBI 兼容层
│   └── riscv_asm.c       # 关键汇编接口预编译实现
├── printf.c              # 精简版 printf 实现
├── serial_ns16550a.c     # 工业标准 NS16550A 串口驱动
├── serial_uartlite.c     # 轻量级 UARTLite 驱动
└── trap.S                # 中断/异常处理入口
```

# 快速开始
```bash
# 1. 编译（选择目标设备）
make MACHINE=MACHINE_VIRT   # QEMU virt 虚拟平台
make MACHINE=MACHINE_NEMU   # NEMU 模拟器

# 2. 运行（以 virt 为例）
qemu-system-riscv64 -machine virt -cpu rv64 -nographic -bios output/hello.elf
```
