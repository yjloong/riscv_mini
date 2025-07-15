#include "common.h"

unsigned long csr_read_num(int csr_num);

void csr_write_num(int csr_num, unsigned long val);

int misa_extension_imp(char ext);

int misa_xlen(void);

void misa_string(int xlen, char *out, unsigned int out_sz);

int pmp_disable(unsigned int n);

int is_pmp_entry_mapped(unsigned long entry);

int pmp_set(unsigned int n, unsigned long prot, unsigned long addr,
            unsigned long log2len);

int pmp_get(unsigned int n, unsigned long *prot_out, unsigned long *addr_out,
            unsigned long *log2len);

typedef char s8;
typedef unsigned char u8;
typedef unsigned char uint8_t;

typedef short s16;
typedef unsigned short u16;
typedef short int16_t;
typedef unsigned short uint16_t;

typedef int s32;
typedef unsigned int u32;
typedef int int32_t;
typedef unsigned int uint32_t;

typedef long s64;
typedef unsigned long u64;
typedef long int64_t;
typedef unsigned long uint64_t;

typedef _Bool bool;

typedef unsigned long ulong;
typedef unsigned long uintptr_t;
typedef unsigned long size_t;
typedef long ssize_t;
typedef unsigned long virtual_addr_t;
typedef unsigned long virtual_size_t;
typedef unsigned long physical_addr_t;
typedef unsigned long physical_size_t;

typedef uint16_t le16_t;
typedef uint16_t be16_t;
typedef uint32_t le32_t;
typedef uint32_t be32_t;
typedef uint64_t le64_t;
typedef uint64_t be64_t;


enum sbi_fwft_feature_t {
  SBI_FWFT_MISALIGNED_EXC_DELEG = 0x0,
  SBI_FWFT_LANDING_PAD = 0x1,
  SBI_FWFT_SHADOW_STACK = 0x2,
  SBI_FWFT_DOUBLE_TRAP = 0x3,
  SBI_FWFT_PTE_AD_HW_UPDATING = 0x4,
  SBI_FWFT_POINTER_MASKING_PMLEN = 0x5,
  SBI_FWFT_LOCAL_RESERVED_START = 0x6,
  SBI_FWFT_LOCAL_RESERVED_END = 0x3fffffff,
  SBI_FWFT_LOCAL_PLATFORM_START = 0x40000000,
  SBI_FWFT_LOCAL_PLATFORM_END = 0x7fffffff,

  SBI_FWFT_GLOBAL_RESERVED_START = 0x80000000,
  SBI_FWFT_GLOBAL_RESERVED_END = 0xbfffffff,
  SBI_FWFT_GLOBAL_PLATFORM_START = 0xc0000000,
  SBI_FWFT_GLOBAL_PLATFORM_END = 0xffffffff,
};

enum sbi_pmu_hw_generic_events_t {
  SBI_PMU_HW_NO_EVENT = 0,
  SBI_PMU_HW_CPU_CYCLES = 1,
  SBI_PMU_HW_INSTRUCTIONS = 2,
  SBI_PMU_HW_CACHE_REFERENCES = 3,
  SBI_PMU_HW_CACHE_MISSES = 4,
  SBI_PMU_HW_BRANCH_INSTRUCTIONS = 5,
  SBI_PMU_HW_BRANCH_MISSES = 6,
  SBI_PMU_HW_BUS_CYCLES = 7,
  SBI_PMU_HW_STALLED_CYCLES_FRONTEND = 8,
  SBI_PMU_HW_STALLED_CYCLES_BACKEND = 9,
  SBI_PMU_HW_REF_CPU_CYCLES = 10,

  SBI_PMU_HW_GENERAL_MAX,
};

enum sbi_pmu_hw_cache_id {
  SBI_PMU_HW_CACHE_L1D = 0,
  SBI_PMU_HW_CACHE_L1I = 1,
  SBI_PMU_HW_CACHE_LL = 2,
  SBI_PMU_HW_CACHE_DTLB = 3,
  SBI_PMU_HW_CACHE_ITLB = 4,
  SBI_PMU_HW_CACHE_BPU = 5,
  SBI_PMU_HW_CACHE_NODE = 6,

  SBI_PMU_HW_CACHE_MAX,
};

enum sbi_pmu_hw_cache_op_id {
  SBI_PMU_HW_CACHE_OP_READ = 0,
  SBI_PMU_HW_CACHE_OP_WRITE = 1,
  SBI_PMU_HW_CACHE_OP_PREFETCH = 2,

  SBI_PMU_HW_CACHE_OP_MAX,
};

enum sbi_pmu_hw_cache_op_result_id {
  SBI_PMU_HW_CACHE_RESULT_ACCESS = 0,
  SBI_PMU_HW_CACHE_RESULT_MISS = 1,

  SBI_PMU_HW_CACHE_RESULT_MAX,
};

enum sbi_pmu_fw_event_code_id {
  SBI_PMU_FW_MISALIGNED_LOAD = 0,
  SBI_PMU_FW_MISALIGNED_STORE = 1,
  SBI_PMU_FW_ACCESS_LOAD = 2,
  SBI_PMU_FW_ACCESS_STORE = 3,
  SBI_PMU_FW_ILLEGAL_INSN = 4,
  SBI_PMU_FW_SET_TIMER = 5,
  SBI_PMU_FW_IPI_SENT = 6,
  SBI_PMU_FW_IPI_RECVD = 7,
  SBI_PMU_FW_FENCE_I_SENT = 8,
  SBI_PMU_FW_FENCE_I_RECVD = 9,
  SBI_PMU_FW_SFENCE_VMA_SENT = 10,
  SBI_PMU_FW_SFENCE_VMA_RCVD = 11,
  SBI_PMU_FW_SFENCE_VMA_ASID_SENT = 12,
  SBI_PMU_FW_SFENCE_VMA_ASID_RCVD = 13,

  SBI_PMU_FW_HFENCE_GVMA_SENT = 14,
  SBI_PMU_FW_HFENCE_GVMA_RCVD = 15,
  SBI_PMU_FW_HFENCE_GVMA_VMID_SENT = 16,
  SBI_PMU_FW_HFENCE_GVMA_VMID_RCVD = 17,

  SBI_PMU_FW_HFENCE_VVMA_SENT = 18,
  SBI_PMU_FW_HFENCE_VVMA_RCVD = 19,
  SBI_PMU_FW_HFENCE_VVMA_ASID_SENT = 20,
  SBI_PMU_FW_HFENCE_VVMA_ASID_RCVD = 21,
  SBI_PMU_FW_MAX,

  SBI_PMU_FW_RESERVED_MAX = 0xFFFE,

  SBI_PMU_FW_PLATFORM = 0xFFFF,
};

enum sbi_pmu_event_type_id {
  SBI_PMU_EVENT_TYPE_HW = 0x0,
  SBI_PMU_EVENT_TYPE_HW_CACHE = 0x1,
  SBI_PMU_EVENT_TYPE_HW_RAW = 0x2,
  SBI_PMU_EVENT_TYPE_HW_RAW_V2 = 0x3,
  SBI_PMU_EVENT_TYPE_FW = 0xf,
  SBI_PMU_EVENT_TYPE_MAX,
};

enum sbi_pmu_ctr_type {
  SBI_PMU_CTR_TYPE_HW = 0,
  SBI_PMU_CTR_TYPE_FW,
};

struct sbi_pmu_event_info {
  uint32_t event_idx;
  uint32_t output;
  uint64_t event_data;
};

enum sbi_cppc_reg_id {
  SBI_CPPC_HIGHEST_PERF = 0x00000000,
  SBI_CPPC_NOMINAL_PERF = 0x00000001,
  SBI_CPPC_LOW_NON_LINEAR_PERF = 0x00000002,
  SBI_CPPC_LOWEST_PERF = 0x00000003,
  SBI_CPPC_GUARANTEED_PERF = 0x00000004,
  SBI_CPPC_DESIRED_PERF = 0x00000005,
  SBI_CPPC_MIN_PERF = 0x00000006,
  SBI_CPPC_MAX_PERF = 0x00000007,
  SBI_CPPC_PERF_REDUC_TOLERANCE = 0x00000008,
  SBI_CPPC_TIME_WINDOW = 0x00000009,
  SBI_CPPC_CTR_WRAP_TIME = 0x0000000A,
  SBI_CPPC_REFERENCE_CTR = 0x0000000B,
  SBI_CPPC_DELIVERED_CTR = 0x0000000C,
  SBI_CPPC_PERF_LIMITED = 0x0000000D,
  SBI_CPPC_ENABLE = 0x0000000E,
  SBI_CPPC_AUTO_SEL_ENABLE = 0x0000000F,
  SBI_CPPC_AUTO_ACT_WINDOW = 0x00000010,
  SBI_CPPC_ENERGY_PERF_PREFERENCE = 0x00000011,
  SBI_CPPC_REFERENCE_PERF = 0x00000012,
  SBI_CPPC_LOWEST_FREQ = 0x00000013,
  SBI_CPPC_NOMINAL_FREQ = 0x00000014,
  SBI_CPPC_ACPI_LAST = SBI_CPPC_NOMINAL_FREQ,
  SBI_CPPC_TRANSITION_LATENCY = 0x80000000,
  SBI_CPPC_NON_ACPI_LAST = SBI_CPPC_TRANSITION_LATENCY,
};

enum sbi_sse_attr_id {
  SBI_SSE_ATTR_STATUS = 0x00000000,
  SBI_SSE_ATTR_PRIO = 0x00000001,
  SBI_SSE_ATTR_CONFIG = 0x00000002,
  SBI_SSE_ATTR_PREFERRED_HART = 0x00000003,
  SBI_SSE_ATTR_ENTRY_PC = 0x00000004,
  SBI_SSE_ATTR_ENTRY_ARG = 0x00000005,
  SBI_SSE_ATTR_INTERRUPTED_SEPC = 0x00000006,
  SBI_SSE_ATTR_INTERRUPTED_FLAGS = 0x00000007,
  SBI_SSE_ATTR_INTERRUPTED_A6 = 0x00000008,
  SBI_SSE_ATTR_INTERRUPTED_A7 = 0x00000009,

  SBI_SSE_ATTR_MAX = 0x0000000A
};

enum sbi_sse_state {
  SBI_SSE_STATE_UNUSED = 0,
  SBI_SSE_STATE_REGISTERED = 1,
  SBI_SSE_STATE_ENABLED = 2,
  SBI_SSE_STATE_RUNNING = 3,
};






struct sbi_scratch {

  unsigned long fw_start;

  unsigned long fw_size;

  unsigned long fw_rw_offset;

  unsigned long fw_heap_offset;

  unsigned long fw_heap_size;

  unsigned long next_arg1;

  unsigned long next_addr;

  unsigned long next_mode;

  unsigned long warmboot_addr;

  unsigned long platform_addr;

  unsigned long hartid_to_scratch;

  unsigned long trap_context;

  unsigned long tmp0;

  unsigned long options;

  unsigned long hartindex;
};

_Static_assert((__builtin_offsetof(struct sbi_scratch, fw_start)) == ((0 * 8)),
               "The offset "
               "SBI_SCRATCH_FW_START_OFFSET"
               " of "
               "fw_start"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, fw_size)) == ((1 * 8)),
               "The offset "
               "SBI_SCRATCH_FW_SIZE_OFFSET"
               " of "
               "fw_size"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, fw_rw_offset)) ==
                   ((2 * 8)),
               "The offset "
               "SBI_SCRATCH_FW_RW_OFFSET"
               " of "
               "fw_rw_offset"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, fw_heap_offset)) ==
                   ((3 * 8)),
               "The offset "
               "SBI_SCRATCH_FW_HEAP_OFFSET"
               " of "
               "fw_heap_offset"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, fw_heap_size)) ==
                   ((4 * 8)),
               "The offset "
               "SBI_SCRATCH_FW_HEAP_SIZE_OFFSET"
               " of "
               "fw_heap_size"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, next_arg1)) == ((5 * 8)),
               "The offset "
               "SBI_SCRATCH_NEXT_ARG1_OFFSET"
               " of "
               "next_arg1"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, next_addr)) == ((6 * 8)),
               "The offset "
               "SBI_SCRATCH_NEXT_ADDR_OFFSET"
               " of "
               "next_addr"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, next_mode)) == ((7 * 8)),
               "The offset "
               "SBI_SCRATCH_NEXT_MODE_OFFSET"
               " of "
               "next_mode"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, warmboot_addr)) ==
                   ((8 * 8)),
               "The offset "
               "SBI_SCRATCH_WARMBOOT_ADDR_OFFSET"
               " of "
               "warmboot_addr"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, platform_addr)) ==
                   ((9 * 8)),
               "The offset "
               "SBI_SCRATCH_PLATFORM_ADDR_OFFSET"
               " of "
               "platform_addr"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, hartid_to_scratch)) ==
                   ((10 * 8)),
               "The offset "
               "SBI_SCRATCH_HARTID_TO_SCRATCH_OFFSET"
               " of "
               "hartid_to_scratch"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, trap_context)) ==
                   ((11 * 8)),
               "The offset "
               "SBI_SCRATCH_TRAP_CONTEXT_OFFSET"
               " of "
               "trap_context"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, tmp0)) == ((12 * 8)),
               "The offset "
               "SBI_SCRATCH_TMP0_OFFSET"
               " of "
               "tmp0"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, options)) == ((13 * 8)),
               "The offset "
               "SBI_SCRATCH_OPTIONS_OFFSET"
               " of "
               "options"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_scratch, hartindex)) ==
                   ((14 * 8)),
               "The offset "
               "SBI_SCRATCH_HARTINDEX_OFFSET"
               " of "
               "hartindex"
               " in "
               "struct sbi_scratch"
               "is not correct, please redefine it.");

enum sbi_scratch_options {

  SBI_SCRATCH_NO_BOOT_PRINTS = (1 << 0),

  SBI_SCRATCH_DEBUG_PRINTS = (1 << 1),
};

int sbi_scratch_init(struct sbi_scratch *scratch);

unsigned long sbi_scratch_alloc_offset(unsigned long size);

void sbi_scratch_free_offset(unsigned long offset);

unsigned long sbi_scratch_used_space(void);

extern u32 sbi_scratch_hart_count;

extern u32 hartindex_to_hartid_table[];

extern struct sbi_scratch *hartindex_to_scratch_table[];

u32 sbi_hartid_to_hartindex(u32 hartid);







struct sbi_trap_regs {

  unsigned long zero;

  unsigned long ra;

  unsigned long sp;

  unsigned long gp;

  unsigned long tp;

  unsigned long t0;

  unsigned long t1;

  unsigned long t2;

  unsigned long s0;

  unsigned long s1;

  unsigned long a0;

  unsigned long a1;

  unsigned long a2;

  unsigned long a3;

  unsigned long a4;

  unsigned long a5;

  unsigned long a6;

  unsigned long a7;

  unsigned long s2;

  unsigned long s3;

  unsigned long s4;

  unsigned long s5;

  unsigned long s6;

  unsigned long s7;

  unsigned long s8;

  unsigned long s9;

  unsigned long s10;

  unsigned long s11;

  unsigned long t3;

  unsigned long t4;

  unsigned long t5;

  unsigned long t6;

  unsigned long mepc;

  unsigned long mstatus;

  unsigned long mstatusH;
};

struct sbi_trap_info {

  unsigned long cause;

  unsigned long tval;

  unsigned long tval2;

  unsigned long tinst;

  unsigned long gva;
};

struct sbi_trap_context {

  struct sbi_trap_regs regs;

  struct sbi_trap_info trap;

  struct sbi_trap_context *prev_context;
};

static inline unsigned long sbi_regs_gva(const struct sbi_trap_regs *regs) {

  return (regs->mstatus & ((0x0000004000000000ULL))) ? 1 : 0;
}

static inline bool sbi_regs_from_virt(const struct sbi_trap_regs *regs) {

  return (regs->mstatus & ((0x0000008000000000ULL))) ? 1 : 0;
}

static inline int sbi_mstatus_prev_mode(unsigned long mstatus) {
  return (mstatus & (((3UL)) << 11)) >> 11;
}

int sbi_trap_redirect(struct sbi_trap_regs *regs,
                      const struct sbi_trap_info *trap);

static inline struct sbi_trap_context *
sbi_trap_get_context(struct sbi_scratch *scratch) {
  return (scratch) ? (void *)scratch->trap_context : ((void *)0);
}

static inline void sbi_trap_set_context(struct sbi_scratch *scratch,
                                        struct sbi_trap_context *tcntx) {
  scratch->trap_context = (unsigned long)tcntx;
}

struct sbi_trap_context *sbi_trap_handler(struct sbi_trap_context *tcntx);


union sbi_ldst_data {
  u64 data_u64;
  u32 data_u32;
  u8 data_bytes[8];
  ulong data_ulong;
};

int sbi_misaligned_load_handler(struct sbi_trap_context *tcntx);

int sbi_misaligned_store_handler(struct sbi_trap_context *tcntx);

int sbi_load_access_handler(struct sbi_trap_context *tcntx);

int sbi_store_access_handler(struct sbi_trap_context *tcntx);

ulong sbi_misaligned_tinst_fixup(ulong orig_tinst, ulong new_tinst,
                                 ulong addr_offset);

int sbi_misaligned_v_ld_emulator(int rlen, union sbi_ldst_data *out_val,
                                 struct sbi_trap_context *tcntx);

int sbi_misaligned_v_st_emulator(int wlen, union sbi_ldst_data in_val,
                                 struct sbi_trap_context *tcntx);


struct sbi_domain_memregion;
struct sbi_ecall_return;
struct sbi_trap_regs;
struct sbi_hart_features;
union sbi_ldst_data;

enum sbi_platform_features {

  SBI_PLATFORM_HAS_MFAULTS_DELEGATION = (1 << 1),

  SBI_PLATFORM_HAS_LAST_FEATURE = SBI_PLATFORM_HAS_MFAULTS_DELEGATION,
};

struct sbi_platform_operations {

  bool (*cold_boot_allowed)(u32 hartid);

  int (*nascent_init)(void);

  int (*early_init)(bool cold_boot);

  int (*final_init)(bool cold_boot);

  void (*early_exit)(void);

  void (*final_exit)(void);

  int (*misa_check_extension)(char ext);

  int (*misa_get_xlen)(void);

  int (*extensions_init)(struct sbi_hart_features *hfeatures);

  int (*domains_init)(void);

  int (*pmu_init)(void);

  uint64_t (*pmu_xlate_to_mhpmevent)(uint32_t event_idx, uint64_t data);

  int (*irqchip_init)(void);

  int (*ipi_init)(void);

  u64 (*get_tlbr_flush_limit)(void);

  u32 (*get_tlb_num_entries)(void);

  int (*timer_init)(void);

  int (*mpxy_init)(void);

  int (*vendor_ext_provider)(long funcid, struct sbi_trap_regs *regs,
                             struct sbi_ecall_return *out);

  int (*emulate_load)(int rlen, unsigned long addr,
                      union sbi_ldst_data *out_val);

  int (*emulate_store)(int wlen, unsigned long addr,
                       union sbi_ldst_data in_val);

  void (*pmp_set)(unsigned int n, unsigned long flags, unsigned long prot,
                  unsigned long addr, unsigned long log2len);

  void (*pmp_disable)(unsigned int n);
};

struct sbi_platform {

  u32 opensbi_version;

  u32 platform_version;

  char name[64];

  u64 features;

  u32 hart_count;

  u32 hart_stack_size;

  u32 heap_size;

  u32 reserved;

  unsigned long platform_ops_addr;

  unsigned long firmware_context;

  const u32 *hart_index2id;

  unsigned long cbom_block_size;
};

_Static_assert((__builtin_offsetof(struct sbi_platform, opensbi_version)) ==
                   ((0x00)),
               "The offset "
               "SBI_PLATFORM_OPENSBI_VERSION_OFFSET"
               " of "
               "opensbi_version"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, platform_version)) ==
                   ((0x04)),
               "The offset "
               "SBI_PLATFORM_VERSION_OFFSET"
               " of "
               "platform_version"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, name)) == ((0x08)),
               "The offset "
               "SBI_PLATFORM_NAME_OFFSET"
               " of "
               "name"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, features)) == ((0x48)),
               "The offset "
               "SBI_PLATFORM_FEATURES_OFFSET"
               " of "
               "features"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, hart_count)) ==
                   ((0x50)),
               "The offset "
               "SBI_PLATFORM_HART_COUNT_OFFSET"
               " of "
               "hart_count"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, hart_stack_size)) ==
                   ((0x54)),
               "The offset "
               "SBI_PLATFORM_HART_STACK_SIZE_OFFSET"
               " of "
               "hart_stack_size"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, heap_size)) == ((0x58)),
               "The offset "
               "SBI_PLATFORM_HEAP_SIZE_OFFSET"
               " of "
               "heap_size"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, reserved)) == ((0x5c)),
               "The offset "
               "SBI_PLATFORM_RESERVED_OFFSET"
               " of "
               "reserved"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, platform_ops_addr)) ==
                   ((0x60)),
               "The offset "
               "SBI_PLATFORM_OPS_OFFSET"
               " of "
               "platform_ops_addr"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, firmware_context)) ==
                   ((0x60 + 8)),
               "The offset "
               "SBI_PLATFORM_FIRMWARE_CONTEXT_OFFSET"
               " of "
               "firmware_context"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, hart_index2id)) ==
                   ((0x60 + (8 * 2))),
               "The offset "
               "SBI_PLATFORM_HART_INDEX2ID_OFFSET"
               " of "
               "hart_index2id"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");
_Static_assert((__builtin_offsetof(struct sbi_platform, cbom_block_size)) ==
                   ((0x60 + (8 * 3))),
               "The offset "
               "SBI_PLATFORM_CBOM_BLOCK_SIZE_OFFSET"
               " of "
               "cbom_block_size"
               " in "
               "struct sbi_platform"
               "is not correct, please redefine it.");

void sbi_platform_get_features_str(const struct sbi_platform *plat,
                                   char *features_str, int nfstr);

static inline const char *sbi_platform_name(const struct sbi_platform *plat) {
  if (plat)
    return plat->name;
  return "Unknown";
}

static inline unsigned long
sbi_platform_get_features(const struct sbi_platform *plat) {
  if (plat)
    return plat->features;
  return 0;
}

static inline u64
sbi_platform_tlbr_flush_limit(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->get_tlbr_flush_limit)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->get_tlbr_flush_limit();
  return (1UL << 12);
}

static inline u32
sbi_platform_tlb_fifo_num_entries(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->get_tlb_num_entries)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->get_tlb_num_entries();
  return sbi_scratch_hart_count;
}

static inline u32 sbi_platform_hart_count(const struct sbi_platform *plat) {
  if (plat)
    return plat->hart_count;
  return 0;
}

static inline u32
sbi_platform_hart_stack_size(const struct sbi_platform *plat) {
  if (plat)
    return plat->hart_stack_size;
  return 0;
}

static inline bool
sbi_platform_cold_boot_allowed(const struct sbi_platform *plat, u32 hartid) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->cold_boot_allowed)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->cold_boot_allowed(hartid);
  return 1;
}

static inline int sbi_platform_nascent_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->nascent_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->nascent_init();
  return 0;
}

static inline int sbi_platform_early_init(const struct sbi_platform *plat,
                                          bool cold_boot) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->early_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->early_init(cold_boot);
  return 0;
}

static inline int sbi_platform_final_init(const struct sbi_platform *plat,
                                          bool cold_boot) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->final_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->final_init(cold_boot);
  return 0;
}

static inline void sbi_platform_early_exit(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->early_exit)
    ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->early_exit();
}

static inline void sbi_platform_final_exit(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->final_exit)
    ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->final_exit();
}

static inline int sbi_platform_misa_extension(const struct sbi_platform *plat,
                                              char ext) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->misa_check_extension)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->misa_check_extension(ext);
  return 0;
}

static inline int sbi_platform_misa_xlen(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->misa_get_xlen)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->misa_get_xlen();
  return -1;
}

static inline int
sbi_platform_extensions_init(const struct sbi_platform *plat,
                             struct sbi_hart_features *hfeatures) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->extensions_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->extensions_init(hfeatures);
  return 0;
}

static inline int sbi_platform_domains_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->domains_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->domains_init();
  return 0;
}

static inline int sbi_platform_pmu_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->pmu_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->pmu_init();
  return 0;
}

static inline uint64_t
sbi_platform_pmu_xlate_to_mhpmevent(const struct sbi_platform *plat,
                                    uint32_t event_idx, uint64_t data) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->pmu_xlate_to_mhpmevent)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->pmu_xlate_to_mhpmevent(event_idx, data);
  return 0;
}

static inline int sbi_platform_irqchip_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->irqchip_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->irqchip_init();
  return 0;
}

static inline int sbi_platform_ipi_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->ipi_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->ipi_init();
  return 0;
}

static inline int sbi_platform_timer_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->timer_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->timer_init();
  return 0;
}

static inline int sbi_platform_mpxy_init(const struct sbi_platform *plat) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->mpxy_init)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->mpxy_init();
  return 0;
}

static inline bool
sbi_platform_vendor_ext_check(const struct sbi_platform *plat) {
  return plat &&
         ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
             ->vendor_ext_provider;
}

static inline int
sbi_platform_vendor_ext_provider(const struct sbi_platform *plat, long funcid,
                                 struct sbi_trap_regs *regs,
                                 struct sbi_ecall_return *out) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->vendor_ext_provider)
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->vendor_ext_provider(funcid, regs, out);

  return -2;
}

static inline int sbi_platform_emulate_load(const struct sbi_platform *plat,
                                            int rlen, unsigned long addr,
                                            union sbi_ldst_data *out_val) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->emulate_load) {
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->emulate_load(rlen, addr, out_val);
  }
  return -2;
}

static inline int sbi_platform_emulate_store(const struct sbi_platform *plat,
                                             int wlen, unsigned long addr,
                                             union sbi_ldst_data in_val) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->emulate_store) {
    return ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->emulate_store(wlen, addr, in_val);
  }
  return -2;
}

static inline void sbi_platform_pmp_set(const struct sbi_platform *plat,
                                        unsigned int n, unsigned long flags,
                                        unsigned long prot, unsigned long addr,
                                        unsigned long log2len) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->pmp_set)
    ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->pmp_set(n, flags, prot, addr, log2len);
}

static inline void sbi_platform_pmp_disable(const struct sbi_platform *plat,
                                            unsigned int n) {
  if (plat &&
      ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
          ->pmp_disable)
    ((const struct sbi_platform_operations *)(plat)->platform_ops_addr)
        ->pmp_disable(n);
}



struct sbi_console_device {

  char name[32];

  void (*console_putc)(char ch);

  unsigned long (*console_puts)(const char *str, unsigned long len);

  int (*console_getc)(void);
};

bool sbi_isprintable(char ch);

int sbi_getc(void);

void sbi_putc(char ch);

void sbi_puts(const char *str);

unsigned long sbi_nputs(const char *str, unsigned long len);

void sbi_gets(char *s, int maxwidth, char endchar);

unsigned long sbi_ngets(char *str, unsigned long len);

int __attribute__((format(printf, 2, 3))) sbi_sprintf(char *out,
                                                      const char *format, ...);

int __attribute__((format(printf, 3, 4))) sbi_snprintf(char *out, u32 out_sz,
                                                       const char *format, ...);

int __attribute__((format(printf, 1, 2))) sbi_printf(const char *format, ...);

int __attribute__((format(printf, 1, 2))) sbi_dprintf(const char *format, ...);

void __attribute__((format(printf, 1, 2))) __attribute__((noreturn))
sbi_panic(const char *format, ...);

const struct sbi_console_device *sbi_console_get_device(void);

void sbi_console_set_device(const struct sbi_console_device *dev);

struct sbi_scratch;


int misa_extension_imp(char ext) {
  unsigned long misa = ({
    register unsigned long __v;
    __asm__ __volatile__("csrr %0, "
                         "0x301"
                         : "=r"(__v)
                         :
                         : "memory");
    __v;
  });

  if (misa) {
    if ('A' <= ext && ext <= 'Z')
      return misa & (1 << (ext - 'A'));
    if ('a' <= ext && ext <= 'z')
      return misa & (1 << (ext - 'a'));
    return 0;
  }

  return sbi_platform_misa_extension(
      ((const struct sbi_platform *)(((struct sbi_scratch *)({
                                       register unsigned long __v;
                                       __asm__("csrr %0, "
                                               "0x340"
                                               : "=r"(__v));
                                       __v;
                                     }))->platform_addr)),
      ext);
}

int misa_xlen(void) {
  long r;

  if (({
        register unsigned long __v;
        __asm__ __volatile__("csrr %0, "
                             "0x301"
                             : "=r"(__v)
                             :
                             : "memory");
        __v;
      }) == 0)
    return sbi_platform_misa_xlen(
        ((const struct sbi_platform *)(((struct sbi_scratch *)({
                                         register unsigned long __v;
                                         __asm__("csrr %0, "
                                                 "0x340"
                                                 : "=r"(__v));
                                         __v;
                                       }))->platform_addr)));

  __asm__ __volatile__("csrr   t0, misa\n\t"
                       "slti   t1, t0, 0\n\t"
                       "slli   t1, t1, 1\n\t"
                       "slli   t0, t0, 1\n\t"
                       "slti   t0, t0, 0\n\t"
                       "add    %0, t0, t1"
                       : "=r"(r)
                       :
                       : "t0", "t1");

  return r ? r : -1;
}

void misa_string(int xlen, char *out, unsigned int out_sz) {
  unsigned int i, pos = 0;
  const char valid_isa_order[] = "iemafdqclbjtpvnhkorwxyzg";

  if (!out)
    return;

  if (5 <= (out_sz - pos)) {
    out[pos++] = 'r';
    out[pos++] = 'v';
    switch (xlen) {
    case 1:
      out[pos++] = '3';
      out[pos++] = '2';
      break;
    case 2:
      out[pos++] = '6';
      out[pos++] = '4';
      break;
    case 3:
      out[pos++] = '1';
      out[pos++] = '2';
      out[pos++] = '8';
      break;
    default:
      printf("%s: Unknown misa.MXL encoding %d", __func__, xlen);
      return;
    }
  }

  for (i = 0; i < (sizeof(valid_isa_order) / sizeof((valid_isa_order)[0])) &&
              (pos < out_sz);
       i++) {
    if (misa_extension_imp(valid_isa_order[i]))
      out[pos++] = valid_isa_order[i];
  }

  if (pos < out_sz)
    out[pos++] = '\0';
}

unsigned long csr_read_num(int csr_num) {

  unsigned long ret = 0;

  switch (csr_num) {
  case 0x3a0 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3a0 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3a0 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 0 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 0 + 16 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 0 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x3b0 + 32 + 16 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb00:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb00"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb02:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb02"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb03:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb03"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb04 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb04 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb04 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb04 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb04 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb04 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb04 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb04 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb08 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb08 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0xb10 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0xb10 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x320:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x320"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x321:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x321"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x322:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x322"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x323:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x323"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x324 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x324 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x324 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x324 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x324 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x324 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x324 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x324 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x328 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x328 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 0 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 0 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 0 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 0 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 0 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 0 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 0 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 0 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 0 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 0 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 4 + 0 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 4 + 0 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 4 + 0 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 4 + 0 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 4 + 2 + 0:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 4 + 2 + 0"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;
  case 0x330 + 8 + 4 + 2 + 1:
    ret = ({
      register unsigned long __v;
      __asm__ __volatile__("csrr %0, "
                           "0x330 + 8 + 4 + 2 + 1"
                           : "=r"(__v)
                           :
                           : "memory");
      __v;
    });
    break;

  default:
    printf("%s: Unknown CSR %#x", __func__, csr_num);
    break;
  }

  return ret;

}

void csr_write_num(int csr_num, unsigned long val) {

  switch (csr_num) {
  case 0x3a0 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3a0 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3a0 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 0 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 0 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 0 + 16 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 0 + 16 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 0 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 0 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x3b0 + 32 + 16 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x3b0 + 32 + 16 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb00:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb00"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb02:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb02"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb03:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb03"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb04 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb04 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb04 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb04 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb04 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb04 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb04 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb04 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb08 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb08 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0xb10 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0xb10 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;

  case 0x320:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x320"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x321:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x321"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x322:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x322"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x323:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x323"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x324 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x324 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x324 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x324 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x324 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x324 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x324 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x324 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x328 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x328 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 0 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 0 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 0 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 0 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 0 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 0 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 0 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 0 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 0 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 0 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 4 + 0 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 4 + 0 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 4 + 0 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 4 + 0 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 4 + 2 + 0:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 4 + 2 + 0"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;
  case 0x330 + 8 + 4 + 2 + 1:
    ({
      unsigned long __v = (unsigned long)(val);
      __asm__ __volatile__("csrw "
                           "0x330 + 8 + 4 + 2 + 1"
                           ", %0"
                           :
                           : "rK"(__v)
                           : "memory");
    });
    break;

  default:
    printf("%s: Unknown CSR %#x", __func__, csr_num);
    break;
  }

}

static unsigned long ctz(unsigned long x) {
  unsigned long ret = 0;

  if (x == 0)
    return 8 * sizeof(x);

  while (!(x & 1UL)) {
    ret++;
    x = x >> 1;
  }

  return ret;
}

int pmp_disable(unsigned int n) {
  int pmpcfg_csr, pmpcfg_shift;
  unsigned long cfgmask, pmpcfg;

  if (n >= 64)
    return -3;

  pmpcfg_csr = (0x3a0 + (n >> 2)) & ~1;
  pmpcfg_shift = (n & 7) << 3;

  cfgmask = ~(0xffUL << pmpcfg_shift);
  pmpcfg = (csr_read_num(pmpcfg_csr) & cfgmask);

  csr_write_num(pmpcfg_csr, pmpcfg);

  return 0;
}

int is_pmp_entry_mapped(unsigned long entry) {
  unsigned long prot;
  unsigned long addr;
  unsigned long log2len;

  if (pmp_get(entry, &prot, &addr, &log2len) != 0)
    return 0;

  if (prot & ((0x18UL)))
    return 1;

  return 0;
}

int pmp_set(unsigned int n, unsigned long prot, unsigned long addr,
            unsigned long log2len) {
  int pmpcfg_csr, pmpcfg_shift, pmpaddr_csr;
  unsigned long cfgmask, pmpcfg;
  unsigned long addrmask, pmpaddr;

  if (n >= 64 || log2len > 64 || log2len < 2)
    return -3;

  pmpcfg_csr = (0x3a0 + (n >> 2)) & ~1;
  pmpcfg_shift = (n & 7) << 3;

  pmpaddr_csr = 0x3b0 + n;

  prot &= ~((0x18UL));
  prot |= (log2len == 2) ? ((0x10UL)) : ((0x18UL));
  cfgmask = ~(0xffUL << pmpcfg_shift);
  pmpcfg = (csr_read_num(pmpcfg_csr) & cfgmask);
  pmpcfg |= ((prot << pmpcfg_shift) & ~cfgmask);

  if (log2len == 2) {
    pmpaddr = (addr >> 2);
  } else {
    if (log2len == 64) {
      pmpaddr = -1UL;
    } else {
      addrmask = (1UL << (log2len - 2)) - 1;
      pmpaddr = ((addr >> 2) & ~addrmask);
      pmpaddr |= (addrmask >> 1);
    }
  }

  csr_write_num(pmpaddr_csr, pmpaddr);
  csr_write_num(pmpcfg_csr, pmpcfg);

  return 0;
}

int pmp_get(unsigned int n, unsigned long *prot_out, unsigned long *addr_out,
            unsigned long *log2len) {
  int pmpcfg_csr, pmpcfg_shift, pmpaddr_csr;
  unsigned long cfgmask, pmpcfg, prot;
  unsigned long t1, addr, len;

  if (n >= 64 || !prot_out || !addr_out || !log2len)
    return -3;
  *prot_out = *addr_out = *log2len = 0;

  pmpcfg_csr = (0x3a0 + (n >> 2)) & ~1;
  pmpcfg_shift = (n & 7) << 3;

  pmpaddr_csr = 0x3b0 + n;

  cfgmask = (0xffUL << pmpcfg_shift);
  pmpcfg = csr_read_num(pmpcfg_csr) & cfgmask;
  prot = pmpcfg >> pmpcfg_shift;

  if ((prot & ((0x18UL))) == ((0x18UL))) {
    addr = csr_read_num(pmpaddr_csr);
    if (addr == -1UL) {
      addr = 0;
      len = 64;
    } else {
      t1 = ctz(~addr);
      addr = (addr & ~((1UL << t1) - 1)) << 2;
      len = (t1 + 2 + 1);
    }
  } else {
    addr = csr_read_num(pmpaddr_csr) << 2;
    len = 2;
  }

  *prot_out = prot;
  *addr_out = addr;
  *log2len = len;

  return 0;
}
