COMPILER_PREFIX := riscv64-unknown-elf-
CC      := $(COMPILER_PREFIX)gcc          
OBJDUMP := $(COMPILER_PREFIX)objdump      
OBJCOPY := $(COMPILER_PREFIX)objcopy      

OUTPUT_DIR := output

MACHINE ?= MACHINE_VIRT

SRCS := $(wildcard src/*.c src/*.S src/opensbi_helper/*.c)
IGNS :=                                  
SRCS := $(filter-out $(IGNS),$(SRCS))    
OBJS := $(foreach src,$(SRCS),$(addprefix $(OUTPUT_DIR)/,$(dir $(src))$(basename $(notdir $(src))).o))  
DEPS := $(OBJS:.o=.d)                    


target := hello
target := $(OUTPUT_DIR)/$(target).elf

CFLAGS  := -static -O3 -g -ffunction-sections -fdata-sections -finline-functions -finline-small-functions \
			-mabi=lp64d -march=rv64gc -mstrict-align -mno-save-restore \
			-Wall -Wno-main -fno-builtin-printf -fno-stack-protector -fomit-frame-pointer -foptimize-sibling-calls \
			-save-temps=obj -fPIC -fno-semantic-interposition -mcmodel=medany -Wno-builtin-declaration-mismatch \
			-specs=nosys.specs -D$(MACHINE)

LDFLAGS := -nostartfiles -static
LDSCRIPT := link.ld.s
LDSCRIPT := $(if $(findstring .ld.s,$(LDSCRIPT)),$(OUTPUT_DIR)/$(LDSCRIPT:.ld.s=.ld),$(LDSCRIPT))
LDSCRIPT_cmd := $(if $(LDSCRIPT),-T$(LDSCRIPT),)
LDFLAGS += -Wl,--cref,-Map=$(target:.elf=.map) $(LDSCRIPT_cmd)
LDLIBS  := 

INC_DIRS := ./ ./include ./opensbi/include                

CFLAGS += $(foreach dr,$(INC_DIRS),-I$(abspath $(shell echo $(dr))))

ifeq (,$(findstring j,$(MAKEFLAGS)))
	MAKEFLAGS += -j
endif

COMPILE_COMMANDS_GENERATOR := intercept-build-14
all:
	command -v $(COMPILE_COMMANDS_GENERATOR) > /dev/null && $(COMPILE_COMMANDS_GENERATOR) \
		--cdb $(OUTPUT_DIR)/compile_commands.json --append make _all && \
		sed -i 's#\bcc\b#$(CC)#' $(OUTPUT_DIR)/compile_commands.json \
		|| make _all

_all: machine_info $(target) $(target:.elf=.bin) $(target:.elf=.txt)

machine_info:
	machine=$(MACHINE) && echo "machine: $${machine#MACHINE_}"

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(target): $(OBJS) $(LDSCRIPT) | $(OUTPUT_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@


$(OUTPUT_DIR)/src/main.o: EXTRA_CFLAGS :=

DEPFLAGS = -MMD -MP -MF $(@:.o=.d)


$(OUTPUT_DIR)/%.o: %.c Makefile | $(OUTPUT_DIR)
	echo "CC $<"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(DEPFLAGS) $(EXTRA_CFLAGS) -c $< -o $@


$(OUTPUT_DIR)/%.o: %.S Makefile | $(OUTPUT_DIR)
	echo "AS $<"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(DEPFLAGS) $(EXTRA_CFLAGS) -c $< -o $@


-include $(DEPS)


%.bin: %.elf
	$(OBJCOPY) -S --set-section-flags .bss=alloc,contents -O binary $< $@


%.txt: %.elf
	$(OBJDUMP) -d $< > $@


$(OUTPUT_DIR)/%.ld: %.ld.s | $(OUTPUT_DIR)
	echo "gen LDSCRIPT $<  --> $@"
	cat $< | $(CC) $(CFLAGS) -E -xc - | grep -v '^#' > $@




.PHONY: clean
clean:
	rm -rf $(OUTPUT_DIR)


.SILENT:
