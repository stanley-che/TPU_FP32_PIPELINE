PRO_PATH ?= ./test/prog1/
SRC_NAME ?= main
ELF_NAME ?= main
SET_NAME ?= setup

SRC ?= $(PRO_PATH)$(SRC_NAME)
ELF ?= $(PRO_PATH)$(ELF_NAME)
SET ?= $(PRO_PATH)$(SET_NAME)

export CROSS_PREFIX ?= ~/riscv/bin/riscv32-unknown-elf-
export RISCV_AS ?= $(CROSS_PREFIX)as
export RISCV_LD ?= $(CROSS_PREFIX)ld
export RISCV_OBJDUMP ?= $(CROSS_PREFIX)objdump -xsd
export RISCV_OBJCOPY ?= $(CROSS_PREFIX)objcopy -O verilog

LDFILE := link.ld
CFLAGS := -march=rv32i -mabi=ilp32
LDFLAGS := -b elf32-littleriscv -T $(LDFILE)


.PHONY: all

all: build_mem build_dump 

# Transfer format (byte -> word) (.hex -> .mem)
build_mem: build_hex
	python3 bin2mem.py --bin $(ELF).hex

# Generate binary file (format: verilog byte by byte) (.elf -> .hex)
build_hex: build_elf
	$(RISCV_OBJCOPY) $(ELF).elf $(ELF).hex

# Generate Executable and Linkable Format (elf) file (.o .o .o ... -> .elf)
# Put setup first, then source file
build_elf: build_object
	$(RISCV_LD) $(LDFLAGS) $(SET).o $(SRC).o -o $(ELF).elf

# Generate object files (*.s -> *.o)
build_object :
	$(RISCV_AS) $(CFLAGS) $(SET).s -o $(SET).o
	$(RISCV_AS) $(CFLAGS) $(SRC).s -o $(SRC).o

# Disassemble for debugging (.elf -> .dump)
build_dump: 
	$(RISCV_OBJDUMP) $(ELF).elf > $(ELF).dump

.PHONY: clean

clean:
	rm -rf $(PRO_PATH)*.elf $(PRO_PATH)*.dump  $(ELF).hex  $(PRO_PATH)*.o $(PRO_PATH)*.mem
