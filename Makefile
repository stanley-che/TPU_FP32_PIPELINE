################################################################################
# TPU_FP32_PIPELINE Makefile
################################################################################

SHELL := /bin/bash

# ------------------------------------------------------------------------------
# Paths
# ------------------------------------------------------------------------------
ROOT_DIR      := $(abspath .)
SRC_DIR       := $(ROOT_DIR)/src
TEST_DIR      := $(ROOT_DIR)/test
BUILD_DIR     := $(ROOT_DIR)/build
RUN_DIR       := $(BUILD_DIR)/run
LOG_DIR       := $(BUILD_DIR)/log
FLIST_DIR     := $(BUILD_DIR)/filelist
SYN_DIR       := $(ROOT_DIR)/syn

# ------------------------------------------------------------------------------
# Tools
# ------------------------------------------------------------------------------
VCS           ?= vcs
VERDI         ?= verdi
URG           ?= urg
DC_SHELL      ?= dc_shell

# ------------------------------------------------------------------------------
# Top / TB
# ------------------------------------------------------------------------------
TOP_MODULE    ?= video_feature_sram_nb_attn_m5_top_fp32
TB_FILE       ?= $(TEST_DIR)/tb_video_feature_sram_nb_attn_m5_top_fp32.sv
TB_TOP        ?= tb_video_feature_sram_nb_attn_m5_top_fp32

GL_TB_FILE    ?= $(TB_FILE)
GL_TB_TOP     ?= $(TB_TOP)

SIMV          := $(RUN_DIR)/simv
SIMV_DAIDIR   := $(RUN_DIR)/simv.daidir

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
SEED          ?= 1
TIMESCALE     ?= 1ns/1ps
DUMP          ?= FSDB
DEBUG         ?= 1
UVM           ?= 0

# ------------------------------------------------------------------------------
# Gate / Synthesis
# ------------------------------------------------------------------------------
NETLIST       ?= $(SYN_DIR)/chip_syn.v
SDF           ?= $(SYN_DIR)/chip_syn.sdf
DC_SCRIPT     ?= $(ROOT_DIR)/script/synthesis.tcl
STDCELL_LIB   ?= /usr/cad/CBDK/Executable_Package/Collaterals/IP/stdcell/N16ADFP_StdCell/VERILOG/N16ADFP_StdCell.v

# ------------------------------------------------------------------------------
# DesignWare
# USE_DW=1 : require DW paths
# USE_DW=0 : skip DW file check
# ------------------------------------------------------------------------------
USE_DW        ?= 1

DW_FP_MULT    ?=
DW_FP_ADD     ?=

ifeq ($(USE_DW),1)
DW_FILES      := $(DW_FP_MULT) $(DW_FP_ADD)
else
DW_FILES      :=
endif

# ------------------------------------------------------------------------------
# Includes / Defines
# ------------------------------------------------------------------------------
DEFINES       :=
INCDIRS       := \
	+incdir+$(SRC_DIR)

ifeq ($(DEBUG),1)
VCS_DEBUG_OPT := -debug_access+all -kdb
else
VCS_DEBUG_OPT :=
endif

ifeq ($(DUMP),FSDB)
DEFINES += +define+FSDB
endif

ifeq ($(DUMP),VCD)
DEFINES += +define+VCD
endif

ifeq ($(UVM),1)
VCS_UVM_OPT := -ntb_opts uvm
else
VCS_UVM_OPT :=
endif

COMMON_VCS_OPTS := \
	-full64 \
	-sverilog \
	-timescale=$(TIMESCALE) \
	-lca \
	$(VCS_DEBUG_OPT) \
	$(VCS_UVM_OPT) \
	$(INCDIRS) \
	$(DEFINES)

# ------------------------------------------------------------------------------
# Source collection
# ------------------------------------------------------------------------------
RTL_SRCS := $(shell find $(SRC_DIR) \
	-type f \( -name "*.sv" -o -name "*.v" \) \
	-not -path "$(SRC_DIR)/EPU/*" | sort)

RTL_FLIST := $(FLIST_DIR)/rtl.f
GL_FLIST  := $(FLIST_DIR)/gate.f

# ------------------------------------------------------------------------------
# Default
# ------------------------------------------------------------------------------
.PHONY: all
all: rtl

# ------------------------------------------------------------------------------
# Directories
# ------------------------------------------------------------------------------
.PHONY: dirs
dirs:
	@mkdir -p $(BUILD_DIR) $(RUN_DIR) $(LOG_DIR) $(FLIST_DIR) $(SYN_DIR)

# ------------------------------------------------------------------------------
# Checks
# ------------------------------------------------------------------------------
.PHONY: check_tb
check_tb:
	@if [ ! -f "$(TB_FILE)" ]; then \
		echo "[ERROR] TB file not found: $(TB_FILE)"; \
		exit 1; \
	fi

.PHONY: check_gl_tb
check_gl_tb:
	@if [ ! -f "$(GL_TB_FILE)" ]; then \
		echo "[ERROR] Gate TB file not found: $(GL_TB_FILE)"; \
		exit 1; \
	fi

.PHONY: check_dw
check_dw:
ifeq ($(USE_DW),1)
	@if [ -z "$(strip $(DW_FP_MULT))" ]; then \
		echo "[ERROR] USE_DW=1 but DW_FP_MULT is empty"; \
		echo "Example:"; \
		echo "  make rtl USE_DW=1 DW_FP_MULT=/path/DW_fp_mult.v DW_FP_ADD=/path/DW_fp_add.v"; \
		exit 1; \
	fi
	@if [ -z "$(strip $(DW_FP_ADD))" ]; then \
		echo "[ERROR] USE_DW=1 but DW_FP_ADD is empty"; \
		echo "Example:"; \
		echo "  make rtl USE_DW=1 DW_FP_MULT=/path/DW_fp_mult.v DW_FP_ADD=/path/DW_fp_add.v"; \
		exit 1; \
	fi
	@if [ ! -f "$(DW_FP_MULT)" ]; then \
		echo "[ERROR] DW_fp_mult.v not found: $(DW_FP_MULT)"; \
		exit 1; \
	fi
	@if [ ! -f "$(DW_FP_ADD)" ]; then \
		echo "[ERROR] DW_fp_add.v not found: $(DW_FP_ADD)"; \
		exit 1; \
	fi
else
	@echo "[INFO] USE_DW=0, skip DW check"
endif

# ------------------------------------------------------------------------------
# Filelists
# ------------------------------------------------------------------------------
$(RTL_FLIST): dirs
	@echo "[INFO] Generate RTL filelist: $@"
	@rm -f $@
	@for f in $(RTL_SRCS); do echo $$f >> $@; done
	@echo "[INFO] RTL file count = $$(wc -l < $@)"

$(GL_FLIST): dirs
	@echo "[INFO] Generate gate filelist: $@"
	@rm -f $@
	@if [ ! -f "$(STDCELL_LIB)" ]; then \
		echo "[ERROR] stdcell lib not found: $(STDCELL_LIB)"; \
		exit 1; \
	fi
	@if [ ! -f "$(NETLIST)" ]; then \
		echo "[ERROR] netlist not found: $(NETLIST)"; \
		exit 1; \
	fi
	@echo "$(STDCELL_LIB)" >> $@
	@echo "$(NETLIST)" >> $@
	@echo "[INFO] Gate file count = $$(wc -l < $@)"

# ------------------------------------------------------------------------------
# Info
# ------------------------------------------------------------------------------
.PHONY: info
info:
	@echo "ROOT_DIR   = $(ROOT_DIR)"
	@echo "TB_FILE    = $(TB_FILE)"
	@echo "TB_TOP     = $(TB_TOP)"
	@echo "USE_DW     = $(USE_DW)"
	@echo "DW_FP_MULT = $(DW_FP_MULT)"
	@echo "DW_FP_ADD  = $(DW_FP_ADD)"
	@echo "NETLIST    = $(NETLIST)"
	@echo "SDF        = $(SDF)"

# ------------------------------------------------------------------------------
# RTL compile
# ------------------------------------------------------------------------------
.PHONY: rtl
rtl: $(RTL_FLIST) dirs check_tb check_dw
	@echo "[INFO] RTL compile..."
	$(VCS) $(COMMON_VCS_OPTS) \
	$(DW_FILES) \
	-file $(RTL_FLIST) \
	$(TB_FILE) \
	-top $(TB_TOP) \
	-o $(SIMV) \
	-l $(LOG_DIR)/rtl_compile.log

# ------------------------------------------------------------------------------
# RTL run
# ------------------------------------------------------------------------------
.PHONY: run
run: rtl
	@echo "[INFO] RTL run..."
	cd $(RUN_DIR) && ./simv +ntb_random_seed=$(SEED) -l $(LOG_DIR)/rtl_run.log

# ------------------------------------------------------------------------------
# Synthesis
# ------------------------------------------------------------------------------
.PHONY: synth
synth: dirs
	@if [ ! -f "$(DC_SCRIPT)" ]; then \
		echo "[ERROR] synthesis script not found: $(DC_SCRIPT)"; \
		exit 1; \
	fi
	cd $(ROOT_DIR) && $(DC_SHELL) -f $(DC_SCRIPT) | tee $(LOG_DIR)/dc_synth.log

# ------------------------------------------------------------------------------
# Gate compile
# ------------------------------------------------------------------------------
.PHONY: gate
gate: $(GL_FLIST) dirs check_gl_tb check_dw
	@echo "[INFO] Gate compile..."
	$(VCS) $(COMMON_VCS_OPTS) \
	$(DW_FILES) \
	-neg_tchk \
	-negdelay \
	+define+GATE_SIM \
	-file $(GL_FLIST) \
	$(GL_TB_FILE) \
	-top $(GL_TB_TOP) \
	-o $(SIMV) \
	-l $(LOG_DIR)/gate_compile.log

# ------------------------------------------------------------------------------
# Gate run
# ------------------------------------------------------------------------------
.PHONY: gate_run
gate_run: gate
	@echo "[INFO] Gate run..."
ifeq ($(strip $(SDF)),)
	cd $(RUN_DIR) && ./simv +ntb_random_seed=$(SEED) -l $(LOG_DIR)/gate_run.log
else
	@if [ ! -f "$(SDF)" ]; then \
		echo "[ERROR] SDF not found: $(SDF)"; \
		exit 1; \
	fi
	cd $(RUN_DIR) && ./simv +ntb_random_seed=$(SEED) \
		+sdfverbose \
		-sdf max:$(GL_TB_TOP).dut:$(SDF) \
		-l $(LOG_DIR)/gate_run.log
endif

# ------------------------------------------------------------------------------
# Verdi
# ------------------------------------------------------------------------------
.PHONY: verdi
verdi:
	@if [ -d "$(SIMV_DAIDIR)" ]; then \
		cd $(RUN_DIR) && $(VERDI) -dbdir simv.daidir & \
	else \
		echo "[WARN] simv.daidir not found"; \
	fi

# ------------------------------------------------------------------------------
# Coverage
# ------------------------------------------------------------------------------
.PHONY: cov
cov:
	@if [ -d "$(RUN_DIR)/simv.vdb" ]; then \
		$(URG) -dir $(RUN_DIR)/simv.vdb -report $(BUILD_DIR)/cov_report; \
	else \
		echo "[WARN] simv.vdb not found"; \
	fi

# ------------------------------------------------------------------------------
# Clean
# ------------------------------------------------------------------------------
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -rf csrc ucli.key novas* verdiLog DVEfiles simv* inter.fsdb *.log *.vpd *.vcd *.fsdb
