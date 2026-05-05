# Makefile for PSRAM behavioral model testbench (iverilog)
#
# Usage:
#   make           - run functional test (tb_psram)
#   make timing    - run timing verification (tb_psram_timing)
#   make wave      - open functional test waveform in gtkwave
#   make wave-t    - open timing test waveform in gtkwave
#   make clean     - remove generated files

IVL       = iverilog
VVP       = vvp
GTKWAVE   = gtkwave

SRC_DIR   = ../../perip/psram
PSRAM_SRC = $(SRC_DIR)/psram.v

IVL_FLAGS = -g2012 -Wall

# Functional test
TOP       = tb_psram
VCD_FILE  = $(TOP).vcd
VVP_FILE  = $(TOP).vvp
SRCS      = $(PSRAM_SRC) $(TOP).v

# Timing test
TIMING_TOP  = tb_psram_timing
TIMING_VCD  = $(TIMING_TOP).vcd
TIMING_VVP  = $(TIMING_TOP).vvp
TIMING_SRCS = $(PSRAM_SRC) $(TIMING_TOP).v

.PHONY: all run timing wave wave-t clean

all: run

$(VVP_FILE): $(SRCS)
	$(IVL) $(IVL_FLAGS) -o $(VVP_FILE) $(SRCS)

run: $(VVP_FILE)
	$(VVP) $(VVP_FILE)

$(TIMING_VVP): $(TIMING_SRCS)
	$(IVL) $(IVL_FLAGS) -o $(TIMING_VVP) $(TIMING_SRCS)

timing: $(TIMING_VVP)
	$(VVP) $(TIMING_VVP)

wave: $(VCD_FILE)
	$(GTKWAVE) $(VCD_FILE) &

wave-t: $(TIMING_VCD)
	$(GTKWAVE) $(TIMING_VCD) &

clean:
	rm -f $(VVP_FILE) $(VCD_FILE) $(TIMING_VVP) $(TIMING_VCD)
	rm -f tb_psram_min.vvp tb_psram_min.vcd
