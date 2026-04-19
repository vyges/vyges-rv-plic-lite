# Copyright 2026 Vyges.
# SPDX-License-Identifier: Apache-2.0
#
# cocotb Makefile for rv_plic_lite -- Verilator simulation
#
# Usage:
#   make                # run all tests
#   make waves          # open waveform viewer after a run
#   make clean          # cleanup sim artifacts

TOPLEVEL_LANG  = verilog
TOPLEVEL       = tb_rv_plic_lite
MODULE         = test_rv_plic_lite
SIM            = verilator

RTL_DIR        = ../../rtl
TB_DIR         = .

VERILOG_SOURCES = $(TB_DIR)/tb_rv_plic_lite.sv \
                  $(RTL_DIR)/rv_plic_lite.sv

COMPILE_ARGS   = -sv --timing
# Pre-existing style warnings in rv_plic_lite.sv -- int unsigned localparams
# initialised from 12'h constants, and a harmless unsigned compare. Silencing
# so cocotb/Verilator runs; fix opportunistically in a follow-up cleanup pass.
EXTRA_ARGS    += --trace-fst \
                 -Wno-WIDTHEXPAND \
                 -Wno-WIDTHTRUNC \
                 -Wno-UNSIGNED

include $(shell cocotb-config --makefiles)/Makefile.sim

.PHONY: waves clean-waves
waves:
	gtkwave dump.fst &

clean-waves:
	rm -f *.fst dump.*
