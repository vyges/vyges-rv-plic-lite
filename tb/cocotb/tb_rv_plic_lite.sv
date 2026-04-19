// Copyright 2026 Vyges.
// SPDX-License-Identifier: Apache-2.0
//
// Cocotb testbench wrapper for rv_plic_lite.
//
// rv_plic_lite's I/O is tlul_pkg struct-based. Verilator/cocotb wants flat
// signals, so this wrapper:
//   1. Provides a minimal test-local tlul_pkg (just the types/opcodes the
//      DUT uses) so we can compile without pulling in opentitan-tlul.
//   2. Stubs tlul_rsp_intg_gen as a straight passthrough (we are testing the
//      plic response FSM, not the integrity-signing module).
//   3. Instantiates the DUT with struct-typed nets built from flat ports,
//      and mirrors the struct response back out to flat ports.

`ifndef TB_RV_PLIC_LITE_SV
`define TB_RV_PLIC_LITE_SV

// ──────────────────────────────────────────────────────────────────────────
// Minimal tlul_pkg for standalone simulation
// ──────────────────────────────────────────────────────────────────────────
package tlul_pkg;
  typedef enum logic [2:0] {
    PutFullData    = 3'h0,
    PutPartialData = 3'h1,
    Get            = 3'h4
  } tl_a_op_e;

  typedef enum logic [2:0] {
    AccessAck     = 3'h0,
    AccessAckData = 3'h1
  } tl_d_op_e;

  typedef struct packed {
    logic        a_valid;
    tl_a_op_e    a_opcode;
    logic [2:0]  a_param;
    logic [2:0]  a_size;
    logic [7:0]  a_source;
    logic [31:0] a_address;
    logic [3:0]  a_mask;
    logic [31:0] a_data;
    logic [15:0] a_user;
    logic        d_ready;
  } tl_h2d_t;

  typedef struct packed {
    logic        d_valid;
    tl_d_op_e    d_opcode;
    logic [2:0]  d_param;
    logic [2:0]  d_size;
    logic [7:0]  d_source;
    logic [7:0]  d_sink;
    logic [31:0] d_data;
    logic [15:0] d_user;
    logic        d_error;
    logic        a_ready;
  } tl_d2h_t;
endpackage

// Passthrough stub for tlul_rsp_intg_gen -- we are not testing integrity
// here, just the plic response FSM. In production this module signs
// rsp_intg + data_intg; for simulation we just forward the struct.
module tlul_rsp_intg_gen #(
  parameter int unsigned EnableRspIntgGen  = 1,
  parameter int unsigned EnableDataIntgGen = 1
) (
  input  tlul_pkg::tl_d2h_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o
);
  assign tl_o = tl_i;
endmodule

// ──────────────────────────────────────────────────────────────────────────
// Flat-signal top wrapper for cocotb
// ──────────────────────────────────────────────────────────────────────────
module tb_rv_plic_lite #(
  parameter int unsigned NUM_SOURCES = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Flat TL-UL a-channel in
  input  logic        a_valid_i,
  input  logic [2:0]  a_opcode_i,
  input  logic [2:0]  a_size_i,
  input  logic [7:0]  a_source_i,
  input  logic [31:0] a_address_i,
  input  logic [3:0]  a_mask_i,
  input  logic [31:0] a_data_i,

  // Flat TL-UL d-channel out
  output logic        d_valid_o,
  output logic [2:0]  d_opcode_o,
  output logic [2:0]  d_size_o,
  output logic [7:0]  d_source_o,
  output logic [31:0] d_data_o,
  output logic        d_error_o,
  output logic        a_ready_o,

  // Interrupt sources (active-high, level-sensitive)
  input  logic [NUM_SOURCES-1:0] intr_src_i,

  // Aggregated interrupt out
  output logic        irq_o
);

  tlul_pkg::tl_h2d_t dut_tl_i;
  tlul_pkg::tl_d2h_t dut_tl_o;

  always_comb begin
    dut_tl_i.a_valid   = a_valid_i;
    dut_tl_i.a_opcode  = tlul_pkg::tl_a_op_e'(a_opcode_i);
    dut_tl_i.a_param   = '0;
    dut_tl_i.a_size    = a_size_i;
    dut_tl_i.a_source  = a_source_i;
    dut_tl_i.a_address = a_address_i;
    dut_tl_i.a_mask    = a_mask_i;
    dut_tl_i.a_data    = a_data_i;
    dut_tl_i.a_user    = '0;
    dut_tl_i.d_ready   = 1'b1;  // host is always ready in this TB
  end

  assign d_valid_o  = dut_tl_o.d_valid;
  assign d_opcode_o = dut_tl_o.d_opcode;
  assign d_size_o   = dut_tl_o.d_size;
  assign d_source_o = dut_tl_o.d_source;
  assign d_data_o   = dut_tl_o.d_data;
  assign d_error_o  = dut_tl_o.d_error;
  assign a_ready_o  = dut_tl_o.a_ready;

  rv_plic_lite #(
    .NUM_SOURCES (NUM_SOURCES)
  ) u_dut (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .tl_i       (dut_tl_i),
    .tl_o       (dut_tl_o),
    .intr_src_i (intr_src_i),
    .irq_o      (irq_o)
  );

endmodule

`endif // TB_RV_PLIC_LITE_SV
