// Copyright 2026 Vyges Inc.
// SPDX-License-Identifier: Apache-2.0
//
// rv_plic_lite — Lightweight RISC-V Platform-Level Interrupt Controller
// Native TL-UL slave interface, up to 32 sources, single hart target.

`ifndef RV_PLIC_LITE_SV
`define RV_PLIC_LITE_SV

module rv_plic_lite
  import tlul_pkg::*;
#(
  parameter int unsigned NUM_SOURCES = 32
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // TL-UL device port
  input  tlul_pkg::tl_h2d_t      tl_i,
  output tlul_pkg::tl_d2h_t      tl_o,

  // Interrupt sources (active-high, level-sensitive)
  input  logic [NUM_SOURCES-1:0]  intr_src_i,

  // Interrupt output to hart
  output logic                    irq_o
);

  // ---------------------------------------------------------------------------
  // Register map offsets
  // ---------------------------------------------------------------------------
  // 0x000 .. 0x07C  PRIO[0..31]       RW  3-bit priority per source
  // 0x080           PENDING           RO  32-bit pending bitmap
  // 0x100           ENABLE            RW  32-bit enable bitmap
  // 0x200           THRESHOLD         RW  3-bit priority threshold
  // 0x204           CLAIM_COMPLETE    RW  read=claim, write=complete

  localparam int unsigned ADDR_PRIO_BASE       = 12'h000;
  localparam int unsigned ADDR_PRIO_END        = 12'h07C; // inclusive
  localparam int unsigned ADDR_PENDING         = 12'h080;
  localparam int unsigned ADDR_ENABLE          = 12'h100;
  localparam int unsigned ADDR_THRESHOLD       = 12'h200;
  localparam int unsigned ADDR_CLAIM_COMPLETE  = 12'h204;

  // ---------------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------------
  logic [2:0]  prio    [NUM_SOURCES];
  logic [31:0] pending;
  logic [31:0] enable;
  logic [2:0]  threshold;

  // Claimed bits — track which sources are currently claimed (in-service).
  logic [31:0] claimed;

  // ---------------------------------------------------------------------------
  // Pending logic — a source is pending when its input is asserted and it has
  // not yet been claimed.
  // ---------------------------------------------------------------------------
  always_comb begin
    pending = '0;
    for (int unsigned i = 0; i < NUM_SOURCES; i++) begin
      pending[i] = intr_src_i[i] & ~claimed[i];
    end
  end

  // ---------------------------------------------------------------------------
  // Priority arbiter — find the highest-priority pending+enabled source whose
  // priority exceeds the threshold.  Source 0 is reserved (no interrupt) per
  // the RISC-V PLIC spec, so valid IDs are 1..NUM_SOURCES-1 when NUM_SOURCES
  // includes ID 0 as "no interrupt".  For simplicity we treat all NUM_SOURCES
  // entries uniformly; software should keep prio[0] = 0.
  // ---------------------------------------------------------------------------
  logic [4:0]  best_id;
  logic [2:0]  best_prio;
  logic        irq_valid;

  always_comb begin
    best_id   = '0;
    best_prio = '0;
    irq_valid = 1'b0;

    for (int unsigned i = 0; i < NUM_SOURCES; i++) begin
      if (pending[i] && enable[i] && (prio[i] > threshold) && (prio[i] > best_prio)) begin
        best_id   = 5'(i);
        best_prio = prio[i];
        irq_valid = 1'b1;
      end
    end
  end

  assign irq_o = irq_valid;

  // ---------------------------------------------------------------------------
  // TL-UL bus logic -- registered (1-cycle) response
  //
  // Originally this slave asserted d_valid combinationally in the same
  // cycle as a_valid (0-cycle response). The TL-UL spec allows it, but
  // the Vyges-generated xbar's OR-mux response path does not tolerate
  // same-cycle a_valid/d_valid from a slave on back-to-back CPU writes
  // (first write completes, second stalls -- confirmed on FPGA bring-up:
  // Ibex hung on PRIO[1] write inside a 14-iter for-loop in plic_init()).
  // Matching spi-host-lite's registered pattern resolves it -- response
  // arrives one cycle after the request, through a clean flip-flop.
  // ---------------------------------------------------------------------------
  logic        tl_req;
  logic        tl_we;
  logic [31:0] tl_addr;
  logic [31:0] tl_wdata;
  logic [31:0] tl_rdata;
  logic        tl_err;

  assign tl_req   = tl_i.a_valid;
  assign tl_we    = (tl_i.a_opcode == PutFullData) || (tl_i.a_opcode == PutPartialData);
  assign tl_addr  = {20'b0, tl_i.a_address[11:0]}; // use lower 12 bits
  assign tl_wdata = tl_i.a_data;

  // Registered response state
  logic        rsp_valid;
  logic [31:0] rsp_rdata;
  logic        rsp_error;
  logic [7:0]  rsp_source_q;
  logic [2:0]  rsp_size_q;
  logic        rsp_write_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rsp_valid    <= 1'b0;
      rsp_rdata    <= '0;
      rsp_error    <= 1'b0;
      rsp_source_q <= '0;
      rsp_size_q   <= '0;
      rsp_write_q  <= 1'b0;
    end else begin
      rsp_valid    <= tl_req;
      rsp_rdata    <= tl_rdata;
      rsp_error    <= tl_err;
      rsp_source_q <= tl_i.a_source;
      rsp_size_q   <= tl_i.a_size;
      rsp_write_q  <= tl_we;
    end
  end

  // Pre-integrity response; tlul_rsp_intg_gen below signs rsp_intg +
  // data_intg so CPU-side tlul_rsp_intg_chk (always-on in opentitan-rv-
  // core-ibex) accepts this slave's d-channel on a signed TL-UL domain.
  tlul_pkg::tl_d2h_t tl_o_pre;
  assign tl_o_pre.d_valid  = rsp_valid;
  assign tl_o_pre.a_ready  = 1'b1;
  assign tl_o_pre.d_opcode = rsp_write_q ? AccessAck : AccessAckData;
  assign tl_o_pre.d_param  = '0;
  assign tl_o_pre.d_size   = rsp_size_q;
  assign tl_o_pre.d_source = rsp_source_q;
  assign tl_o_pre.d_sink   = '0;
  assign tl_o_pre.d_data   = rsp_rdata;
  assign tl_o_pre.d_user   = '0;
  assign tl_o_pre.d_error  = rsp_error;

  tlul_rsp_intg_gen #(
    .EnableRspIntgGen  (1),
    .EnableDataIntgGen (1)
  ) u_rsp_intg_gen (
    .tl_i (tl_o_pre),
    .tl_o (tl_o)
  );

  // ---------------------------------------------------------------------------
  // Read path (combinational)
  // ---------------------------------------------------------------------------
  always_comb begin
    tl_rdata = '0;
    tl_err   = 1'b0;

    if (tl_req) begin
      unique casez (tl_addr[11:0])
        // Priority registers 0x000 – 0x07C
        12'h0??: begin
          if (tl_addr[11:0] <= ADDR_PRIO_END[11:0]) begin
            tl_rdata = {29'b0, prio[tl_addr[6:2]]};
          end else if (tl_addr[11:0] == ADDR_PENDING[11:0]) begin
            tl_rdata = pending;
          end else begin
            tl_err = 1'b1;
          end
        end
        12'h100: tl_rdata = enable;
        12'h200: tl_rdata = {29'b0, threshold};
        12'h204: tl_rdata = irq_valid ? {27'b0, best_id} : 32'b0;
        default: tl_err = 1'b1;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Write path + claimed tracking (sequential)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int unsigned i = 0; i < NUM_SOURCES; i++) begin
        prio[i] <= 3'b0;
      end
      enable    <= '0;
      threshold <= '0;
      claimed   <= '0;
    end else begin
      // --- Claim on read of CLAIM_COMPLETE register ---
      if (tl_req && !tl_we && (tl_addr[11:0] == ADDR_CLAIM_COMPLETE[11:0]) && irq_valid) begin
        claimed[best_id] <= 1'b1;
      end

      // --- Write handling ---
      if (tl_req && tl_we) begin
        unique casez (tl_addr[11:0])
          12'h0??: begin
            if (tl_addr[11:0] <= ADDR_PRIO_END[11:0]) begin
              prio[tl_addr[6:2]] <= tl_wdata[2:0];
            end
          end
          // PENDING (0x080) is read-only — writes are silently ignored
          12'h100: enable    <= tl_wdata;
          12'h200: threshold <= tl_wdata[2:0];
          12'h204: begin
            // Complete: clear claimed bit for the written source ID
            if (tl_wdata[4:0] < NUM_SOURCES[4:0]) begin
              claimed[tl_wdata[4:0]] <= 1'b0;
            end
          end
          default: ; // ignore
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Assertions
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  initial begin
    assert (NUM_SOURCES > 0 && NUM_SOURCES <= 32)
      else $fatal(1, "NUM_SOURCES must be in [1,32]");
  end
  // synthesis translate_on

endmodule

`endif // RV_PLIC_LITE_SV
