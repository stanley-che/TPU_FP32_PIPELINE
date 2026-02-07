`ifndef FEAT_SRAM_BURST_TOP_SV
`define FEAT_SRAM_BURST_TOP_SV
`include "./src/AMOLED/feature_sram/burst_engine.sv"
`include "./src/AMOLED/feature_sram/rd_assemble_fifo.sv"

`timescale 1ns/1ps
`default_nettype none


module feat_sram_burst_top #(
  parameter int unsigned SRAM_BUS_W      = 32,
  parameter int unsigned MEM_AW          = 18,
  parameter int unsigned FEAT_W          = 256,
  parameter int unsigned WORDS_PER_FEAT  = 8,
  parameter int unsigned TAG_W           = 16,

  parameter int unsigned REQ_DEPTH       = 2,
  parameter int unsigned META_DEPTH      = 16,
  parameter int unsigned BEAT_DEPTH      = 16,

  parameter int unsigned RD_FIFO_DEPTH   = 2   // rd_assemble_fifo output fifo depth
)(
  input  logic clk,
  input  logic rst,

  // =============================
  // Request in (same as burst_engine)
  // =============================
  input  logic               req_valid,
  output logic               req_ready,
  input  logic               req_is_wr,
  input  logic [FEAT_W-1:0]  req_wdata,
  input  logic [TAG_W-1:0]   req_tag,
  input  logic [MEM_AW-1:0]  req_addr,

  // =============================
  // SRAM command out
  // =============================
  output logic                  mem_req_valid,
  input  logic                  mem_req_ready,
  output logic                  mem_req_is_wr,
  output logic [MEM_AW-1:0]     mem_req_addr,
  output logic [SRAM_BUS_W-1:0] mem_req_wdata,
  output logic [SRAM_BUS_W/8-1:0] mem_req_wmask,

  // =============================
  // SRAM read return in
  // =============================
  input  logic                  mem_rvalid,
  input  logic [SRAM_BUS_W-1:0] mem_rdata,

  // =============================
  // Feature out (READ only)
  // =============================
  output logic               feat_out_valid,
  input  logic               feat_out_ready,
  output logic [TAG_W-1:0]   feat_out_tag,
  output logic [FEAT_W-1:0]  feat_out_data
);

  localparam int unsigned BEAT_IDX_W =
    (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);

  // -----------------------------
  // internal beat wires
  // -----------------------------
  logic                  beat_valid;
  logic                  beat_ready;
  logic [TAG_W-1:0]      beat_tag;
  logic [BEAT_IDX_W-1:0] beat_idx;
  logic [SRAM_BUS_W-1:0] beat_data;

  // -----------------------------
  // burst_engine
  // -----------------------------
  burst_engine #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .MEM_AW(MEM_AW),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(TAG_W),
    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH)
  ) u_burst (
    .clk(clk),
    .rst(rst),

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .req_wdata(req_wdata),
    .req_tag(req_tag),
    .req_addr(req_addr),

    .mem_req_valid(mem_req_valid),
    .mem_req_ready(mem_req_ready),
    .mem_req_is_wr(mem_req_is_wr),
    .mem_req_addr(mem_req_addr),
    .mem_req_wdata(mem_req_wdata),
    .mem_req_wmask(mem_req_wmask),

    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),

    .beat_out_valid(beat_valid),
    .beat_out_ready(beat_ready),  // driven by assembler
    .beat_out_data(beat_data),
    .beat_out_tag(beat_tag),
    .beat_out_idx(beat_idx)
  );

  // -----------------------------
  // rd_assemble_fifo
  // -----------------------------
  rd_assemble_fifo #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(TAG_W),
    .FIFO_DEPTH(RD_FIFO_DEPTH)
  ) u_rd_asm (
    .clk(clk),
    .rst(rst),

    .beat_valid(beat_valid),
    .beat_ready(beat_ready),
    .beat_tag(beat_tag),
    .beat_idx(beat_idx),
    .beat_data(beat_data),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data)
  );

endmodule

`default_nettype wire
`endif
