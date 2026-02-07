`ifndef TILE_FEAT_SRAM_BURST_SYSTEM_SV
`define TILE_FEAT_SRAM_BURST_SYSTEM_SV

`include "./src/AMOLED/feature_sram/burst_req_if_tileaddr_wrap.sv"
`include "./src/AMOLED/feature_sram/feat_sram_burst_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tile_feat_sram_burst_system #(
  parameter int unsigned tile_x   = 320,
  parameter int unsigned tile_y   = 180,
  parameter int unsigned sram_bus = 32,
  parameter int unsigned feat_dim = 8,
  parameter int unsigned elen_W   = 32,
  parameter int unsigned tag_w    = 16,
  parameter bit          isclamp  = 1'b0,

  // derived
  parameter int unsigned FEAT_W          = feat_dim * elen_W,
  parameter int unsigned WORDS_PER_FEAT  = (FEAT_W + sram_bus - 1) / sram_bus,
  parameter int unsigned TOTAL_WORDS     = tile_x * tile_y * WORDS_PER_FEAT,
  parameter int unsigned MEM_AW          = (TOTAL_WORDS <= 1) ? 1 : $clog2(TOTAL_WORDS),

  // FIFOs
  parameter int unsigned REQ_DEPTH       = 2,
  parameter int unsigned META_DEPTH      = 16,
  parameter int unsigned BEAT_DEPTH      = 16,
  parameter int unsigned RD_FIFO_DEPTH   = 2
)(
  input  logic clk,
  input  logic rst,

  // =============================
  // Tile write req
  // =============================
  input  logic                      valid_wr,
  output logic                      ready_wr,
  input  logic [$clog2(tile_y)-1:0] tile_i_wr,
  input  logic [$clog2(tile_x)-1:0] tile_j_wr,
  input  logic [FEAT_W-1:0]         feat_in,

  // =============================
  // Tile read req
  // =============================
  input  logic                      valid_rd,
  output logic                      ready_rd,
  input  logic [$clog2(tile_y)-1:0] tile_i_rd,
  input  logic [$clog2(tile_x)-1:0] tile_j_rd,
  input  logic [tag_w-1:0]          tag_rd,

  // =============================
  // SRAM command out
  // =============================
  output logic                      mem_req_valid,
  input  logic                      mem_req_ready,
  output logic                      mem_req_is_wr,
  output logic [MEM_AW-1:0]         mem_req_addr,
  output logic [sram_bus-1:0]       mem_req_wdata,
  output logic [sram_bus/8-1:0]     mem_req_wmask,

  // =============================
  // SRAM read return in
  // =============================
  input  logic                      mem_rvalid,
  input  logic [sram_bus-1:0]       mem_rdata,

  // =============================
  // Feature out (READ only)
  // =============================
  output logic                      feat_out_valid,
  input  logic                      feat_out_ready,
  output logic [tag_w-1:0]          feat_out_tag,
  output logic [FEAT_W-1:0]         feat_out_data
);

  // -----------------------------------------
  // 1) wrap: tile -> burst req
  // -----------------------------------------
  logic                 req_valid;
  logic                 req_ready;
  logic                 req_is_wr;
  logic [MEM_AW-1:0]    req_addr;
  logic [FEAT_W-1:0]    req_wdata;
  logic [tag_w-1:0]     req_tag;

  burst_req_if_tileaddr_wrap #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp)
  ) u_if (
    .clk(clk),
    .rst(rst),

    .valid_wr(valid_wr),
    .ready_wr(ready_wr),
    .tile_i_wr(tile_i_wr),
    .tile_j_wr(tile_j_wr),
    .feat_in(feat_in),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .sram_addr(req_addr),
    .sram_wdata(req_wdata),
    .sram_tag(req_tag)
  );

  // -----------------------------------------
  // 2) burst + read assemble
  // -----------------------------------------
  feat_sram_burst_top #(
    .SRAM_BUS_W(sram_bus),
    .MEM_AW(MEM_AW),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(tag_w),
    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH)
  ) u_burst_top (
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

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data)
  );

endmodule

`default_nettype wire
`endif
