`ifndef FEATURE_SRAM_SV
`define FEATURE_SRAM_SV

`include "./src/AMOLED/feature_sram/tile_feat_sram_burst_system.sv"
`include "./src/AMOLED/feature_sram/sram_phy_wrap.sv"

`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// tile_feat_sram_burst_system_with_phy
// - 把 tile_feat_sram_burst_system + sram_phy_wrap 直接接在一起
// - 外部只看到 tile wr/rd + feat_out
// ------------------------------------------------------------
module feature_sram #(
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

  // FIFOs (burst)
  parameter int unsigned REQ_DEPTH       = 2,
  parameter int unsigned META_DEPTH      = 16,
  parameter int unsigned BEAT_DEPTH      = 16,
  parameter int unsigned RD_FIFO_DEPTH   = 2,

  // SRAM PHY
  parameter int unsigned RD_LAT          = 2,
  parameter int unsigned MEM_WORDS       = (1 << MEM_AW)
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
  // Feature out (READ only)
  // =============================
  output logic                      feat_out_valid,
  input  logic                      feat_out_ready,
  output logic [tag_w-1:0]          feat_out_tag,
  output logic [FEAT_W-1:0]         feat_out_data
);

  // ------------------------------------------------------------
  // Internal SRAM cmd / return wires
  // ------------------------------------------------------------
  logic                      mem_req_valid;
  logic                      mem_req_ready;
  logic                      mem_req_is_wr;
  logic [MEM_AW-1:0]         mem_req_addr;
  logic [sram_bus-1:0]       mem_req_wdata;
  logic [sram_bus/8-1:0]     mem_req_wmask;

  logic                      mem_rvalid;
  logic [sram_bus-1:0]       mem_rdata;

  // ------------------------------------------------------------
  // 1) tile -> burst system
  // ------------------------------------------------------------
  tile_feat_sram_burst_system #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp),

    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TOTAL_WORDS(TOTAL_WORDS),
    .MEM_AW(MEM_AW),

    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH)
  ) u_sys (
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

  // ------------------------------------------------------------
  // 2) SRAM PHY WRAP (reg-mem model now, later換macro/BRAM)
  // ------------------------------------------------------------
  logic phy_cmd_ready;

  sram_phy_wrap #(
    .SRAM_BUS_W(sram_bus),
    .MEM_AW(MEM_AW),
    .RD_LAT(RD_LAT),
    .MEM_WORDS(MEM_WORDS)
  ) u_phy (
    .clk(clk),

    .mem_cmd_valid(mem_req_valid),
    .mem_cmd_ready(phy_cmd_ready),
    .mem_cmd_we   (mem_req_is_wr),
    .mem_cmd_addr (mem_req_addr),
    .mem_cmd_wdata(mem_req_wdata),
    .mem_cmd_wmask(mem_req_wmask),

    .mem_rvalid(mem_rvalid),
    .mem_rdata (mem_rdata)
  );

  // burst system expects mem_req_ready
  assign mem_req_ready = phy_cmd_ready;

endmodule

`default_nettype wire
`endif
