// ============================================================
// tile_nb_feature_fetch_3x3_bundle9_top_inst.sv  (INSTANCE-ONLY INTEGRATION)
// - Integrate by instantiating ONLY:
//   A) tile_nb_feature_fetch_3x3_top_inst   : center -> 9 feat tokens
//   B) nb_feat_bundle_assembler_9           : 9 tokens -> kv0..kv8 bundle
//
// External behavior:
// - center_valid/ready accepts a center tile request
// - SRAM read cmd/return interface same as fetch_top_inst
// - Downstream sees out_valid/out_ready with kv0..kv8/q_vec/kv_bus/out_group_tag
//
// Notes:
// - fetch.feat_ready is driven by assembler.feat_ready -> full backpressure propagation
// - fetch.sched_done_pulse pulses when 9th FEATURE token is CONSUMED by assembler
// - bundle_done_pulse pulses when the final bundle is CONSUMED (out_fire)
// ============================================================

`ifndef TILE_NB_FEATURE_FETCH_3X3_BUNDLE9_TOP_INST_SV
`define TILE_NB_FEATURE_FETCH_3X3_BUNDLE9_TOP_INST_SV

`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_feature_fetch_3x3_top_inst.sv"
`include "./src/AMOLED/tile_neighborhood_fetch/nb_feat_bundle_assembler_9.sv"

`timescale 1ns/1ps
`default_nettype none

module tile_nb_feature_fetch_3x3_bundle9_top_inst #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,

  parameter int unsigned TAG_W   = 16,
  parameter int unsigned IDX_W   = 4,

  parameter int unsigned FEAT_W         = 256,
  parameter int unsigned MEM_W          = 64,
  parameter int unsigned BASE_ADDR_WORD = 0,

  // final output banking to SRAM
  parameter int unsigned BANKS          = 1,     // 1=no bank, >1 power-of-2
  parameter int unsigned ADDR_W         = 32,
  parameter int unsigned READ_LATENCY   = 1,
  parameter bit          USE_RTAG       = 0,

  // assembler option
  parameter bit          Q_FROM_CENTER_IDX4 = 1'b1
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Center tile request
  // ----------------------------
  input  logic                         center_valid,
  output logic                         center_ready,
  input  logic [$clog2(TILES_Y)-1:0]    center_i,
  input  logic [$clog2(TILES_X)-1:0]    center_j,
  input  logic [TAG_W-1:0]             center_tag,

  // ----------------------------
  // Downstream SRAM read command (per beat)
  // ----------------------------
  output logic                         mem_rd_valid,
  input  logic                         mem_rd_ready,
  output logic [ADDR_W-1:0]            mem_addr,   // bank-split addr (if BANKS>1)
  output logic [((BANKS<=1)?1:$clog2(BANKS))-1:0] mem_bank,
  output logic [TAG_W-1:0]             mem_tag,

  // ----------------------------
  // SRAM read return
  // ----------------------------
  input  logic                         mem_rvalid,
  input  logic [MEM_W-1:0]             mem_rdata,
  input  logic [TAG_W-1:0]             mem_rtag,

  // ----------------------------
  // Bundle output (1 bundle per center request)
  // ----------------------------
  output logic                         out_valid,
  input  logic                         out_ready,

  output logic [FEAT_W-1:0]            kv0,
  output logic [FEAT_W-1:0]            kv1,
  output logic [FEAT_W-1:0]            kv2,
  output logic [FEAT_W-1:0]            kv3,
  output logic [FEAT_W-1:0]            kv4,
  output logic [FEAT_W-1:0]            kv5,
  output logic [FEAT_W-1:0]            kv6,
  output logic [FEAT_W-1:0]            kv7,
  output logic [FEAT_W-1:0]            kv8,

  output logic [9*FEAT_W-1:0]          kv_bus,
  output logic [FEAT_W-1:0]            q_vec,
  output logic [TAG_W-1:IDX_W]         out_group_tag,

  // ----------------------------
  // Status / pulses
  // ----------------------------
  output logic                         sched_busy,
  output logic                         sched_done_pulse,   // 9th feature token CONSUMED by assembler
  output logic                         bundle_done_pulse,  // bundle CONSUMED (out_fire)
  output logic [8:0]                   nb_is_center
);

  // ------------------------------------------------------------
  // Internal stream: fetch -> assembler
  // ------------------------------------------------------------
  wire              feat_valid_w;
  wire              feat_ready_w;
  wire [FEAT_W-1:0] feat_vec_w;
  wire [TAG_W-1:0]  feat_tag_w;

  // ------------------------------------------------------------
  // A) Fetch top (center -> 9 feat tokens)
  // ------------------------------------------------------------
  tile_nb_feature_fetch_3x3_top_inst #(
    .TILES_X       (TILES_X),
    .TILES_Y       (TILES_Y),
    .TAG_W         (TAG_W),
    .IDX_W         (IDX_W),
    .FEAT_W        (FEAT_W),
    .MEM_W         (MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS         (BANKS),
    .ADDR_W        (ADDR_W),
    .READ_LATENCY  (READ_LATENCY),
    .USE_RTAG      (USE_RTAG)
  ) u_fetch (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i    (center_i),
    .center_j    (center_j),
    .center_tag  (center_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr    (mem_addr),
    .mem_bank    (mem_bank),
    .mem_tag     (mem_tag),

    .mem_rvalid(mem_rvalid),
    .mem_rdata (mem_rdata),
    .mem_rtag  (mem_rtag),

    .feat_valid(feat_valid_w),
    .feat_ready(feat_ready_w),
    .feat_vec  (feat_vec_w),
    .feat_tag  (feat_tag_w),

    .sched_busy      (sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .nb_is_center    (nb_is_center)
  );

  // ------------------------------------------------------------
  // B) Bundle assembler (9 tokens -> bundle)
  // ------------------------------------------------------------
  nb_feat_bundle_assembler_9 #(
    .FEAT_W(FEAT_W),
    .TAG_W (TAG_W),
    .IDX_W (IDX_W),
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4)
  ) u_bundle (
    .clk(clk),
    .rst(rst),
    .en (en),

    .feat_valid(feat_valid_w),
    .feat_ready(feat_ready_w),
    .feat_vec  (feat_vec_w),
    .feat_tag  (feat_tag_w),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .kv0(kv0), .kv1(kv1), .kv2(kv2), .kv3(kv3), .kv4(kv4),
    .kv5(kv5), .kv6(kv6), .kv7(kv7), .kv8(kv8),

    .kv_bus(kv_bus),
    .q_vec(q_vec),
    .out_group_tag(out_group_tag)
  );

  // ------------------------------------------------------------
  // C) bundle_done_pulse (bundle consumed)
  // ------------------------------------------------------------
  wire go = en && !rst;
  wire out_fire = go && out_valid && out_ready;

  always @* begin
    if (!go) bundle_done_pulse = 1'b0;
    else     bundle_done_pulse = out_fire;   // 1-cycle pulse
  end

endmodule

`default_nettype wire
`endif
