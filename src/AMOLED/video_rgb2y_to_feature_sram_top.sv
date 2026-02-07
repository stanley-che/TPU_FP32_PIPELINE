// ============================================================
// video_rgb2y_to_feature_sram_top.sv  (IVERILOG-SAFE VERSION)
// - FIX: Use upstream ev[sof/eol/eof] from video_rgb2y_integrated_top
// - FIX: Gate events with pix_fire (out_valid && out_ready) so events
//        stay aligned under backpressure.
// - FIX: Mask y with (go && yv) to avoid X poisoning.
// ============================================================

`ifndef VIDEO_RGB2Y_TO_FEATURE_SRAM_TOP_SV
`define VIDEO_RGB2Y_TO_FEATURE_SRAM_TOP_SV

`include "./src/AMOLED/video_rgb2y_integrated_top.sv"
`include "./src/AMOLED/pix_xy_feature_sram_integrated_top.sv"

`timescale 1ns/1ps
`default_nettype none

module video_rgb2y_to_feature_sram_top #(
  // ----------------------------
  // Shared video geometry
  // ----------------------------
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  // sideband: {events(4), sync(3), y(Y_W), x(X_W)}
  parameter int unsigned SB_W = (4+3+Y_W+X_W),

  // ----------------------------
  // video_rgb2y_integrated_top params
  // ----------------------------
  parameter bit VS_INV = 1'b0,
  parameter bit HS_INV = 1'b0,
  parameter bit DE_INV = 1'b0,
  parameter bit USE_DEGLITCH = 1'b0,
  parameter int unsigned STABLE_CYCLES = 2,

  parameter bit FRAME_START_ON_VS_RISE = 1'b1,
  parameter bit USE_HSYNC_FOR_EOL      = 1'b0,

  parameter bit Y_INC_ON_DE_RISE = 1'b0,
  parameter int unsigned X_LIMIT_MODE = 2,
  parameter int unsigned Y_LIMIT_MODE = 2,
  parameter bit ENABLE_BOUNDS     = 1'b1,
  parameter bit GATE_BY_IN_FRAME  = 1'b1,
  parameter bit ENABLE_ASSERT     = 1'b0,

  parameter bit FEV_USE_VS_FALL   = 1'b0,
  parameter bit FEV_GATE_BY_FRAME = 1'b1,

  // rgb2y_luma_top
  parameter int unsigned FORMAT = 0,
  parameter int unsigned LAT_U  = 1,
  parameter bit          ZERO_WHEN_INVALID = 1'b1,

  parameter int unsigned LAT_M  = 1,
  parameter bit          USE_DSP = 1'b0,
  parameter bit          BYPASS  = 1'b0,

  parameter int unsigned LAT_SUM = 1,
  parameter bit          HOLD_VALID_WHEN_STALL = 1'b1,

  parameter int unsigned LAT_RND = 1,
  parameter int unsigned ROUND_MODE = 1,
  parameter int unsigned Y8_MODE = 0,

  // out_pack
  parameter int unsigned PACK_IN_W  = 10,
  parameter int unsigned PACK_OUT_W = 10,
  parameter int unsigned PACK_LAT   = 2,

  parameter bit PACK_GATE_SIDEBAND_WITH_VALID = 1'b1,
  parameter bit PACK_USE_ROUND = 1'b0,
  parameter bit PACK_USE_SAT   = 1'b1,
  parameter bit PACK_USE_CLIP  = 1'b0,
  parameter bit PACK_COUNT_DBG = 1'b0,

  // ----------------------------
  // pix_xy_feature_sram_integrated_top params
  // ----------------------------
  parameter int unsigned TILE_SHIFT = 2,
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  parameter int unsigned YPIX_W = 8,

  // Y width convert: y_pix = Y >> Y_SHIFT_RIGHT
  parameter int unsigned Y_SHIFT_RIGHT =
      (PACK_OUT_W > YPIX_W) ? (PACK_OUT_W - YPIX_W) : 0,

  parameter bit USE_SOF = 1'b1,
  parameter bit USE_EOL = 1'b0,
  parameter bit USE_EOF = 1'b0,

  parameter bit SAT_AT_MAX = 1'b0,
  parameter bit ADVANCE_ON_VALID_ONLY = 1'b1,

  parameter bit          ROI_EN_DEFAULT  = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT  = 0,
  parameter int unsigned ROI_Y0_DEFAULT  = 0,
  parameter int unsigned ROI_X1_DEFAULT  = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT  = ACTIVE_H-1,

  parameter bit DBG_COUNTERS_EN = 1'b1,

  // tile_stats_edge knobs
  parameter int unsigned TILE_W = 4,
  parameter int unsigned TILE_H = 4,

  parameter bit SUPPORT_PIX_VALID_STATS = 1'b1,
  parameter bit DO_SUMSQ_STATS          = 1'b0,
  parameter bit ASSERT_ON_STATS         = 1'b1,

  parameter bit SUPPORT_PIX_VALID_EDGE  = 1'b1,
  parameter bit SUPPORT_TILE_START_EDGE = 1'b1,
  parameter bit ASSERT_ON_EDGE          = 1'b0,

  parameter int unsigned EDGE_MODE = 0,
  parameter int unsigned EDGE_THR  = 8,
  parameter int unsigned EDGE_W    = 32,
  parameter int unsigned CNT_W     = 8,

  // feature pack knobs
  parameter int unsigned FEAT_W   = 16,
  parameter int unsigned FEAT_DIM = 8,

  parameter bit EDGE_USE_MEAN = 1'b1,

  parameter int unsigned TILE_PIX_SHIFT  = 4,
  parameter int unsigned TILE_PIX_N      = 16,
  parameter bit          MEAN_MODE_SHIFT = 1'b1,

  parameter bit MATCH_MODE_BUFFERED = 1'b0,
  parameter bit ZERO_WHEN_INVALID_F = 1'b0,

  parameter int unsigned FEAT0_SHIFT = 0,
  parameter int unsigned FEAT1_SHIFT = 0,
  parameter int unsigned FEAT2_SHIFT = 0,
  parameter int unsigned FEAT3_SHIFT = 0,
  parameter int signed   FEAT0_BIAS  = 0,
  parameter int signed   FEAT1_BIAS  = 0,
  parameter int signed   FEAT2_BIAS  = 0,
  parameter int signed   FEAT3_BIAS  = 0,
  parameter int unsigned FEAT_CLAMP_MAX = (1<<FEAT_W)-1,

  parameter int unsigned TILE_I_W  = 16,
  parameter int unsigned TILE_J_W  = 16,
  parameter int unsigned TILE_ID_W = (TILES_X*TILES_Y <= 1) ? 1 : $clog2(TILES_X*TILES_Y),

  parameter bit ENABLE_CNTS = 1'b1,

  // feature_sram
  parameter int unsigned sram_bus = 32,
  parameter int unsigned elen_W   = 32,
  parameter int unsigned tag_w    = 16,
  parameter bit          isclamp  = 1'b0,

  parameter int unsigned REQ_DEPTH     = 2,
  parameter int unsigned META_DEPTH    = 16,
  parameter int unsigned BEAT_DEPTH    = 16,
  parameter int unsigned RD_FIFO_DEPTH = 2,

  parameter int unsigned RD_LAT    = 2
)(
  input  logic clk,
  input  logic rst,
  input  logic en,

  // raw video in
  input  logic        vsync_i,
  input  logic        hsync_i,
  input  logic        de_i,
  input  logic [23:0] rgb_i,

  // clip controls (rgb2y pack)
  input  logic                  clip_en,
  input  logic [PACK_OUT_W-1:0]  clip_min,
  input  logic [PACK_OUT_W-1:0]  clip_max,

  // external READ port (SRAM)
  input  logic                       valid_rd,
  output logic                       ready_rd,
  input  logic [$clog2(TILES_Y)-1:0]  tile_i_rd,
  input  logic [$clog2(TILES_X)-1:0]  tile_j_rd,
  input  logic [tag_w-1:0]            tag_rd,

  // feature OUT from SRAM
  output logic                       feat_out_valid,
  input  logic                       feat_out_ready,
  output logic [tag_w-1:0]           feat_out_tag,
  output logic [FEAT_DIM*elen_W-1:0] feat_out_data,

  // write-side activity
  output logic                       wr_fire,
  output logic [TILE_I_W-1:0]        wr_tile_i,
  output logic [TILE_J_W-1:0]        wr_tile_j,

  // (optional) debug pass-through from rgb2y stage
  output logic                       rgb2y_out_valid,
  output logic                       rgb2y_out_ready,
  output logic [PACK_OUT_W-1:0]      rgb2y_Y,
  output logic [SB_W-1:0]            rgb2y_sb_out,

  output logic                       in_frame,
  output logic                       in_line,
  output logic                       pix_valid_timing,
  output logic [X_W-1:0]             x,
  output logic [Y_W-1:0]             y,

  output logic                       skid_drop_pulse,
  output logic [31:0]                skid_drop_cnt,

  // packer debug from feature_top
  output logic                       err_mismatch_pulse,
  output logic [31:0]                cnt_join_ok,
  output logic [31:0]                cnt_mismatch,
  output logic [31:0]                cnt_drop
);

  // ============================================================
  // 1) RGB -> Y + sideband
  // ============================================================
  logic                  yv;
  logic                  yr;
  logic [PACK_OUT_W-1:0] Y10;
  logic [SB_W-1:0]       sb;

  video_rgb2y_integrated_top #(
    .VS_INV(VS_INV),
    .HS_INV(HS_INV),
    .DE_INV(DE_INV),
    .USE_DEGLITCH(USE_DEGLITCH),
    .STABLE_CYCLES(STABLE_CYCLES),

    .FRAME_START_ON_VS_RISE(FRAME_START_ON_VS_RISE),
    .USE_HSYNC_FOR_EOL(USE_HSYNC_FOR_EOL),

    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .Y_INC_ON_DE_RISE(Y_INC_ON_DE_RISE),
    .X_LIMIT_MODE(X_LIMIT_MODE),
    .Y_LIMIT_MODE(Y_LIMIT_MODE),
    .ENABLE_BOUNDS(ENABLE_BOUNDS),
    .GATE_BY_IN_FRAME(GATE_BY_IN_FRAME),
    .ENABLE_ASSERT(ENABLE_ASSERT),

    .FEV_USE_VS_FALL(FEV_USE_VS_FALL),
    .FEV_GATE_BY_FRAME(FEV_GATE_BY_FRAME),

    .FORMAT(FORMAT),
    .LAT_U(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .LAT_M(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS),
    .LAT_SUM(LAT_SUM),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),
    .LAT_RND(LAT_RND),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE),

    .SB_W(SB_W),

    .PACK_IN_W(PACK_IN_W),
    .PACK_OUT_W(PACK_OUT_W),
    .PACK_LAT(PACK_LAT),

    .PACK_GATE_SIDEBAND_WITH_VALID(PACK_GATE_SIDEBAND_WITH_VALID),
    .PACK_USE_ROUND(PACK_USE_ROUND),
    .PACK_USE_SAT(PACK_USE_SAT),
    .PACK_USE_CLIP(PACK_USE_CLIP),
    .PACK_COUNT_DBG(PACK_COUNT_DBG)
  ) u_rgb2y_top (
    .clk(clk),
    .rst(rst),
    .en(en),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i(de_i),
    .rgb_i(rgb_i),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .out_valid(yv),
    .out_ready(yr),
    .Y(Y10),
    .sb_out(sb),

    .in_frame(in_frame),
    .in_line(in_line),
    .pix_valid(pix_valid_timing),
    .x(x),
    .y(y),

    .skid_drop_pulse(skid_drop_pulse),
    .skid_drop_cnt(skid_drop_cnt)
  );

  // debug export
  assign rgb2y_out_valid = yv;
  assign rgb2y_out_ready = yr;
  assign rgb2y_Y         = Y10;
  assign rgb2y_sb_out    = sb;

  // ============================================================
  // 2) Extract events + Y convert (ALIGNED WITH FIRE)
  // ============================================================
  wire go       = en && !rst;
  wire pix_fire = yv && yr;   // IMPORTANT: align events with accepted pixels

  // sb layout: {ev(4), sync(3), y(Y_W), x(X_W)}
  logic [3:0] ev;
  assign ev = sb[X_W+Y_W+3 +: 4];

  // upstream events (already aligned to yv)
  wire sof_s = ev[3];
  wire sol_s = ev[2]; // optional / unused
  wire eol_s = ev[1];
  wire eof_s = ev[0];

  logic [YPIX_W-1:0] y_pix;
  always_comb begin
    if (PACK_OUT_W <= YPIX_W) begin
      y_pix = {{(YPIX_W-PACK_OUT_W){1'b0}}, Y10};
    end else begin
      y_pix = (Y10 >> Y_SHIFT_RIGHT);
    end
  end

  // mask (no X)
  logic sof_m, eol_m, eof_m;
  logic [YPIX_W-1:0] y_m;

  // NOTE:
  // - If feature side stalls, we only emit sof/eol/eof when the pixel is accepted.
  // - This keeps tile stats / line boundaries consistent.
  assign sof_m = go && pix_fire && sof_s;
  assign eol_m = go && pix_fire && eol_s;
  assign eof_m = go && pix_fire && eof_s;

  assign y_m   = (go && yv) ? y_pix : '0;

  // ============================================================
  // 3) Feature extract + SRAM
  // ============================================================
  logic pix_ready_feat;
  assign yr = pix_ready_feat;

  pix_xy_feature_sram_integrated_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .YPIX_W(YPIX_W),

    .USE_SOF(USE_SOF),
    .USE_EOL(USE_EOL),
    .USE_EOF(USE_EOF),
    .SAT_AT_MAX(SAT_AT_MAX),
    .ADVANCE_ON_VALID_ONLY(ADVANCE_ON_VALID_ONLY),

    .ROI_EN_DEFAULT(ROI_EN_DEFAULT),
    .ROI_X0_DEFAULT(ROI_X0_DEFAULT),
    .ROI_Y0_DEFAULT(ROI_Y0_DEFAULT),
    .ROI_X1_DEFAULT(ROI_X1_DEFAULT),
    .ROI_Y1_DEFAULT(ROI_Y1_DEFAULT),

    .DBG_COUNTERS_EN(DBG_COUNTERS_EN),

    .TILE_W(TILE_W),
    .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID_STATS(SUPPORT_PIX_VALID_STATS),
    .DO_SUMSQ_STATS(DO_SUMSQ_STATS),
    .ASSERT_ON_STATS(ASSERT_ON_STATS),

    .SUPPORT_PIX_VALID_EDGE(SUPPORT_PIX_VALID_EDGE),
    .SUPPORT_TILE_START_EDGE(SUPPORT_TILE_START_EDGE),
    .ASSERT_ON_EDGE(ASSERT_ON_EDGE),

    .EDGE_MODE(EDGE_MODE),
    .EDGE_THR(EDGE_THR),
    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W),

    .FEAT_W(FEAT_W),
    .FEAT_DIM(FEAT_DIM),

    .EDGE_USE_MEAN(EDGE_USE_MEAN),
    .TILE_PIX_SHIFT(TILE_PIX_SHIFT),
    .TILE_PIX_N(TILE_PIX_N),
    .MEAN_MODE_SHIFT(MEAN_MODE_SHIFT),

    .MATCH_MODE_BUFFERED(MATCH_MODE_BUFFERED),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID_F),

    .FEAT0_SHIFT(FEAT0_SHIFT),
    .FEAT1_SHIFT(FEAT1_SHIFT),
    .FEAT2_SHIFT(FEAT2_SHIFT),
    .FEAT3_SHIFT(FEAT3_SHIFT),
    .FEAT0_BIAS(FEAT0_BIAS),
    .FEAT1_BIAS(FEAT1_BIAS),
    .FEAT2_BIAS(FEAT2_BIAS),
    .FEAT3_BIAS(FEAT3_BIAS),
    .FEAT_CLAMP_MAX(FEAT_CLAMP_MAX),

    .TILE_I_W(TILE_I_W),
    .TILE_J_W(TILE_J_W),
    .TILE_ID_W(TILE_ID_W),

    .ENABLE_CNTS(ENABLE_CNTS),

    .sram_bus(sram_bus),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp),

    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH),

    .RD_LAT(RD_LAT)
  ) u_feat_sram_top (
    .clk(clk),
    .rst(rst),

    .en(en),
    .pix_valid(yv),
    .pix_ready(pix_ready_feat),

    // masked, fire-aligned events
    .sof(sof_m),
    .eol(eol_m),
    .eof(eof_m),
    .y_in(y_m),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data),

    .wr_fire(wr_fire),
    .wr_tile_i(wr_tile_i),
    .wr_tile_j(wr_tile_j),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

endmodule

`default_nettype wire
`endif
