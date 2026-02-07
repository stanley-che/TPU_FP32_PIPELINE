// pix_xy_linebuf_stats_edge_top.sv
// ------------------------------------------------------------
// Integrates:
//   1) pix_xy_linebuf_top      : pixel_xy_counter + 1-line Y buffer
//   2) tile_stats_edge_4x4     : 4x4 tile stats + edge energy, joined bundle
//
// Stream:
//   (pix_valid/y_in/sof) -> linebuf -> (y_cur,y_left,y_up,x_mod,y_mod,tile_first,tile_last,meta)
//                         -> tile_stats_edge_4x4 -> (sum/min/max/mean/range + edge metrics) ready/valid
// ------------------------------------------------------------

`ifndef PIX_XY_LINEBUF_STATS_EDGE_TOP_SV
`define PIX_XY_LINEBUF_STATS_EDGE_TOP_SV

`include "./src/AMOLED/tile feature_extracter/pixel_xy_linebuf_1line_top.sv"
`include "./src/AMOLED/tile feature_extracter/tile_stats_edge_raster_4x4.sv"


`timescale 1ns/1ps
`default_nettype none

module pix_xy_linebuf_stats_edge_top #(
  // -----------------------------
  // video / coordinate
  // -----------------------------
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  // Tile config (4x4 default)
  parameter int unsigned TILE_SHIFT = 2,
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  // pixel value width
  parameter int unsigned YPIX_W = 8,

  // pixel_xy_counter behavior
  parameter bit USE_SOF = 1'b1,
  parameter bit USE_EOL = 1'b0,
  parameter bit USE_EOF = 1'b0,
  parameter bit SAT_AT_MAX = 1'b0,
  parameter bit ADVANCE_ON_VALID_ONLY = 1'b1,

  // ROI defaults (keep same style as your pix_xy_linebuf_top)
  parameter bit          ROI_EN_DEFAULT  = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT  = 0,
  parameter int unsigned ROI_Y0_DEFAULT  = 0,
  parameter int unsigned ROI_X1_DEFAULT  = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT  = ACTIVE_H-1,

  parameter bit DBG_COUNTERS_EN = 1'b1,

  // -----------------------------
  // tile_stats_edge_4x4 knobs
  // -----------------------------
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

  // ---- derived widths for stats output ports ----
  parameter int unsigned TILE_PIXELS = TILE_W*TILE_H,
  parameter int unsigned SUM_MAX     = TILE_PIXELS * ((1<<YPIX_W) - 1),
  parameter int unsigned SUM_W       = (SUM_MAX <= 1) ? 1 : $clog2(SUM_MAX + 1),

  parameter int unsigned SUMSQ_MAX   = TILE_PIXELS * (((1<<YPIX_W)-1) * ((1<<YPIX_W)-1)),
  parameter int unsigned SUMSQ_W     = (SUMSQ_MAX <= 1) ? 1 : $clog2(SUMSQ_MAX + 1)
)(
  input  logic                   clk,
  input  logic                   rst,
  input  logic                   en,

  // -----------------------------
  // upstream pixel/control stream
  // -----------------------------
  input  logic                   pix_valid,
  output logic                   pix_ready,
  input  logic                   sof,
  input  logic                   eol,
  input  logic                   eof,
  input  logic [YPIX_W-1:0]      y_in,

  // -----------------------------
  // downstream tile bundle (ready/valid)
  // -----------------------------
  output logic                   out_valid,
  input  logic                   out_ready,

  // ---- stats outputs ----
  output logic [SUM_W-1:0]       sumY,
  output logic [YPIX_W-1:0]      minY,
  output logic [YPIX_W-1:0]      maxY,
  output logic [YPIX_W-1:0]      meanY,
  output logic [YPIX_W-1:0]      rangeY,

  output logic                   stats_tile_err,
  output logic [7:0]             stats_err_code,
  output logic [$clog2(TILE_W*TILE_H+1)-1:0] stats_pix_cnt_o,

  // meta pass-through (from our wrapper policy)
  output logic [15:0]            y_meta_o,
  output logic [15:0]            tile_x_o,
  output logic [15:0]            tile_y_o,
  output logic [15:0]            frame_id_o,

  // ---- edge outputs ----
  output logic [EDGE_W-1:0]      edge_sum,
  output logic [EDGE_W-1:0]      edge_max,
  output logic [CNT_W-1:0]       edge_cnt,
  output logic [EDGE_W-1:0]      edge_mean,

  // -----------------------------
  // also export coordinates/meta if you need them
  // -----------------------------
  output logic [X_W-1:0]         x,
  output logic [Y_W-1:0]         y,

  output logic [X_W-1:0]         tile_j,
  output logic [Y_W-1:0]         tile_i,
  output logic [1:0]             x_mod,
  output logic [1:0]             y_mod,

  output logic                   tile_first,
  output logic                   tile_last,
  output logic                   in_roi,

  output logic                   err_sof_midframe,
  output logic                   err_eol_mismatch,
  output logic [31:0]            frame_cnt,
  output logic [31:0]            line_cnt
);

  // ============================================================
  // Stage A: pix_xy_linebuf_top
  // ============================================================
  logic                v2_valid;
  logic                v2_ready;
  logic [YPIX_W-1:0]    y_cur, y_left, y_up;

  // Unused but available from pix_xy_linebuf_top (keep for debug if you want)
  logic tile_x_last, tile_y_last;
  logic x_last, y_last, line_start, line_last, frame_start, frame_last;
  logic [$clog2(TILES_X*TILES_Y)-1:0] tile_idx;

  pix_xy_linebuf_top #(
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

    .DBG_COUNTERS_EN(DBG_COUNTERS_EN)
  ) u_linebuf (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .pix_ready(pix_ready),

    .sof(sof),
    .eol(eol),
    .eof(eof),

    .y_in(y_in),

    .out_valid(v2_valid),
    .out_ready(v2_ready),

    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up),

    .x(x),
    .y(y),

    .tile_j(tile_j),
    .tile_i(tile_i),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .tile_first(tile_first),
    .tile_last(tile_last),
    .tile_x_last(tile_x_last),
    .tile_y_last(tile_y_last),

    .x_last(x_last),
    .y_last(y_last),
    .line_start(line_start),
    .line_last(line_last),
    .frame_start(frame_start),
    .frame_last(frame_last),

    .tile_idx(tile_idx),

    .in_roi(in_roi),

    .err_sof_midframe(err_sof_midframe),
    .err_eol_mismatch(err_eol_mismatch),
    .frame_cnt(frame_cnt),
    .line_cnt(line_cnt)
  );

  // ============================================================
  // Meta policy for tile_stats_edge_4x4
  // - tile_x/y idx: use tile_j/tile_i (truncate to 16)
  // - frame_id:     use frame_cnt (truncate to 16)
  // - y_meta:       optional; here we pass current y (or 0)
  // - pix_in_tile_valid: use in_roi (or 1 if you want always)
  // - tile_start: use tile_first
  // ============================================================
  logic [15:0] tile_x_idx, tile_y_idx, frame_id, y_meta;
  logic        pix_in_tile_valid;
  logic        tile_start;
assign tile_x_idx = {{(16-X_W){1'b0}}, tile_j};    // tile_j width = X_W
assign tile_y_idx = {{(16-Y_W){1'b0}}, tile_i};    // tile_i width = Y_W
assign frame_id   = frame_cnt[15:0];               // frame_cnt is 32 bits
assign y_meta     = {{(16-Y_W){1'b0}}, y};         // y width = Y_W

  // 2) or pass something else (e.g., mean of tile, luma tag, etc.)

  //assign pix_in_tile_valid = in_roi;     // common choice: only ROI pixels count
  assign pix_in_tile_valid = 1'b1;  
  assign tile_start        = tile_first; // first pixel of tile

    // ============================================================
  // Stage B (RASTER): per-tile stats+edge that works with raster scan
  // ============================================================
  tile_stats_edge_raster_4x4 #(
    .Y_W(YPIX_W),
    .TILE_W(TILE_W),
    .TILE_H(TILE_H),
    .TILES_X(TILES_X),

    .SUM_W(SUM_W),
    .EDGE_W(EDGE_W),

    .OUT_FIFO_DEPTH(2)
  ) u_stats_edge_raster (
    .clk(clk),
    .rst(rst),
    .en(en),

    .v2_valid(v2_valid),
    .v2_ready(v2_ready),

    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up),

    .tile_x_idx(tile_x_idx),
    .tile_y_idx(tile_y_idx),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .frame_id(frame_id),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .sumY(sumY),
    .minY(minY),
    .maxY(maxY),

    .edge_sum(edge_sum),

    .tile_x_o(tile_x_o),
    .tile_y_o(tile_y_o),
    .frame_id_o(frame_id_o)
  );

  // edge_max/edge_cnt/edge_mean 你如果暫時不用，可以先 tie-off
  assign edge_max  = '0;
  assign edge_cnt  = '0;
  assign edge_mean = '0;


endmodule

`default_nettype wire
`endif
