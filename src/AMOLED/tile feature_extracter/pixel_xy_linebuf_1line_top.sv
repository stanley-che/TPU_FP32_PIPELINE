// pix_xy_linebuf_top.sv
// ------------------------------------------------------------
// Integrates:
//   1) pixel_xy_counter  : generates x/y + tile flags with ready/valid
//   2) linebuf_y_1line   : 1-line Y buffer (y_cur/y_left/y_up) with backpressure
//
// Data flow:
//   (pix_valid, sof/eol/eof) ---> pixel_xy_counter ---> (v1_valid,x,y) ---> linebuf ---> (out_valid, y_cur/y_left/y_up)
//                                         ^                                      |
//                                         |                                      v
//                                      pix_ready <--- backpressure <--- out_ready
// ------------------------------------------------------------

`ifndef PIX_XY_LINEBUF_TOP_SV
`define PIX_XY_LINEBUF_TOP_SV
`include "./src/AMOLED/tile feature_extracter/pixel_xy_counter.sv"
`include "./src/AMOLED/tile feature_extracter/linebuf_y_1line.sv"
`timescale 1ns/1ps
`default_nettype none

module pix_xy_linebuf_top #(
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  // Tile config (4x4 default)
  parameter int unsigned TILE_SHIFT = 2,
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  // pixel value width
  parameter int unsigned YPIX_W = 10,

  // pixel_xy_counter behavior
  parameter bit USE_SOF = 1'b1,
  parameter bit USE_EOL = 1'b0,
  parameter bit USE_EOF = 1'b0,
  parameter bit SAT_AT_MAX = 1'b0,
  parameter bit ADVANCE_ON_VALID_ONLY = 1'b1,

  // ROI defaults
  parameter bit          ROI_EN_DEFAULT  = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT  = 0,
  parameter int unsigned ROI_Y0_DEFAULT  = 0,
  parameter int unsigned ROI_X1_DEFAULT  = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT  = ACTIVE_H-1,

  parameter bit DBG_COUNTERS_EN = 1'b1
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

  // pixel payload (Y)
  input  logic [YPIX_W-1:0]      y_in,

  // -----------------------------
  // downstream output (ready/valid)
  // -----------------------------
  output logic                   out_valid,
  input  logic                   out_ready,

  output logic [YPIX_W-1:0]      y_cur,
  output logic [YPIX_W-1:0]      y_left,
  output logic [YPIX_W-1:0]      y_up,

  // also export coordinates/meta if you need them
  output logic [X_W-1:0]         x,
  output logic [Y_W-1:0]         y,

  output logic [X_W-1:0]         tile_j,
  output logic [Y_W-1:0]         tile_i,
  output logic [1:0]             x_mod,
  output logic [1:0]             y_mod,

  output logic                   tile_first,
  output logic                   tile_last,
  output logic                   tile_x_last,
  output logic                   tile_y_last,

  output logic                   x_last,
  output logic                   y_last,
  output logic                   line_start,
  output logic                   line_last,
  output logic                   frame_start,
  output logic                   frame_last,

  output logic [$clog2(TILES_X*TILES_Y)-1:0] tile_idx,

  output logic                   in_roi,

  output logic                   err_sof_midframe,
  output logic                   err_eol_mismatch,
  output logic [31:0]            frame_cnt,
  output logic [31:0]            line_cnt
);

  // ============================================================
  // internal ready/valid between counter -> linebuf
  // ============================================================
  logic                  v1_valid;
  logic                  v1_ready;
  logic [X_W-1:0]         x_int;
  logic [Y_W-1:0]         y_int;

  // ============================================================
  // pixel_xy_counter
  // ============================================================
  pixel_xy_counter #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

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
  ) u_xy (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .pix_ready(pix_ready),

    .sof(sof),
    .eol(eol),
    .eof(eof),

    .v1_valid(v1_valid),
    .v1_ready(v1_ready),
    .x(x_int),
    .y(y_int),

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

  // export x/y
  assign x = x_int;
  assign y = y_int;

  // ============================================================
  // linebuf_y_1line
  //  - consumes (v1_valid,x,y_in)
  //  - produces (out_valid, y_cur/y_left/y_up)
  // ============================================================
  linebuf_y_1line #(
    .X_W(X_W),
    .Y_W(YPIX_W),
    .ACTIVE_W(ACTIVE_W)
  ) u_lb (
    .clk(clk),
    .rst(rst),
    .en(en),

    .v1_valid(v1_valid),
    .v1_ready(v1_ready),
    .x(x_int),
    .y_in(y_in),

    .v2_valid(out_valid),
    .v2_ready(out_ready),
    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up)
  );

endmodule

`default_nettype wire
`endif
