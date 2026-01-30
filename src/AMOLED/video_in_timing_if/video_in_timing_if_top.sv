`ifndef VIDEO_IN_TIMING_IF_TOP_SV
`define VIDEO_IN_TIMING_IF_TOP_SV

`include "./src/AMOLED/video_in_timing_if/sync_edge_top.sv"
`include "./src/AMOLED/video_in_timing_if/active_window_tracker.sv"
`include "./src/AMOLED/video_in_timing_if/xy_counter.sv"
`include "./src/AMOLED/video_in_timing_if/frame_event_gen.sv"

`timescale 1ns/1ps
`default_nettype none

module video_in_timing_if_top #(
  // ----------------------------
  // sync_edge_top params
  // ----------------------------
  parameter bit VS_INV = 1'b0,
  parameter bit HS_INV = 1'b0,
  parameter bit DE_INV = 1'b0,
  parameter bit USE_DEGLITCH = 1'b0,
  parameter int unsigned STABLE_CYCLES = 2,

  // ----------------------------
  // active_window_tracker params
  // ----------------------------
  parameter bit FRAME_START_ON_VS_RISE = 1'b1,
  parameter bit USE_HSYNC_FOR_EOL      = 1'b0,

  // ----------------------------
  // xy_counter params
  // ----------------------------
  parameter int unsigned X_W = 11,
  parameter int unsigned Y_W = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  parameter bit Y_INC_ON_DE_RISE = 1'b0,
  parameter int unsigned X_LIMIT_MODE = 2, // 0 none, 1 wrap, 2 sat
  parameter int unsigned Y_LIMIT_MODE = 2,

  parameter bit ENABLE_BOUNDS     = 1'b1,
  parameter bit GATE_BY_IN_FRAME  = 1'b1,
  parameter bit ENABLE_ASSERT     = 1'b0,

  // ----------------------------
  // frame_event_gen params
  // ----------------------------
  parameter bit FEV_USE_VS_FALL   = 1'b0,
  parameter bit FEV_GATE_BY_FRAME = 1'b1
)(
  input  logic                 clk,
  input  logic                 rst,

  // raw input
  input  logic                 vsync_i,
  input  logic                 hsync_i,
  input  logic                 de_i,
  input  logic [23:0]          rgb_i,

  // cleaned outputs (optional export)
  output logic                 vsync,
  output logic                 hsync,
  output logic                 de,

  // tracker state
  output logic                 in_frame,
  output logic                 in_line,
  output logic                 pix_valid,

  // xy
  output logic [X_W-1:0]        x,
  output logic [Y_W-1:0]        y,

  // xy pulses (from xy_counter)
  output logic                 frame_start_p,
  output logic                 line_start_p,
  output logic                 line_end_p,

  // flags
  output logic                 x_at_last,
  output logic                 y_at_last,
  output logic                 x_overflow,
  output logic                 y_overflow,

  // frame events (from frame_event_gen)
  output logic                 sof,
  output logic                 sol,
  output logic                 eol,
  output logic                 eof,

  // forwarded pixel bus (from frame_event_gen)
  output logic                 pix_valid_o,
  output logic [23:0]          pix_rgb_out,
  output logic [X_W-1:0]       x_o,
  output logic [Y_W-1:0]       y_o
);

  // ------------------------------------------------------------
  // 1) sync + edge pulses (internal wires)
  // ------------------------------------------------------------
  logic vs_rise, vs_fall;
  logic hs_rise, hs_fall;
  logic de_rise, de_fall;

  sync_edge_top #(
    .VS_INV(VS_INV),
    .HS_INV(HS_INV),
    .DE_INV(DE_INV),
    .USE_DEGLITCH(USE_DEGLITCH),
    .STABLE_CYCLES(STABLE_CYCLES)
  ) u_se (
    .clk(clk),
    .rst(rst),
    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i   (de_i),
    .vsync  (vsync),
    .hsync  (hsync),
    .de     (de),
    .vs_rise(vs_rise),
    .vs_fall(vs_fall),
    .hs_rise(hs_rise),
    .hs_fall(hs_fall),
    .de_rise(de_rise),
    .de_fall(de_fall)
  );

  // ------------------------------------------------------------
  // 2) active window tracking
  // ------------------------------------------------------------
  active_window_tracker #(
    .FRAME_START_ON_VS_RISE(FRAME_START_ON_VS_RISE),
    .USE_HSYNC_FOR_EOL     (USE_HSYNC_FOR_EOL)
  ) u_awt (
    .clk      (clk),
    .rst      (rst),
    .vs_rise  (vs_rise),
    .vs_fall  (vs_fall),
    .hs_rise  (hs_rise),
    .hs_fall  (hs_fall),
    .de       (de),
    .de_rise  (de_rise),
    .de_fall  (de_fall),
    .in_frame (in_frame),
    .in_line  (in_line),
    .pix_valid(pix_valid)
  );

  // ------------------------------------------------------------
  // 3) xy counter
  // ------------------------------------------------------------
  xy_counter #(
    .X_W             (X_W),
    .Y_W             (Y_W),
    .ACTIVE_W        (ACTIVE_W),
    .ACTIVE_H        (ACTIVE_H),
    .Y_INC_ON_DE_RISE(Y_INC_ON_DE_RISE),
    .X_LIMIT_MODE    (X_LIMIT_MODE),
    .Y_LIMIT_MODE    (Y_LIMIT_MODE),
    .ENABLE_BOUNDS   (ENABLE_BOUNDS),
    .GATE_BY_IN_FRAME(GATE_BY_IN_FRAME),
    .ENABLE_ASSERT   (ENABLE_ASSERT)
  ) u_xy (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .de_rise  (de_rise),
    .de_fall  (de_fall),
    .vs_rise  (vs_rise),
    .in_frame_i(in_frame),
    .x(x),
    .y(y),
    .frame_start_p(frame_start_p),
    .line_start_p (line_start_p),
    .line_end_p   (line_end_p),
    .x_at_last (x_at_last),
    .y_at_last (y_at_last),
    .x_overflow(x_overflow),
    .y_overflow(y_overflow)
  );

  // ------------------------------------------------------------
  // 4) frame_event_gen (events + pixel forwarding)
  // ------------------------------------------------------------
  frame_event_gen #(
    .USE_VS_FALL  (FEV_USE_VS_FALL),
    .GATE_BY_FRAME(FEV_GATE_BY_FRAME)
  ) u_fev (
    .clk(clk),
    .rst(rst),

    .pix_valid  (pix_valid),
    .pix_rgb_in (rgb_i),
    .x          (x),
    .y          (y),

    .vs_rise    (vs_rise),
    .vs_fall    (vs_fall),
    .de_rise    (de_rise),
    .de_fall    (de_fall),

    .in_frame   (in_frame),

    .sof        (sof),
    .sol        (sol),
    .eol        (eol),
    .eof        (eof),

    .pix_valid_o(pix_valid_o),
    .pix_rgb_out(pix_rgb_out),
    .x_o        (x_o),
    .y_o        (y_o)
  );

endmodule

`default_nettype wire
`endif
