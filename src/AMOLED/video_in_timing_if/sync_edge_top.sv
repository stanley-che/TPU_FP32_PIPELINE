`ifndef SYNC_EDGE_TOP_SV
`define SYNC_EDGE_TOP_SV
`include "./src/AMOLED/video_in_timing_if/sync_sanitizer.sv"
`include "./src/AMOLED/video_in_timing_if/edge_pulse_gen.sv"
`timescale 1ns/1ps
`default_nettype none

/*
  sync_edge_top
- Wrap sync_sanitizer + edge_pulse_gen
- Input: async or already-synced vsync_i/hsync_i/de_i
 - Output: cleaned vsync/hsync/de + edge pulses
  
*/
module sync_edge_top #(
  // ---- same params as sync_sanitizer ----
  parameter bit VS_INV = 1'b0,
  parameter bit HS_INV = 1'b0,
  parameter bit DE_INV = 1'b0,

  parameter bit USE_DEGLITCH = 1'b0,
  parameter int unsigned STABLE_CYCLES = 2
)(
  input  logic clk,
  input  logic rst,

  input  logic vsync_i,
  input  logic hsync_i,
  input  logic de_i,

  // cleaned/synced outputs
  output logic vsync,
  output logic hsync,
  output logic de,

  // 1-cycle edge pulses based on cleaned signals
  output logic vs_rise,
  output logic vs_fall,
  output logic hs_rise,
  output logic hs_fall,
  output logic de_rise,
  output logic de_fall
);

  // -----------------------------
  // 1) Sync + (optional) deglitch
  // -----------------------------
  sync_sanitizer #(
    .VS_INV(VS_INV),
    .HS_INV(HS_INV),
    .DE_INV(DE_INV),
    .USE_DEGLITCH(USE_DEGLITCH),
    .STABLE_CYCLES(STABLE_CYCLES)
  ) u_sync (
    .clk    (clk),
    .rst    (rst),
    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i   (de_i),
    .vsync  (vsync),
    .hsync  (hsync),
    .de     (de)
  );

  // -----------------------------
  // 2) Edge pulse generation
  // -----------------------------
  edge_pulse_gen u_edge (
    .clk    (clk),
    .rst    (rst),
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

endmodule

`default_nettype wire
`endif
