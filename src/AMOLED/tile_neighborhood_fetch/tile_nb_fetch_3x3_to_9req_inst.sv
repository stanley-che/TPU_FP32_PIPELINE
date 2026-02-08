// ============================================================
// tile_nb_fetch_3x3_to_9req_inst.sv  (IVERILOG-SAFE)
// - Top module that INSTANTIATES (instances) the two modules:
//   1) tile_coord_clamp_3x3
//   2) nb_fetch_scheduler_9req
//
// center_valid/ready + center_i/j/tag  ->  rd_valid/ready + rd_tile_i/j + rd_tag
// ============================================================

`ifndef TILE_NB_FETCH_3X3_TO_9REQ_INST_SV
`define TILE_NB_FETCH_3X3_TO_9REQ_INST_SV
`include "./src/AMOLED/tile_neighborhood_fetch/tile_coord_clamp_3x3.sv"
`include "./src/AMOLED/tile_neighborhood_fetch/nb_fetch_scheduler_9req.sv"
`timescale 1ns/1ps
`default_nettype none

module tile_nb_fetch_3x3_to_9req_inst #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,
  parameter int unsigned TAG_W   = 16,
  parameter int unsigned IDX_W   = 4
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Center request in
  // ----------------------------
  input  logic                         center_valid,
  output logic                         center_ready,
  input  logic [$clog2(TILES_Y)-1:0]    center_i,
  input  logic [$clog2(TILES_X)-1:0]    center_j,
  input  logic [TAG_W-1:0]             center_tag,

  // ----------------------------
  // SRAM read req out (9 beats)
  // ----------------------------
  output logic                         rd_valid,
  input  logic                         rd_ready,
  output logic [$clog2(TILES_Y)-1:0]    rd_tile_i,
  output logic [$clog2(TILES_X)-1:0]    rd_tile_j,
  output logic [TAG_W-1:0]             rd_tag,

  // ----------------------------
  // Optional side outputs
  // ----------------------------
  output logic                         sched_busy,
  output logic                         sched_done_pulse,
  output logic [8:0]                   nb_is_center
);

  // ----------------------------
  // Internal wires between clamp -> scheduler
  // ----------------------------
  logic                         nb_valid;
  logic                         nb_ready;

  logic [$clog2(TILES_Y)-1:0]    nb_i0, nb_i1, nb_i2, nb_i3, nb_i4, nb_i5, nb_i6, nb_i7, nb_i8;
  logic [$clog2(TILES_X)-1:0]    nb_j0, nb_j1, nb_j2, nb_j3, nb_j4, nb_j5, nb_j6, nb_j7, nb_j8;
  logic [TAG_W-1:0]             nb_tag;

  // ============================================================
  // Instance A: tile_coord_clamp_3x3
  // ============================================================
  tile_coord_clamp_3x3 #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W  (TAG_W)
  ) u_clamp_3x3 (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i    (center_i),
    .center_j    (center_j),
    .center_tag  (center_tag),

    .nb_valid(nb_valid),
    .nb_ready(nb_ready),

    .nb_i0(nb_i0), .nb_j0(nb_j0),
    .nb_i1(nb_i1), .nb_j1(nb_j1),
    .nb_i2(nb_i2), .nb_j2(nb_j2),
    .nb_i3(nb_i3), .nb_j3(nb_j3),
    .nb_i4(nb_i4), .nb_j4(nb_j4),
    .nb_i5(nb_i5), .nb_j5(nb_j5),
    .nb_i6(nb_i6), .nb_j6(nb_j6),
    .nb_i7(nb_i7), .nb_j7(nb_j7),
    .nb_i8(nb_i8), .nb_j8(nb_j8),

    .nb_tag(nb_tag),
    .nb_is_center(nb_is_center)
  );

  // ============================================================
  // Instance B: nb_fetch_scheduler_9req
  // ============================================================
  nb_fetch_scheduler_9req #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W  (TAG_W),
    .IDX_W  (IDX_W)
  ) u_sched_9req (
    .clk(clk),
    .rst(rst),
    .en (en),

    .nb_valid(nb_valid),
    .nb_ready(nb_ready),

    .nb_i0(nb_i0), .nb_j0(nb_j0),
    .nb_i1(nb_i1), .nb_j1(nb_j1),
    .nb_i2(nb_i2), .nb_j2(nb_j2),
    .nb_i3(nb_i3), .nb_j3(nb_j3),
    .nb_i4(nb_i4), .nb_j4(nb_j4),
    .nb_i5(nb_i5), .nb_j5(nb_j5),
    .nb_i6(nb_i6), .nb_j6(nb_j6),
    .nb_i7(nb_i7), .nb_j7(nb_j7),
    .nb_i8(nb_i8), .nb_j8(nb_j8),

    .nb_tag(nb_tag),

    .rd_valid (rd_valid),
    .rd_ready (rd_ready),
    .rd_tile_i(rd_tile_i),
    .rd_tile_j(rd_tile_j),
    .rd_tag   (rd_tag),

    .sched_busy      (sched_busy),
    .sched_done_pulse(sched_done_pulse)
  );

endmodule

`default_nettype wire
`endif
