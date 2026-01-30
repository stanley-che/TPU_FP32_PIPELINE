// tile_stats_edge_4x4.sv
`ifndef TILE_STATS_EDGE_4X4_SV
`define TILE_STATS_EDGE_4X4_SV
`include "./src/AMOLED/tile feature_extracter/tile_stats_4x4.sv"
`include "./src/AMOLED/tile feature_extracter/tile_edge_energy_4x4.sv"
`timescale 1ns/1ps
`default_nettype none



module tile_stats_edge_4x4 #(
  parameter int unsigned Y_W    = 8,
  parameter int unsigned TILE_W = 4,
  parameter int unsigned TILE_H = 4,

  // ---- stats switches ----
  parameter bit SUPPORT_PIX_VALID_STATS = 1'b1,
  parameter bit DO_SUMSQ_STATS          = 1'b0,
  parameter bit ASSERT_ON_STATS         = 1'b1,

  // ---- edge switches ----
  parameter bit SUPPORT_PIX_VALID_EDGE  = 1'b1,
  parameter bit SUPPORT_TILE_START_EDGE = 1'b1,
  parameter bit ASSERT_ON_EDGE          = 1'b0,

  parameter int unsigned EDGE_MODE      = 0,
  parameter int unsigned EDGE_THR       = 8,

  // Fix widths for easy wiring
  parameter int unsigned EDGE_W = 32,
  parameter int unsigned CNT_W  = 8,

  // ---- DERIVED widths (make them parameters so ports can use) ----
  parameter int unsigned TILE_PIXELS = TILE_W*TILE_H,

  parameter int unsigned SUM_MAX     = TILE_PIXELS * ((1<<Y_W) - 1),
  parameter int unsigned SUM_W       = (SUM_MAX <= 1) ? 1 : $clog2(SUM_MAX + 1),

  parameter int unsigned SUMSQ_MAX   = TILE_PIXELS * (((1<<Y_W)-1) * ((1<<Y_W)-1)),
  parameter int unsigned SUMSQ_W     = (SUMSQ_MAX <= 1) ? 1 : $clog2(SUMSQ_MAX + 1)
)(
  input  logic                     clk,
  input  logic                     rst,

  // -----------------------------
  // Upstream pixel stream (v2)
  // -----------------------------
  input  logic                     v2_valid,
  output logic                     v2_ready,

  input  logic [Y_W-1:0]            y_cur,
  input  logic [Y_W-1:0]            y_left,
  input  logic [Y_W-1:0]            y_up,
  input  logic [$clog2(TILE_W)-1:0] x_mod,
  input  logic [$clog2(TILE_H)-1:0] y_mod,

  input  logic                     tile_last,
  input  logic                     tile_start,
  input  logic                     pix_in_tile_valid,

  // meta
  input  logic [15:0]              tile_x_idx,
  input  logic [15:0]              tile_y_idx,
  input  logic [15:0]              frame_id,
  input  logic [15:0]              y_meta,

  // -----------------------------
  // Downstream tile bundle
  // -----------------------------
  output logic                     out_valid,
  input  logic                     out_ready,

  // ---- stats outputs ----
  output logic [SUM_W-1:0]         sumY,
  output logic [Y_W-1:0]           minY,
  output logic [Y_W-1:0]           maxY,
  output logic [Y_W-1:0]           meanY,
  output logic [Y_W-1:0]           rangeY,

  output logic                     stats_tile_err,
  output logic [7:0]               stats_err_code,
  output logic [$clog2(TILE_W*TILE_H+1)-1:0] stats_pix_cnt_o,

  output logic [15:0]              y_meta_o,
  output logic [15:0]              tile_x_o,
  output logic [15:0]              tile_y_o,
  output logic [15:0]              frame_id_o,

  // ---- edge outputs ----
  output logic [EDGE_W-1:0]        edge_sum,
  output logic [EDGE_W-1:0]        edge_max,
  output logic [CNT_W-1:0]         edge_cnt,
  output logic [EDGE_W-1:0]        edge_mean
);

  // ============================================================
  // tile_stats_4x4
  // ============================================================
  logic stats_v3_valid, stats_v3_ready;

  logic [SUM_W-1:0]   stats_sumY;
  logic [Y_W-1:0]     stats_minY, stats_maxY, stats_meanY, stats_rangeY;
  logic [SUMSQ_W-1:0] stats_sumSq;

  logic [15:0] stats_y_meta_o, stats_tile_x_o, stats_tile_y_o, stats_frame_id_o;
  logic        stats_tile_err_i;
  logic [7:0]  stats_err_code_i;
  logic [$clog2(TILE_W*TILE_H+1)-1:0] stats_pix_cnt_i;
  logic        stats_ov_sum, stats_ov_sumsq;

  wire stats_hold = stats_v3_valid && !stats_v3_ready;

  tile_stats_4x4 #(
    .Y_W(Y_W),
    .TILE_W(TILE_W),
    .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(SUPPORT_PIX_VALID_STATS),
    .DO_SUMSQ(DO_SUMSQ_STATS),
    .ASSERT_ON(ASSERT_ON_STATS)
  ) u_stats (
    .clk, .rst,

    .v2_valid(v2_valid),
    .y_in(y_cur),
    .x_mod(x_mod),
    .y_mod(y_mod),
    .tile_last(tile_last),
    .y_cur(y_meta),

    .pix_in_tile_valid(pix_in_tile_valid),

    .tile_x_idx(tile_x_idx),
    .tile_y_idx(tile_y_idx),
    .frame_id(frame_id),

    .v3_valid(stats_v3_valid),
    .v3_ready(stats_v3_ready),

    .sumY(stats_sumY),
    .minY(stats_minY),
    .maxY(stats_maxY),
    .meanY(stats_meanY),
    .rangeY(stats_rangeY),
    .sumSq(stats_sumSq),

    .y_cur_o(stats_y_meta_o),
    .tile_x_o(stats_tile_x_o),
    .tile_y_o(stats_tile_y_o),
    .frame_id_o(stats_frame_id_o),

    .tile_err(stats_tile_err_i),
    .err_code(stats_err_code_i),
    .pix_cnt_o(stats_pix_cnt_i),
    .overflow_sum(stats_ov_sum),
    .overflow_sumsq(stats_ov_sumsq)
  );

  // ============================================================
  // tile_edge_energy_4x4
  // ============================================================
  logic edge_v2_ready;
  logic edge_v4_valid, edge_v4_ready;
  logic [EDGE_W-1:0] edge_sum_i, edge_max_i, edge_mean_i;
  logic [CNT_W-1:0]  edge_cnt_i;

  tile_edge_energy_4x4 #(
    .Y_W(Y_W),
    .TILE_W(TILE_W),
    .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(SUPPORT_PIX_VALID_EDGE),
    .SUPPORT_TILE_START(SUPPORT_TILE_START_EDGE),
    .ASSERT_ON(ASSERT_ON_EDGE),
    .MODE(EDGE_MODE),
    .THR(EDGE_THR),
    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W),
    .MEAN_W(EDGE_W)
  ) u_edge (
    .clk, .rst,

    .v2_valid(v2_valid),
    .v2_ready(edge_v2_ready),

    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .tile_last(tile_last),
    .tile_start(tile_start),
    .pix_in_tile_valid(pix_in_tile_valid),

    .v4_valid(edge_v4_valid),
    .v4_ready(edge_v4_ready),

    .edge_sum(edge_sum_i),
    .edge_max(edge_max_i),
    .edge_cnt(edge_cnt_i),
    .edge_mean(edge_mean_i)
  );

  // ============================================================
  // Join / handshake
  // ============================================================
  wire bundle_valid = stats_v3_valid && edge_v4_valid;
  assign out_valid  = bundle_valid;

  assign stats_v3_ready = out_ready && edge_v4_valid;
  assign edge_v4_ready  = out_ready && stats_v3_valid;

  wire join_hold = (stats_v3_valid ^ edge_v4_valid);
  assign v2_ready = edge_v2_ready && !stats_hold && !join_hold;

  // ============================================================
  // Outputs
  // ============================================================
  assign sumY   = stats_sumY;
  assign minY   = stats_minY;
  assign maxY   = stats_maxY;
  assign meanY  = stats_meanY;
  assign rangeY = stats_rangeY;

  assign stats_tile_err  = stats_tile_err_i;
  assign stats_err_code  = stats_err_code_i;
  assign stats_pix_cnt_o = stats_pix_cnt_i;

  assign y_meta_o   = stats_y_meta_o;
  assign tile_x_o   = stats_tile_x_o;
  assign tile_y_o   = stats_tile_y_o;
  assign frame_id_o = stats_frame_id_o;

  assign edge_sum  = edge_sum_i;
  assign edge_max  = edge_max_i;
  assign edge_cnt  = edge_cnt_i;
  assign edge_mean = edge_mean_i;

endmodule

`default_nettype wire
`endif
