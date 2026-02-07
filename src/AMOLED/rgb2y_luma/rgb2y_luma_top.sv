// rgb2y_luma_top.sv
// Integrate:
//   rgb2y_luma_top (RGB->Y10)  +  rgb2y_out_pack (Y10->Yout + sideband + clip + elastic)
//
// Handshake:
//   upstream:  pix_valid / in_ready
//   downstream: out_valid / out_ready
//
// Key wiring:
//   full_pipe.out_ready  <= out_pack.in_ready
//   out_pack.in_valid    <= full_pipe.v4_valid
//   out_pack.y_in        <= full_pipe.y10
//   top.in_ready         <= full_pipe.in_ready
//   top.out_valid/Y/sb   <= out_pack outputs

`ifndef RGB2Y_FULL_PIPE_WITH_PACK_TOP_SV
`define RGB2Y_FULL_PIPE_WITH_PACK_TOP_SV

`include "./src/AMOLED/rgb2y_luma/rgb2y_full_pipe_top.sv"
`include "./src/AMOLED/rgb2y_luma/rgb2y_out_pack.sv"

`timescale 1ns/1ps
`default_nettype none

module rgb2y_luma_top #(
  // ----------------------------
  // rgb2y_full_pipe_top params
  // ----------------------------
  parameter int unsigned FORMAT = 0,      // 0 RGB888, 1 RGB101010, 2 RGB121212, 3 RGB565
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

  // ----------------------------
  // rgb2y_out_pack params
  // ----------------------------
  parameter int unsigned PACK_IN_W  = 10,  // should match full_pipe y10 width (=10)
  parameter int unsigned PACK_OUT_W = 10,  // output width you want (e.g. 10)
  parameter int unsigned SB_W       = 0,   // sideband width (0 ok)
  parameter int unsigned PACK_LAT   = 0,   // elastic depth = PACK_LAT+1

  parameter bit          PACK_GATE_SIDEBAND_WITH_VALID = 1'b1,
  parameter bit          PACK_USE_ROUND = 1'b0,
  parameter bit          PACK_USE_SAT   = 1'b1,
  parameter bit          PACK_USE_CLIP  = 1'b0,
  parameter bit          PACK_COUNT_DBG = 1'b0
)(
  input  logic                 clk,
  input  logic                 rst,

  // global enable
  input  logic                 en,

  // ----------------------------
  // input RGB stream
  // ----------------------------
  input  logic                 pix_valid,
  input  logic [35:0]          rgb_in,
  input  logic [SB_W-1:0]      sb_in,     // optional: sof/eol/vs/hs/de/x/y...
  output logic                 in_ready,

  // ----------------------------
  // output stream (post-pack)
  // ----------------------------
  output logic                 out_valid,
  input  logic                 out_ready,
  output logic [PACK_OUT_W-1:0] Y,
  output logic [SB_W-1:0]      sb_out,

  // ----------------------------
  // clip controls (for out_pack)
  // ----------------------------
  input  logic                 clip_en,
  input  logic [PACK_OUT_W-1:0] clip_min,
  input  logic [PACK_OUT_W-1:0] clip_max,

  // ----------------------------
  // optional debug from full_pipe
  // ----------------------------
  output logic                 v1_valid,
  output logic [7:0]           r8,
  output logic [7:0]           g8,
  output logic [7:0]           b8,

  output logic                 v2_valid,
  output logic [15:0]          r_term,
  output logic [15:0]          g_term,
  output logic [15:0]          b_term,

  output logic                 v4_valid,
  output logic [16:0]          sum17,
  output logic [9:0]           y10,
  output logic [7:0]           y8,
  output logic                 ovf_sum,
  output logic                 ovf_y,

  // ----------------------------
  // optional debug from out_pack
  // ----------------------------
  output logic                 drop_pulse,
  output logic                 stall_pulse,
  output logic [31:0]          drop_cnt,
  output logic [31:0]          stall_cnt
);

  // ============================================================
  // Stage A: RGB -> Y10 (ready/valid)
  // ============================================================
  logic fp_out_ready; // backpressure into full_pipe (from pack.in_ready)

  rgb2y_full_pipe_top #(
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
    .Y8_MODE(Y8_MODE)
  ) u_full_pipe (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .rgb_in(rgb_in),

    .out_ready(fp_out_ready), 
    .in_ready(in_ready),

    .v4_valid(v4_valid),
    .sum17(sum17),
    .y10(y10),
    .y8(y8),
    .ovf_sum(ovf_sum),
    .ovf_y(ovf_y),

    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8),

    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term)
  );

  // ============================================================
  // Stage B: Y10 -> Yout + sideband + clip + elastic (ready/valid)
  // ============================================================
  logic pack_in_ready;

  rgb2y_out_pack #(
    .IN_W(PACK_IN_W),
    .OUT_W(PACK_OUT_W),
    .SB_W(SB_W),
    .LAT(PACK_LAT),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),

    .GATE_SIDEBAND_WITH_VALID(PACK_GATE_SIDEBAND_WITH_VALID),

    .USE_ROUND(PACK_USE_ROUND),
    .USE_SAT(PACK_USE_SAT),
    .USE_CLIP(PACK_USE_CLIP),

    .COUNT_DBG(PACK_COUNT_DBG)
  ) u_pack (
    .clk(clk),
    .rst(rst),
    .en(en),

    .bypass(1'b0),              // 需要的話你也可以拉成 top 參數/輸入
    .in_valid(v4_valid),
    .in_ready(pack_in_ready),
    .y_in(y10[PACK_IN_W-1:0]),  // PACK_IN_W=10 時就是 y10
    .sb_in(sb_in),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .Y(Y),
    .sb_out(sb_out),

    .drop_pulse(drop_pulse),
    .stall_pulse(stall_pulse),
    .drop_cnt(drop_cnt),
    .stall_cnt(stall_cnt)
  );

  // backpressure chain: pack -> full_pipe
  assign fp_out_ready = pack_in_ready;


endmodule

`default_nettype wire
`endif
