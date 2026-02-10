// ============================================================
// attn_m1_m2_chain_top.sv
// - Chain by instances:
//   M1: attn_score_9xD_fp32        -> score_flat
//   M2: attn_norm_relu_fp32     -> w_flat   (softmax approx)
// - Proper ready/valid backpressure propagation
// ============================================================

`include "./src/AMOLED/atten_core/attn_score_9xD_fp32_pipe.sv"
`include "./src/AMOLED/atten_core/attn_norm_relu_fp32.sv"

`timescale 1ns/1ps
`default_nettype none

module attn_m1_m2_chain_top #(
  parameter int unsigned TOKENS          = 9,
  parameter int unsigned D               = 8,
  parameter int unsigned M1_PIPE_STAGES  = 2,
  parameter int unsigned M2_PIPE_STAGES  = 1
)(
  input  wire                       clk,
  input  wire                       rst_n,

  // input (Q, K)
  input  wire                       in_valid,
  output wire                       in_ready,
  input  wire [D*32-1:0]            q_vec,
  input  wire [TOKENS*D*32-1:0]     k_vecs,

  // output (weights)
  output wire                       out_valid,
  input  wire                       out_ready,
  output wire [TOKENS*32-1:0]       w_flat,

  // optional debug: expose score
  output wire [TOKENS*32-1:0]       score_flat_dbg
);

  wire                  m1_out_valid;
  wire                  m1_out_ready;
  wire [TOKENS*32-1:0]  score_flat;

  // ----------------------------
  // M1: score = dot(Q, K[t]) / sqrt(D)
  // ----------------------------
  attn_score_9xD_fp32 #(
    .TOKENS      (TOKENS),
    .D           (D),
    .PIPE_STAGES (M1_PIPE_STAGES)
  ) u_m1_score (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (in_valid),
    .in_ready   (in_ready),
    .q_vec      (q_vec),
    .k_vecs     (k_vecs),
    .out_valid  (m1_out_valid),
    .out_ready  (m1_out_ready),
    .score_flat (score_flat)
  );

  // ----------------------------
  // M2: w = softmax(score)
  //      (exp2 approximation, stable, non-sparse)
  // ----------------------------
  attn_softmax_exp2_fp32 #(
    .TOKENS      (TOKENS),
    .PIPE_STAGES (M2_PIPE_STAGES),
    .XCLAMP      (8.0)   // 可調：8~10 都很合理
  ) u_m2_softmax (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (m1_out_valid),
    .in_ready   (m1_out_ready),   // backpressure propagates here
    .score_flat (score_flat),
    .out_valid  (out_valid),
    .out_ready  (out_ready),
    .w_flat     (w_flat)
  );

  // debug tap
  assign score_flat_dbg = score_flat;

endmodule

`default_nettype wire
