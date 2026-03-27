`include "./src/AMOLED/atten_core/attn_score_9xD_fp32_pipe.sv"
`include "./src/AMOLED/atten_core/attn_norm_relu_fp32.sv"
`include "./src/AMOLED/atten_core/attn_m3_m4_top.sv"

`timescale 1ns/1ps
`default_nettype none

module attn_m1_m4_chain_top_pipe #(
  parameter integer TOKENS          = 9,
  parameter integer D               = 8,
  parameter integer ELEM_W          = 16,
  parameter integer FRAC_W          = 12,
  parameter integer ACC_W           = 40,

  parameter integer M1_PIPE_STAGES  = 2,
  parameter integer M2_PIPE_STAGES  = 1,
  parameter integer M3_PIPE_STG     = 1,
  parameter integer M4_PIPE_STG     = 1,

  parameter integer W_PIPE_STAGES   = 1,
  parameter integer V_FIFO_DEPTH    = 1,
  parameter integer ALPHA_MODE = 0,
 parameter [31:0] ALPHA_SCALE_FP32  = 32'h3f800000,
  parameter [31:0] ALPHA_BIAS_FP32   = 32'h00000000,
  parameter [31:0] ALPHA_SIG_A_FP32  = 32'h3f800000
  // 如果之後 alpha 要做��複��近似，
  // 可以再加 integer/fixed-point 參數
)(
  input  wire                               clk,
  input  wire                               rst_n,

  input  wire                               in_valid,
  output wire                               in_ready,
  input  wire [D*ELEM_W-1:0]                q_vec,
  input  wire [TOKENS*D*ELEM_W-1:0]         k_vecs,
  input  wire [TOKENS*D*ELEM_W-1:0]         v_vecs,

  output wire                               out_valid,
  input  wire                               out_ready,
  output wire [D*ELEM_W-1:0]                out_vec_dbg,
  output wire [ELEM_W-1:0]                  alpha_fp32,

  output wire [TOKENS*ELEM_W-1:0]           score_flat_dbg,
  output wire [TOKENS*ELEM_W-1:0]           w_flat_dbg
);

  localparam integer SCORE_W = TOKENS * ELEM_W;
  localparam integer W_W     = TOKENS * ELEM_W;
  localparam integer V_W     = TOKENS * D * ELEM_W;
  localparam integer WV_W    = W_W + V_W;

  // ----------------------------
  // M1
  // ----------------------------
  wire                        m1_out_valid;
  wire                        m1_out_ready;
  wire [SCORE_W-1:0]          score_flat;

  // ----------------------------
  // M2
  // ----------------------------
  wire                        m2_out_valid;
  wire                        m2_out_ready;
  wire [W_W-1:0]              w_raw;

  // ----------------------------
  // v_fifo
  // ----------------------------
  reg  [V_W-1:0]              v_fifo [0:V_FIFO_DEPTH-1];
  integer                     v_wr, v_rd;
  integer                     v_cnt;

  wire                        v_fifo_full;
  wire                        v_fifo_empty;

  assign v_fifo_full  = (v_cnt >= V_FIFO_DEPTH);
  assign v_fifo_empty = (v_cnt == 0);

  wire                        m1_in_ready_raw;
  assign in_ready = m1_in_ready_raw && !v_fifo_full;
  wire                        in_fire = in_valid && in_ready;

  // ----------------------------
  // w/v bundle
  // ----------------------------
  wire                        wv_in_valid;
  wire                        wv_in_ready;
  wire [WV_W-1:0]             wv_in_data;

  wire                        wv_out_valid;
  wire                        wv_out_ready;
  wire [WV_W-1:0]             wv_out_data;

  wire [V_W-1:0]              v_head;
  assign v_head = v_fifo[v_rd];

  assign wv_in_valid = m2_out_valid && !v_fifo_empty;
  assign wv_in_data  = {w_raw, v_head};
  assign m2_out_ready = wv_in_ready && !v_fifo_empty;

  wire                        wv_fire = wv_in_valid && wv_in_ready;

  generate
    if (W_PIPE_STAGES == 0) begin : g_wv_bypass
      assign wv_out_valid = wv_in_valid;
      assign wv_in_ready  = wv_out_ready;
      assign wv_out_data  = wv_in_data;
    end else begin : g_wv_pipe
      rv_pipe #(
        .WIDTH (WV_W),
        .STAGES(W_PIPE_STAGES)
      ) u_wv_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (wv_in_valid),
        .in_ready  (wv_in_ready),
        .in_data   (wv_in_data),
        .out_valid (wv_out_valid),
        .out_ready (wv_out_ready),
        .out_data  (wv_out_data)
      );
    end
  endgenerate

  // ----------------------------
  // connect to M3/M4
  // ----------------------------
  wire                        m34_in_valid;
  wire                        m34_in_ready;
  wire [W_W-1:0]              w_to_m34;
  wire [V_W-1:0]              v_to_m34;

  assign m34_in_valid = wv_out_valid;
  assign wv_out_ready = m34_in_ready;

  assign w_to_m34 = wv_out_data[WV_W-1 -: W_W];
  assign v_to_m34 = wv_out_data[V_W-1:0];

  assign score_flat_dbg = score_flat;
  assign w_flat_dbg     = w_raw;

  // ----------------------------
  // M1: fixed-point score
  // ----------------------------
  attn_score_9xD_fp32 #(
    .TOKENS      (TOKENS),
    .D           (D),
   // .ELEM_W      (ELEM_W),
   // .FRAC_W      (FRAC_W),
   // .ACC_W       (ACC_W),
    .PIPE_STAGES (M1_PIPE_STAGES)
  ) u_m1 (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (in_valid),
    .in_ready   (m1_in_ready_raw),
    .q_vec      (q_vec),
    .k_vecs     (k_vecs),
    .out_valid  (m1_out_valid),
    .out_ready  (m1_out_ready),
    .score_flat (score_flat)
  );

  // ----------------------------
  // M2: fixed-point softmax
  // ----------------------------
  attn_softmax_exp2_fp32 #(
    .TOKENS      (TOKENS),
    .ELEM_W      (ELEM_W),
    .FRAC_W      (FRAC_W),
    .ACC_W       (24),
    .PIPE_STAGES (M2_PIPE_STAGES)
  ) u_m2 (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (m1_out_valid),
    .in_ready   (m1_out_ready),
    .score_flat (score_flat),
    .out_valid  (m2_out_valid),
    .out_ready  (m2_out_ready),
    .w_flat     (w_raw)
  );

  // ----------------------------
  // M3/M4: fixed-point chain
  // ----------------------------
  attn_m3_m4_top #(
    .TOKENS      (TOKENS),
    .D           (D),
    .ELEM_W      (ELEM_W),
    .FRAC_W      (FRAC_W),
    .ACC_W       (ACC_W),
    .M3_PIPE_STG (M3_PIPE_STG),
    .M4_PIPE_STG (M4_PIPE_STG)
  ) u_m34 (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (m34_in_valid),
    .in_ready   (m34_in_ready),
    .w_flat     (w_to_m34),
    .v_vecs     (v_to_m34),
    .out_valid  (out_valid),
    .out_ready  (out_ready),
    .alpha_fp32   (alpha_fp32),
    .out_vec_dbg(out_vec_dbg)
  );

  // ----------------------------
  // v_fifo control
  // ----------------------------
  wire push_v = in_fire;
  wire pop_v  = wv_fire;

  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_wr  <= 0;
      v_rd  <= 0;
      v_cnt <= 0;
      for (i = 0; i < V_FIFO_DEPTH; i = i + 1)
        v_fifo[i] <= {V_W{1'b0}};
    end else begin
      if (push_v)
        v_fifo[v_wr] <= v_vecs;

      case ({push_v, pop_v})
        2'b10: begin
          if (v_wr == V_FIFO_DEPTH-1) v_wr <= 0;
          else                        v_wr <= v_wr + 1;
          v_cnt <= v_cnt + 1;
        end
        2'b01: begin
          if (v_rd == V_FIFO_DEPTH-1) v_rd <= 0;
          else                        v_rd <= v_rd + 1;
          v_cnt <= v_cnt - 1;
        end
        2'b11: begin
          if (v_wr == V_FIFO_DEPTH-1) v_wr <= 0;
          else                        v_wr <= v_wr + 1;

          if (v_rd == V_FIFO_DEPTH-1) v_rd <= 0;
          else                        v_rd <= v_rd + 1;

          v_cnt <= v_cnt;
        end
        default: begin
          v_cnt <= v_cnt;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
