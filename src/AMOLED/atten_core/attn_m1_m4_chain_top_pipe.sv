`include "./src/AMOLED/atten_core/attn_score_9xD_fp32_pipe.sv"
`include "./src/AMOLED/atten_core/attn_norm_relu_fp32.sv"
`include "./src/AMOLED/atten_core/attn_m3_m4_top.sv"
`timescale 1ns/1ps
`default_nettype none

module attn_m1_m4_chain_top_pipe #(
  parameter int unsigned TOKENS          = 9,
  parameter int unsigned D               = 8,
  parameter int unsigned M1_PIPE_STAGES  = 2,
  parameter int unsigned M2_PIPE_STAGES  = 1,
  parameter int unsigned M3_PIPE_STG     = 1,
  parameter int unsigned M4_PIPE_STG     = 1,

  // pipeline between M2 -> M3/M4 (bundle={w,v})
  parameter int unsigned W_PIPE_STAGES   = 1,   // 0=bypass, >=1 elastic stages
  parameter int unsigned V_FIFO_DEPTH    = 16,  // >= possible in-flight count

  parameter int unsigned ALPHA_MODE      = 1,
  parameter real         ALPHA_SCALE     = 1.0,
  parameter real         ALPHA_BIAS      = 0.0,
  parameter real         ALPHA_SIG_A     = 1.0
)(
  input  wire                         clk,
  input  wire                         rst_n,

  // input
  input  wire                         in_valid,
  output wire                         in_ready,
  input  wire [D*32-1:0]              q_vec,
  input  wire [TOKENS*D*32-1:0]       k_vecs,
  input  wire [TOKENS*D*32-1:0]       v_vecs,

  // output
  output wire                         out_valid,
  input  wire                         out_ready,
  output wire [D*32-1:0]              out_vec_dbg,
  output wire [31:0]                  alpha_fp32,

  // debug
  output wire [TOKENS*32-1:0]         score_flat_dbg,
  output wire [TOKENS*32-1:0]         w_flat_dbg
);

  localparam int unsigned SCORE_W = TOKENS*32;
  localparam int unsigned W_W     = TOKENS*32;
  localparam int unsigned V_W     = TOKENS*D*32;

  // ----------------------------
  // M1 score
  // ----------------------------
  wire                  m1_out_valid;
  wire                  m1_out_ready;
  wire [SCORE_W-1:0]     score_flat;

  // ----------------------------
  // M2 softmax -> w_raw
  // ----------------------------
  wire                  m2_out_valid;
  wire                  m2_out_ready;
  wire [W_W-1:0]         w_raw;

  // ----------------------------
  // v_vecs FIFO (order-preserving)
  // push: input accepted
  // pop : wv bundle accepted
  // ----------------------------
  reg  [V_W-1:0] v_fifo [0:V_FIFO_DEPTH-1];
  integer v_wr, v_rd;
  integer v_cnt;

  wire v_fifo_full  = (v_cnt >= V_FIFO_DEPTH);
  wire v_fifo_empty = (v_cnt == 0);

  // accept input only if M1 can accept AND v_fifo has space
  wire m1_in_ready_raw;
  assign in_ready = m1_in_ready_raw && !v_fifo_full;
  wire in_fire    = in_valid && in_ready;

  // ----------------------------
  // Bundle {w, v_head} into pipe
  // ----------------------------
  localparam int unsigned WV_W = W_W + V_W;

  wire              wv_in_valid;
  wire              wv_in_ready;
  wire [WV_W-1:0]    wv_in_data;

  wire              wv_out_valid;
  wire              wv_out_ready;
  wire [WV_W-1:0]    wv_out_data;

  // head of v_fifo (combinational read)
  wire [V_W-1:0] v_head = v_fifo[v_rd];

  // Only accept M2 output when we have matching v available
  assign wv_in_valid  = m2_out_valid && !v_fifo_empty;
  assign wv_in_data   = {w_raw, v_head};
  assign m2_out_ready = wv_in_ready && !v_fifo_empty;

  wire wv_fire = wv_in_valid && wv_in_ready;

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

  // Connect to M3/M4
  wire              m34_in_valid = wv_out_valid;
  wire              m34_in_ready;
  assign wv_out_ready = m34_in_ready;

  wire [W_W-1:0] w_to_m34 = wv_out_data[WV_W-1 -: W_W];
  wire [V_W-1:0] v_to_m34 = wv_out_data[V_W-1:0];

  // debug
  assign score_flat_dbg = score_flat;
  assign w_flat_dbg     = w_raw;

  // ----------------------------
  // Instantiate M1
  // ----------------------------
  attn_score_9xD_fp32 #(
    .TOKENS      (TOKENS),
    .D           (D),
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
  // Instantiate M2
  // ----------------------------
  attn_softmax_exp2_fp32 #(
    .TOKENS      (TOKENS),
    .PIPE_STAGES (M2_PIPE_STAGES),
    .XCLAMP      (8.0)
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
  // Instantiate M3/M4
  // ----------------------------
  attn_m3_m4_top #(
    .TOKENS      (TOKENS),
    .D           (D),
    .M3_PIPE_STG (M3_PIPE_STG),
    .M4_PIPE_STG (M4_PIPE_STG),
    .ALPHA_MODE  (ALPHA_MODE),
    .ALPHA_SCALE (ALPHA_SCALE),
    .ALPHA_BIAS  (ALPHA_BIAS),
    .ALPHA_SIG_A (ALPHA_SIG_A)
  ) u_m34 (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (m34_in_valid),
    .in_ready   (m34_in_ready),
    .w_flat     (w_to_m34),
    .v_vecs     (v_to_m34),
    .out_valid  (out_valid),
    .out_ready  (out_ready),
    .alpha_fp32 (alpha_fp32),
    .out_vec_dbg(out_vec_dbg)
  );

  // ----------------------------
  // v_fifo control (FIXED: handle push/pop same cycle correctly)
  // ----------------------------
  wire push_v = in_fire;
  wire pop_v  = wv_fire;

  integer i;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v_wr  <= 0;
      v_rd  <= 0;
      v_cnt <= 0;
      for (i=0; i<V_FIFO_DEPTH; i=i+1) v_fifo[i] <= '0;
    end else begin
      // write on push
      if (push_v) begin
        v_fifo[v_wr] <= v_vecs;
      end

      // pointers + count update (single write to v_cnt)
      case ({push_v, pop_v})
        2'b10: begin // push only
          v_wr  <= (v_wr + 1) % V_FIFO_DEPTH;
          v_cnt <= v_cnt + 1;
        end
        2'b01: begin // pop only
          v_rd  <= (v_rd + 1) % V_FIFO_DEPTH;
          v_cnt <= v_cnt - 1;
        end
        2'b11: begin // push and pop
          v_wr  <= (v_wr + 1) % V_FIFO_DEPTH;
          v_rd  <= (v_rd + 1) % V_FIFO_DEPTH;
          v_cnt <= v_cnt; // unchanged
        end
        default: begin
          v_cnt <= v_cnt;
        end
      endcase
    end
  end

  // (optional) debug guardrails
  // always @(posedge clk) if (rst_n) begin
  //   if (pop_v && v_fifo_empty) begin
  //     $display("[M14][FATAL] v_fifo underflow!");
  //     $finish;
  //   end
  //   if (push_v && v_fifo_full) begin
  //     $display("[M14][FATAL] v_fifo overflow!");
  //     $finish;
  //   end
  // end

endmodule

`default_nettype wire
