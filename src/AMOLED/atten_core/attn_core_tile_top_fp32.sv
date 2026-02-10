// ============================================================
// attn_core_tile_top_fp32.sv  (IVERILOG-SAFE)
// - M5: Attention 核心 Top : score -> weight -> out -> alpha
//
// 功能：
//   將 9 tokens 的 bundle (Q/K/V) 輸入後，輸出每個 tile 一筆 alpha_fp32 (+ 可選 out_tag)
//   score = Q*K^T
//   w     = softmax(score) (你的 M2 實作：exp2 approx + clamp)
//   out   = Σ w*V
//   alpha = g(out)         (你的 M4)
//
// I/O：
//   in_valid/in_ready
//   q_vec   : D*FP32
//   k_vecs  : TOKENS*D*FP32
//   v_vecs  : TOKENS*D*FP32
//   (optional) in_tag -> out_tag (order-preserving)
//
// 注意：
// - out_valid && !out_ready 時，DUT 內部已保證 out_vec_dbg/alpha hold。
// - 本 Top 額外保證 out_tag 也 hold（tag_fifo 只在 out_fire pop）。
// ============================================================

`timescale 1ns/1ps
`default_nettype none

`include "./src/AMOLED/atten_core/attn_m1_m4_chain_top_pipe.sv"

module attn_core_tile_top_fp32 #(
  parameter int unsigned TOKENS          = 9,
  parameter int unsigned D               = 8,

  // pass-through to chain
  parameter int unsigned M1_PIPE_STAGES  = 2,
  parameter int unsigned M2_PIPE_STAGES  = 1,
  parameter int unsigned M3_PIPE_STG     = 1,
  parameter int unsigned M4_PIPE_STG     = 1,

  parameter int unsigned W_PIPE_STAGES   = 1,   // 0=bypass, >=1 elastic stages
  parameter int unsigned V_FIFO_DEPTH    = 16,  // >= possible in-flight count

  parameter int unsigned ALPHA_MODE      = 1,
  parameter real         ALPHA_SCALE     = 1.0,
  parameter real         ALPHA_BIAS      = 0.0,
  parameter real         ALPHA_SIG_A     = 1.0,

  // optional tag
  parameter bit          TAG_EN          = 1'b1,
  parameter int unsigned TAG_W           = 16,
  parameter int unsigned TAG_FIFO_DEPTH  = 32   // should be >= max in-flight
)(
  input  wire                         clk,
  input  wire                         rst_n,

  // input bundle
  input  wire                         in_valid,
  output wire                         in_ready,
  input  wire [D*32-1:0]              q_vec,
  input  wire [TOKENS*D*32-1:0]       k_vecs,
  input  wire [TOKENS*D*32-1:0]       v_vecs,
  input  wire [TAG_W-1:0]             in_tag,

  // output
  output wire                         out_valid,
  input  wire                         out_ready,
  output wire [31:0]                  alpha_fp32,
  output wire [D*32-1:0]              out_vec_dbg,
  output wire [TAG_W-1:0]             out_tag,

  // debug (optional probe)
  output wire [TOKENS*32-1:0]         score_flat_dbg,
  output wire [TOKENS*32-1:0]         w_flat_dbg
);

  localparam int unsigned OUT_W = D*32;

  // ----------------------------
  // Chain instance (M1->M4)
  // ----------------------------
  wire chain_in_ready;

  wire chain_out_valid;
  wire chain_out_ready;

  wire [31:0] chain_alpha_fp32;
  wire [OUT_W-1:0] chain_out_vec_dbg;

  wire [TOKENS*32-1:0] chain_score_flat_dbg;
  wire [TOKENS*32-1:0] chain_w_flat_dbg;

  // ----------------------------
  // Tag FIFO (order-preserving)
  // push on input fire
  // pop  on output fire
  // ----------------------------
  reg [TAG_W-1:0] tag_fifo [0:TAG_FIFO_DEPTH-1];
  integer tag_wr, tag_rd;
  integer tag_cnt;

  wire tag_fifo_full  = (tag_cnt >= TAG_FIFO_DEPTH);
  wire tag_fifo_empty = (tag_cnt == 0);

  wire in_fire;
  wire out_fire;

  // in_ready: chain ready AND tag_fifo has space (if TAG_EN)
  assign in_ready = chain_in_ready && (!TAG_EN || !tag_fifo_full);
  assign in_fire  = in_valid && in_ready;

  // we should not let chain output be accepted unless tag is available (if TAG_EN)
  assign chain_out_ready = out_ready && (!TAG_EN || !tag_fifo_empty);
  assign out_fire        = out_valid && out_ready;

  // out_valid should also be gated by tag availability (if TAG_EN)
  assign out_valid  = chain_out_valid && (!TAG_EN || !tag_fifo_empty);

  // out data
  assign alpha_fp32  = chain_alpha_fp32;
  assign out_vec_dbg = chain_out_vec_dbg;

  // out_tag: hold stable when stalled
  // - when TAG_EN=0, drive zero.
  // - when TAG_EN=1, combinational head of fifo; pop only on out_fire.
  assign out_tag = (TAG_EN) ? tag_fifo[tag_rd] : {TAG_W{1'b0}};

  // debug passthrough (note: not aligned to output timing; for waveform only)
  assign score_flat_dbg = chain_score_flat_dbg;
  assign w_flat_dbg     = chain_w_flat_dbg;

  // ----------------------------
  // Instantiate chain
  // ----------------------------
  attn_m1_m4_chain_top_pipe #(
    .TOKENS         (TOKENS),
    .D              (D),
    .M1_PIPE_STAGES (M1_PIPE_STAGES),
    .M2_PIPE_STAGES (M2_PIPE_STAGES),
    .M3_PIPE_STG    (M3_PIPE_STG),
    .M4_PIPE_STG    (M4_PIPE_STG),
    .W_PIPE_STAGES  (W_PIPE_STAGES),
    .V_FIFO_DEPTH   (V_FIFO_DEPTH),
    .ALPHA_MODE     (ALPHA_MODE),
    .ALPHA_SCALE    (ALPHA_SCALE),
    .ALPHA_BIAS     (ALPHA_BIAS),
    .ALPHA_SIG_A    (ALPHA_SIG_A)
  ) u_chain (
    .clk           (clk),
    .rst_n         (rst_n),

    .in_valid      (in_valid),
    .in_ready      (chain_in_ready),
    .q_vec         (q_vec),
    .k_vecs        (k_vecs),
    .v_vecs        (v_vecs),

    .out_valid     (chain_out_valid),
    .out_ready     (chain_out_ready),
    .out_vec_dbg   (chain_out_vec_dbg),
    .alpha_fp32    (chain_alpha_fp32),

    .score_flat_dbg(chain_score_flat_dbg),
    .w_flat_dbg    (chain_w_flat_dbg)
  );

  // ----------------------------
  // Tag FIFO control
  // ----------------------------
  integer i;
  wire push_tag = (TAG_EN) ? in_fire  : 1'b0;
  wire pop_tag  = (TAG_EN) ? out_fire : 1'b0;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tag_wr  <= 0;
      tag_rd  <= 0;
      tag_cnt <= 0;
      for (i=0; i<TAG_FIFO_DEPTH; i=i+1) tag_fifo[i] <= '0;
    end else begin
      if (push_tag) begin
        tag_fifo[tag_wr] <= in_tag;
      end

      case ({push_tag, pop_tag})
        2'b10: begin // push only
          tag_wr  <= (tag_wr + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt + 1;
        end
        2'b01: begin // pop only
          tag_rd  <= (tag_rd + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt - 1;
        end
        2'b11: begin // push and pop
          tag_wr  <= (tag_wr + 1) % TAG_FIFO_DEPTH;
          tag_rd  <= (tag_rd + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt; // unchanged
        end
        default: begin
          tag_cnt <= tag_cnt;
        end
      endcase
    end
  end

  // optional guardrails
  // always @(posedge clk) if (rst_n && TAG_EN) begin
  //   if (push_tag && tag_fifo_full) begin
  //     $display("[M5][FATAL] tag_fifo overflow!");
  //     $finish;
  //   end
  //   if (pop_tag && tag_fifo_empty) begin
  //     $display("[M5][FATAL] tag_fifo underflow!");
  //     $finish;
  //   end
  // end

endmodule

`default_nettype wire
