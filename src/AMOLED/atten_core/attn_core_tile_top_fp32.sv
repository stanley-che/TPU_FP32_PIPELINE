`timescale 1ns/1ps
`default_nettype none

`include "./src/AMOLED/atten_core/attn_m1_m4_chain_top_pipe.sv"

module attn_core_tile_top_fp32 #(
  parameter int unsigned TOKENS           = 9,
  parameter int unsigned D                = 8,

  parameter int unsigned M1_PIPE_STAGES   = 2,
  parameter int unsigned M2_PIPE_STAGES   = 1,
  parameter int unsigned M3_PIPE_STG      = 1,
  parameter int unsigned M4_PIPE_STG      = 1,

  parameter int unsigned W_PIPE_STAGES    = 1,
  parameter int unsigned V_FIFO_DEPTH     = 16,

  parameter int unsigned ALPHA_MODE       = 1,
  parameter logic [31:0] ALPHA_SCALE_FP32 = 32'h3F800000, // 1.0
  parameter logic [31:0] ALPHA_BIAS_FP32  = 32'h00000000, // 0.0
  parameter logic [31:0] ALPHA_SIG_A_FP32 = 32'h3F800000, // 1.0

  parameter bit          TAG_EN           = 1'b1,
  parameter int unsigned TAG_W            = 16,
  parameter int unsigned TAG_FIFO_DEPTH   = 32
)(
  input  wire                         clk,
  input  wire                         rst_n,

  input  wire                         in_valid,
  output wire                         in_ready,
  input  wire [D*32-1:0]              q_vec,
  input  wire [TOKENS*D*32-1:0]       k_vecs,
  input  wire [TOKENS*D*32-1:0]       v_vecs,
  input  wire [TAG_W-1:0]             in_tag,

  output wire                         out_valid,
  input  wire                         out_ready,
  output wire [31:0]                  alpha_fp32,
  output wire [D*32-1:0]              out_vec_dbg,
  output wire [TAG_W-1:0]             out_tag,

  output wire [TOKENS*32-1:0]         score_flat_dbg,
  output wire [TOKENS*32-1:0]         w_flat_dbg
);

  localparam int unsigned OUT_W = D*32;

  wire chain_in_ready;
  wire chain_out_valid;
  wire chain_out_ready;

  wire [31:0] chain_alpha_fp32;
  wire [OUT_W-1:0] chain_out_vec_dbg;

  wire [TOKENS*32-1:0] chain_score_flat_dbg;
  wire [TOKENS*32-1:0] chain_w_flat_dbg;

  reg [TAG_W-1:0] tag_fifo [0:TAG_FIFO_DEPTH-1];
  integer tag_wr, tag_rd;
  integer tag_cnt;

  wire tag_fifo_full  = (tag_cnt >= TAG_FIFO_DEPTH);
  wire tag_fifo_empty = (tag_cnt == 0);

  wire in_fire;
  wire out_fire;

  assign in_ready = chain_in_ready && (!TAG_EN || !tag_fifo_full);
  assign in_fire  = in_valid && in_ready;

  assign chain_out_ready = out_ready && (!TAG_EN || !tag_fifo_empty);
  assign out_fire        = out_valid && out_ready;

  assign out_valid  = chain_out_valid && (!TAG_EN || !tag_fifo_empty);

  assign alpha_fp32  = chain_alpha_fp32;
  assign out_vec_dbg = chain_out_vec_dbg;
  assign out_tag     = (TAG_EN) ? tag_fifo[tag_rd] : {TAG_W{1'b0}};

  assign score_flat_dbg = chain_score_flat_dbg;
  assign w_flat_dbg     = chain_w_flat_dbg;

  attn_m1_m4_chain_top_pipe #(
    .TOKENS           (TOKENS),
    .D                (D),
	.ELEM_W           (32),
    .M1_PIPE_STAGES   (M1_PIPE_STAGES),
    .M2_PIPE_STAGES   (M2_PIPE_STAGES),
    .M3_PIPE_STG      (M3_PIPE_STG),
    .M4_PIPE_STG      (M4_PIPE_STG),
    .W_PIPE_STAGES    (W_PIPE_STAGES),
    .V_FIFO_DEPTH     (V_FIFO_DEPTH),
    .ALPHA_MODE       (ALPHA_MODE),
    .ALPHA_SCALE_FP32 (ALPHA_SCALE_FP32),
    .ALPHA_BIAS_FP32  (ALPHA_BIAS_FP32),
    .ALPHA_SIG_A_FP32 (ALPHA_SIG_A_FP32)
  ) u_chain (
    .clk            (clk),
    .rst_n          (rst_n),
    .in_valid       (in_valid),
    .in_ready       (chain_in_ready),
    .q_vec          (q_vec),
    .k_vecs         (k_vecs),
    .v_vecs         (v_vecs),
    .out_valid      (chain_out_valid),
    .out_ready      (chain_out_ready),
    .out_vec_dbg    (chain_out_vec_dbg),
    .alpha_fp32     (chain_alpha_fp32),
    .score_flat_dbg (chain_score_flat_dbg),
    .w_flat_dbg     (chain_w_flat_dbg)
  );

  integer i;
  wire push_tag = (TAG_EN) ? in_fire  : 1'b0;
  wire pop_tag  = (TAG_EN) ? out_fire : 1'b0;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tag_wr  <= 0;
      tag_rd  <= 0;
      tag_cnt <= 0;
      for (i = 0; i < TAG_FIFO_DEPTH; i = i + 1)
        tag_fifo[i] <= {TAG_W{1'b0}};
    end else begin
      if (push_tag) begin
        tag_fifo[tag_wr] <= in_tag;
      end

      case ({push_tag, pop_tag})
        2'b10: begin
          tag_wr  <= (tag_wr + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt + 1;
        end
        2'b01: begin
          tag_rd  <= (tag_rd + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt - 1;
        end
        2'b11: begin
          tag_wr  <= (tag_wr + 1) % TAG_FIFO_DEPTH;
          tag_rd  <= (tag_rd + 1) % TAG_FIFO_DEPTH;
          tag_cnt <= tag_cnt;
        end
        default: begin
          tag_cnt <= tag_cnt;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
