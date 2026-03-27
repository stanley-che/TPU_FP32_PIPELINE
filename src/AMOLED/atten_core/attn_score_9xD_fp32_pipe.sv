`timescale 1ns/1ps
`default_nettype none

module attn_score_9xD_fp32 #(
  parameter int unsigned TOKENS      = 9,
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 2
)(
  input  wire                       clk,
  input  wire                       rst_n,
  input  wire                       in_valid,
  output wire                       in_ready,
  input  wire [D*32-1:0]            q_vec,
  input  wire [TOKENS*D*32-1:0]     k_vecs,
  output wire                       out_valid,
  input  wire                       out_ready,
  output wire [TOKENS*32-1:0]       score_flat
);

  localparam int unsigned SCORE_W = TOKENS*32;

  // 1/sqrt(8) = 0.3535533906 ~= 32'h3eb504f3
  // 若 D 會變，建議改成 generate + case
  localparam [31:0] INV_SQRT_D_FP32 =
    (D == 1)  ? 32'h3f800000 :
    (D == 2)  ? 32'h3f3504f3 :
    (D == 4)  ? 32'h3f000000 :
    (D == 8)  ? 32'h3eb504f3 :
    (D == 16) ? 32'h3e800000 :
                32'h3f800000;

  reg               s0_vld;
  reg [SCORE_W-1:0] s0_dat;
  wire              s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  integer t, d;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= '0;
    end else begin
      if (do_accept) begin
        // 這裡先保留結果暫存
        // 真正的乘加請放到下方 combinational DW network 或多拍 pipeline
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  // 你需要在這裡建立每個 token 的 dot product:
  // prod[t][d] = q[d] * k[t][d]
  // sum[t]     = Σ prod[t][d]
  // score[t]   = sum[t] * INV_SQRT_D_FP32
  //
  // 建議用 generate instantiate:
  // - DW_fp_mult
  // - DW_fp_add
  //
  // 最後把 score[t] pack 到 s0_dat

  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid    = s0_vld;
      assign s0_out_ready = out_ready;
      assign score_flat   = s0_dat;
    end else begin : g_pipe
      rv_pipe #(
        .WIDTH (SCORE_W),
        .STAGES(PIPE_STAGES-1)
      ) u_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_dat),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (score_flat)
      );
    end
  endgenerate

endmodule

`default_nettype wire
