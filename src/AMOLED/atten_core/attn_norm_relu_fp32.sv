`ifndef ATTN_SOFTMAX_EXP2_FX_SV
`define ATTN_SOFTMAX_EXP2_FX_SV

`timescale 1ns/1ps
`default_nettype none

module attn_softmax_exp2_fp32 #(
  parameter integer TOKENS      = 9,
  parameter integer ELEM_W      = 16,   // fixed-point width
  parameter integer FRAC_W      = 12,   // Q4.12
  parameter integer ACC_W       = 24,   // sum width
  parameter integer PIPE_STAGES = 1
)(
  input  wire                            clk,
  input  wire                            rst_n,

  input  wire                            in_valid,
  output wire                            in_ready,
  input  wire [TOKENS*ELEM_W-1:0]        score_flat,

  output wire                            out_valid,
  input  wire                            out_ready,
  output wire [TOKENS*ELEM_W-1:0]        w_flat
);

  localparam integer W_W = TOKENS * ELEM_W;

  // log2(e) in Q4.12 ~= 1.442695 * 4096 = 5909
  localparam signed [15:0] LOG2E_Q = 16'sd5909;

  // ------------------------------------------------------------
  // 2^(-f), f in [0,1), 16-entry LUT, Q0.12
  // index = frac[FRAC_W-1 -: 4]
  // ------------------------------------------------------------
  function [ELEM_W-1:0] exp2_neg_frac_lut;
    input [3:0] idx;
    begin
      case (idx)
        4'd0:  exp2_neg_frac_lut = 16'd4096; // 1.0000
        4'd1:  exp2_neg_frac_lut = 16'd3922; // ~0.9576
        4'd2:  exp2_neg_frac_lut = 16'd3756; // ~0.9170
        4'd3:  exp2_neg_frac_lut = 16'd3597; // ~0.8780
        4'd4:  exp2_neg_frac_lut = 16'd3444; // ~0.8409
        4'd5:  exp2_neg_frac_lut = 16'd3298; // ~0.8050
        4'd6:  exp2_neg_frac_lut = 16'd3158; // ~0.7710
        4'd7:  exp2_neg_frac_lut = 16'd3024; // ~0.7384
        4'd8:  exp2_neg_frac_lut = 16'd2896; // ~0.7071
        4'd9:  exp2_neg_frac_lut = 16'd2773; // ~0.6771
        4'd10: exp2_neg_frac_lut = 16'd2656; // ~0.6484
        4'd11: exp2_neg_frac_lut = 16'd2543; // ~0.6209
        4'd12: exp2_neg_frac_lut = 16'd2436; // ~0.5946
        4'd13: exp2_neg_frac_lut = 16'd2333; // ~0.5694
        4'd14: exp2_neg_frac_lut = 16'd2234; // ~0.5453
        4'd15: exp2_neg_frac_lut = 16'd2140; // ~0.5221
        default: exp2_neg_frac_lut = 16'd4096;
      endcase
    end
  endfunction

  // ------------------------------------------------------------
  // optional rv pipe uses existing rv_pipe in your project
  // ------------------------------------------------------------
  reg                 s0_vld;
  reg [W_W-1:0]       s0_dat;
  wire                s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;

  integer i;

  reg signed [ELEM_W-1:0] score   [0:TOKENS-1];
  reg signed [ELEM_W-1:0] max_s;
  reg signed [ELEM_W-1:0] delta_s;

  reg signed [31:0] y_q;          // Q4.12 after *log2(e)
  reg [ELEM_W-1:0]  frac_term;
  integer           int_part;
  reg [ELEM_W-1:0]  a_val [0:TOKENS-1];
  reg [ACC_W-1:0]   sum_a;
  reg [W_W-1:0]     tmp_out;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= {W_W{1'b0}};
    end else begin
      if (in_valid && in_ready) begin
        // unpack
        for (i = 0; i < TOKENS; i = i + 1)
          score[i] = $signed(score_flat[i*ELEM_W +: ELEM_W]);

        // max
        max_s = $signed(score_flat[0 +: ELEM_W]);
        for (i = 1; i < TOKENS; i = i + 1) begin
          if ($signed(score_flat[i*ELEM_W +: ELEM_W]) > max_s)
            max_s = $signed(score_flat[i*ELEM_W +: ELEM_W]);
        end

        // exp approx and sum
        sum_a = {ACC_W{1'b0}};
        for (i = 0; i < TOKENS; i = i + 1) begin
          delta_s = max_s - $signed(score_flat[i*ELEM_W +: ELEM_W]); // >=0 ideally

          // y = delta * log2(e), still Q4.12
          y_q = (delta_s * LOG2E_Q) >>> FRAC_W;

          if (y_q < 0)
            y_q = 0;

          int_part  = y_q >>> FRAC_W;
          frac_term = exp2_neg_frac_lut(y_q[FRAC_W-1 -: 4]); // Q0.12

          if (int_part >= ELEM_W) begin
            a_val[i] = {ELEM_W{1'b0}};
          end else begin
            a_val[i] = frac_term >> int_part; // 2^(-I) * 2^(-F)
          end

          sum_a = sum_a + a_val[i];
        end

        // normalize -> w in Q0.12
        tmp_out = {W_W{1'b0}};
        if (sum_a == 0) begin
          // uniform = (1<<FRAC_W)/TOKENS
          for (i = 0; i < TOKENS; i = i + 1)
            tmp_out[i*ELEM_W +: ELEM_W] = (1 << FRAC_W) / TOKENS;
        end else begin
          for (i = 0; i < TOKENS; i = i + 1)
            tmp_out[i*ELEM_W +: ELEM_W] = (a_val[i] << FRAC_W) / sum_a;
        end

        s0_dat <= tmp_out;
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid    = s0_vld;
      assign s0_out_ready = out_ready;
      assign w_flat       = s0_dat;
    end else begin : g_pipe
      rv_pipe #(
        .WIDTH (W_W),
        .STAGES(PIPE_STAGES-1)
      ) u_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_dat),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (w_flat)
      );
    end
  endgenerate

endmodule

`default_nettype wire
`endif
