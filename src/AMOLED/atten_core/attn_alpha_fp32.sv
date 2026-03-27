`timescale 1ns/1ps
`default_nettype none

module attn_alpha_fp32 #(
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 1,
  parameter int unsigned MODE        = 1
)(
  input  wire                  clk,
  input  wire                  rst_n,

  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [D*32-1:0]       out_vec,

  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [31:0]           alpha_fp32
);

  // ----------------------------------------------------------
  // Stage0 elastic buffer
  // ----------------------------------------------------------
  reg        s0_vld;
  reg [31:0] s0_dat;
  wire       s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  // pick out_vec[0]
  wire [31:0] out0_bits = out_vec[31:0];

  // ----------------------------------------------------------
  // FP32 clamp to [0,1] using bit inspection only
  //
  // x < 0      => 0.0
  // 0 <= x <=1 => x
  // x > 1      => 1.0
  //
  // IEEE754 single:
  // sign = [31]
  // exp  = [30:23]
  // mant = [22:0]
  //
  // 1.0 = 32'h3f800000 = sign=0 exp=127 mant=0
  // ----------------------------------------------------------
  function automatic [31:0] clamp_fp32_0_1(input [31:0] x);
    reg sign;
    reg [7:0] exp;
    reg [22:0] mant;
    begin
      sign = x[31];
      exp  = x[30:23];
      mant = x[22:0];

      if (sign) begin
        clamp_fp32_0_1 = 32'h00000000; // negative -> 0.0
      end
      else if (exp < 8'd127) begin
        // 0 <= x < 1
        clamp_fp32_0_1 = x;
      end
      else if ((exp == 8'd127) && (mant == 23'd0)) begin
        // x == 1.0
        clamp_fp32_0_1 = x;
      end
      else begin
        // x > 1.0 or inf/nan -> clamp to 1.0
        clamp_fp32_0_1 = 32'h3f800000;
      end
    end
  endfunction

  // optional: MODE=0/2 暫時先共用 clamp 行為，保持可綜合
  function automatic [31:0] alpha_map(input [31:0] x);
    begin
      case (MODE)
        0: alpha_map = clamp_fp32_0_1(x);
        1: alpha_map = clamp_fp32_0_1(x);
        2: alpha_map = clamp_fp32_0_1(x);
        default: alpha_map = clamp_fp32_0_1(x);
      endcase
    end
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= 32'h0;
    end else begin
      if (do_accept) begin
        s0_dat <= alpha_map(out0_bits);
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  // ----------------------------------------------------------
  // Optional extra elastic stages
  // ----------------------------------------------------------
  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid    = s0_vld;
      assign s0_out_ready = out_ready;
      assign alpha_fp32   = s0_dat;
    end else begin : g_pipe
      rv_pipe #(
        .WIDTH (32),
        .STAGES(PIPE_STAGES-1)
      ) u_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_dat),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (alpha_fp32)
      );
    end
  endgenerate

endmodule

`default_nettype wire
