// ============================================================
// attn_alpha_fp32.sv  (IVERILOG-SAFE)
// - M4: alpha = g(out_vec)
//   Demo options (pick one via MODE):
//     MODE=0: alpha = sigmoid(out_vec[0])   (approx, stable)
//     MODE=1: alpha = clamp(out_vec[0], 0..1)
//     MODE=2: alpha = clamp(SCALE*out_vec[0] + BIAS, 0..1)
//
// - ready/valid elastic, alpha holds under backpressure
// - No shortreal, use real + local FP32 pack/unpack
// ============================================================

`timescale 1ns/1ps
`default_nettype none

module attn_alpha_fp32 #(
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 1,   // >=1 (extra elastic stages)
  parameter int unsigned MODE        = 1,   // 0=sigmoid,1=clamp,2=linear+clamp
  parameter real         SCALE       = 1.0, // used in MODE=2
  parameter real         BIAS        = 0.0, // used in MODE=2
  parameter real         SIG_A       = 1.0  // used in MODE=0: sigmoid steepness
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
  // pow2i (iverilog-safe)
  // ----------------------------------------------------------
  function real pow2i(input integer e);
    integer i;
    real v;
    begin
      v = 1.0;
      if (e >= 0)
        for (i = 0; i < e; i = i + 1) v = v * 2.0;
      else
        for (i = 0; i < (-e); i = i + 1) v = v / 2.0;
      pow2i = v;
    end
  endfunction

  // ----------------------------------------------------------
  // FP32 <-> real (local)
  // ----------------------------------------------------------
  function real fp32_to_real(input [31:0] f);
    reg sign;
    integer exp, mant;
    real frac, val;
    begin
      sign = f[31];
      exp  = f[30:23];
      mant = f[22:0];

      if (exp == 0) begin
        if (mant == 0) val = 0.0;
        else begin
          frac = mant / 8388608.0;
          val  = frac * pow2i(-126);
        end
      end else if (exp == 255) begin
        val = (mant == 0) ? 1.0e30 : 0.0;
      end else begin
        frac = 1.0 + (mant / 8388608.0);
        val  = frac * pow2i(exp - 127);
      end

      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign;
    real a, v;
    integer e, exp_i, mant_i;
    real frac;
    reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;
      out  = 32'h0;

      if (a == 0.0) begin
        out = 32'h0;
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end

        exp_i = e + 127;

        frac   = v - 1.0;
        mant_i = integer'(frac * 8388608.0 + 0.5);
        if (mant_i >= 8388608) begin
          mant_i = 0;
          exp_i  = exp_i + 1;
        end
        out = {sign, exp_i[7:0], mant_i[22:0]};
      end

      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // helpers: clamp / exp2 approx / sigmoid approx
  // ----------------------------------------------------------
  function real clamp01(input real x);
    begin
      if (x < 0.0) clamp01 = 0.0;
      else if (x > 1.0) clamp01 = 1.0;
      else clamp01 = x;
    end
  endfunction

  localparam real LN2   = 0.6931471805599453;
  localparam real LOG2E = 1.4426950408889634;

  function real exp2_frac(input real F);
    real z,z2,z3;
    begin
      z  = F * LN2;
      z2 = z*z;
      z3 = z2*z;
      exp2_frac = 1.0 + z + (z2*0.5) + (z3*(1.0/6.0));
    end
  endfunction

  function real exp2_approx(input real y);
    integer it, ifloor;
    real F;
    begin
      it = integer'(y);
      if ((y < 0.0) && (y != it)) ifloor = it - 1;
      else                        ifloor = it;
      F = y - ifloor;
      exp2_approx = pow2i(ifloor) * exp2_frac(F);
    end
  endfunction

  // sigmoid(x) = 1 / (1 + exp(-a*x))
  // use exp(-a*x) ~ exp2((-a*x)*log2(e))
  function real sigmoid_approx(input real x);
    real z, e;
    begin
      // clamp to avoid overflow
      if (x > 8.0) sigmoid_approx = 0.999664; // ~sigmoid(8)
      else if (x < -8.0) sigmoid_approx = 0.000335; // ~sigmoid(-8)
      else begin
        z = (-SIG_A * x) * LOG2E;     // log2(exp(-a*x))
        e = exp2_approx(z);           // exp(-a*x)
        sigmoid_approx = 1.0 / (1.0 + e);
      end
    end
  endfunction

  // ----------------------------------------------------------
  // Stage0: compute-on-accept 1-deep elastic buffer
  // ----------------------------------------------------------
  reg        s0_vld;
  reg [31:0] s0_dat;
  wire       s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  // pick out_vec[0]
  wire [31:0] out0_bits = out_vec[0*32 +: 32];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= 32'h0;
    end else begin
      if (do_accept) begin
        real x, a;
        x = fp32_to_real(out0_bits);

        if (MODE == 0) begin
          a = sigmoid_approx(x);
        end else if (MODE == 1) begin
          a = clamp01(x);
        end else begin
          a = clamp01(SCALE*x + BIAS);
        end

        s0_dat <= real_to_fp32(a);
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
      // else hold
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
      // need rv_pipe from your shared util; simplest: reuse same file where rv_pipe exists
      // If you don't have it globally, include/compile the file that defines rv_pipe.
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
