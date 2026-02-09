// ============================================================
// attn_softmax_exp2_fp32.sv  (IVERILOG-SAFE)
// - Softmax approximation via exp2 (no exp(), no LUT required)
//   1) m = max(score[0..TOKENS-1])
//   2) x[t] = clamp(score[t] - m, [-XCLAMP, 0])    // <= 0
//   3) y[t] = x[t] * LOG2E                          // convert e^x -> 2^(x*log2(e))
//   4) a[t] = exp2_approx(y[t]) = 2^I * 2^F         // y = I + F, I integer, F in [0,1)
//   5) w[t] = a[t] / sum(a[t])
//   - if sum == 0 -> uniform (1/TOKENS)
// - real + local FP32 pack/unpack
// - ready/valid elastic, output holds under backpressure
// ============================================================

`timescale 1ns/1ps
`default_nettype none

module attn_softmax_exp2_fp32 #(
  parameter int unsigned TOKENS      = 9,
  parameter int unsigned PIPE_STAGES = 1,    // >=1 (extra elastic stages)
  parameter real         XCLAMP      = 8.0   // clamp x=(s-m) to [-XCLAMP,0]
)(
  input  wire                    clk,
  input  wire                    rst_n,

  // input
  input  wire                    in_valid,
  output wire                    in_ready,
  input  wire [TOKENS*32-1:0]    score_flat,

  // output
  output wire                    out_valid,
  input  wire                    out_ready,
  output wire [TOKENS*32-1:0]    w_flat
);

  localparam int unsigned W_W = TOKENS*32;

  // constants
  localparam real LOG2E = 1.4426950408889634; // log2(e)
  localparam real LN2   = 0.6931471805599453; // ln(2)

  // ----------------------------------------------------------
  // pow2i (iverilog-safe): 2^e for integer e
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

        if (exp_i <= 0) begin
          frac   = a / pow2i(-126);
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i < 0) mant_i = 0;
          if (mant_i > 8388607) mant_i = 8388607;
          out = {sign, 8'h00, mant_i[22:0]};
        end else if (exp_i >= 255) begin
          out = {sign, 8'hFF, 23'h0};
        end else begin
          frac   = v - 1.0;
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i >= 8388608) begin
            mant_i = 0;
            exp_i  = exp_i + 1;
            if (exp_i >= 255) out = {sign, 8'hFF, 23'h0};
            else              out = {sign, exp_i[7:0], mant_i[22:0]};
          end else begin
            out = {sign, exp_i[7:0], mant_i[22:0]};
          end
        end
      end
      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // exp2_frac(F) for F in [0,1):
  //   2^F = e^(F ln2) ~ 1 + z + z^2/2 + z^3/6 , z = F*ln2
  // ----------------------------------------------------------
  function real exp2_frac(input real F);
    real z, z2, z3;
    begin
      z  = F * LN2;
      z2 = z * z;
      z3 = z2 * z;
      exp2_frac = 1.0 + z + (z2 * 0.5) + (z3 * (1.0/6.0));
    end
  endfunction

  // ----------------------------------------------------------
  // exp2_approx(y): y real (can be negative)
  // y = I + F, I=floor(y), F in [0,1)
  // exp2(y) = 2^I * 2^F
  // ----------------------------------------------------------
  function real exp2_approx(input real y);
    integer I_trunc;
    integer I_floor;
    real F;
    begin
      I_trunc = integer'(y); // trunc toward 0
      if ((y < 0.0) && (y != I_trunc)) I_floor = I_trunc - 1;
      else                            I_floor = I_trunc;

      F = y - I_floor; // [0,1)
      exp2_approx = pow2i(I_floor) * exp2_frac(F);
    end
  endfunction

  // ----------------------------------------------------------
  // Stage0: compute softmax-exp2 on accept
  // ----------------------------------------------------------
  reg              s0_vld;
  reg [W_W-1:0]    s0_dat;
  wire             s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= '0;
    end else begin
      if (do_accept) begin
        integer t;
        real s [0:TOKENS-1];
        real m;
        real x, y;
        real a [0:TOKENS-1];
        real sum;
        reg [W_W-1:0] tmp;

        // unpack
        for (t = 0; t < TOKENS; t = t + 1)
          s[t] = fp32_to_real(score_flat[t*32 +: 32]);

        // max
        m = s[0];
        for (t = 1; t < TOKENS; t = t + 1)
          if (s[t] > m) m = s[t];

        // exp2 + sum
        sum = 0.0;
        for (t = 0; t < TOKENS; t = t + 1) begin
          x = s[t] - m;               // <= 0
          if (x < -XCLAMP) x = -XCLAMP;
          if (x >  0.0)    x =  0.0;

          y = x * LOG2E;              // <= 0
          a[t] = exp2_approx(y);       // > 0
          sum = sum + a[t];
        end

        // normalize
        tmp = '0;
        if (sum == 0.0) begin
          real u;
          u = 1.0 / TOKENS;
          for (t = 0; t < TOKENS; t = t + 1)
            tmp[t*32 +: 32] = real_to_fp32(u);
        end else begin
          for (t = 0; t < TOKENS; t = t + 1)
            tmp[t*32 +: 32] = real_to_fp32(a[t] / sum);
        end

        s0_dat <= tmp;
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
