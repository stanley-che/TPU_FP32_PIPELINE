`include "./src/EPU/attention_score/fp32_exp_lut.sv"
`timescale 1ns/1ps
`default_nettype none
`timescale 1ns/1ps
`default_nettype none

// -----------------------------------------------------------------------------
// exp_fp32_wrapper_no_lut.sv
// FP32 exp wrapper (NO LUT):
//   FP32 -> signed Q16.16 -> exp_q16_16_poly -> unsigned Q16.16 -> FP32
// Interface:
//   in_valid + in_fp32  -> out_valid + out_fp32
// -----------------------------------------------------------------------------

module exponential_lut (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  // ----------------------------
  // FP32 -> signed Q16.16  (iverilog-friendly)
  // ----------------------------
  function automatic logic signed [31:0] fp32_to_q16_16(input logic [31:0] f);
    logic        sign;
    int          exp_u;
    int          exp;           // unbiased exponent
    logic [22:0] frac;
    logic [23:0] mant;          // 1.xxx as 24-bit (or 0.xxx for subnormal)
    logic        is_zero, is_inf, is_nan, is_sub;
    logic signed [63:0] val;
    int          sh;

    begin
      sign = f[31];
      exp_u = f[30:23];
      frac  = f[22:0];

      is_zero = (exp_u == 0) && (frac == 0);
      is_inf  = (exp_u == 8'hFF) && (frac == 0);
      is_nan  = (exp_u == 8'hFF) && (frac != 0);
      is_sub  = (exp_u == 0) && (frac != 0);

      if (is_zero) begin
        fp32_to_q16_16 = 32'sd0;
      end else if (is_nan) begin
        fp32_to_q16_16 = 32'sd0;          // treat NaN as 0 (safe)
      end else if (is_inf) begin
        fp32_to_q16_16 = sign ? -32'sh7FFF_0000 : 32'sh7FFF_0000; // saturate
      end else begin
        if (is_sub) begin
          // subnormal: exponent = -126, mantissa = 0.frac
          exp  = -126;
          mant = {1'b0, frac};            // 23 bits frac, leading 0
        end else begin
          exp  = exp_u - 127;
          mant = {1'b1, frac};            // 1.frac
        end

        // We want: value = (-1)^sign * mant * 2^(exp-23)   (since mant has 23 frac bits)
        // Convert to Q16.16 => multiply by 2^16:
        // Q = mant * 2^(exp - 23 + 16) = mant * 2^(exp - 7)
        val = $signed({1'b0,mant});       // up to 24 bits positive
        sh  = exp - 7;

        if (sh >= 0) val = val <<< sh;
        else         val = val >>> (-sh);

        if (sign) val = -val;

        // saturate to 32-bit signed
        if (val >  64'sh7FFF_FFFF) fp32_to_q16_16 = 32'sh7FFF_FFFF;
        else if (val < -64'sh8000_0000) fp32_to_q16_16 = -32'sh8000_0000;
        else fp32_to_q16_16 = val[31:0];
      end
    end
  endfunction

  // ----------------------------
  // unsigned Q16.16 -> FP32 (no LUT)
  // exp output is always >= 0, so we only support unsigned here.
  // ----------------------------
  function automatic logic [31:0] q16_16_to_fp32(input logic [31:0] q);
    int msb;
    int exp_u;
    logic [55:0] norm;      // enough bits for shifting
    logic [23:0] mant24;    // 1.xxx (24 bits including hidden 1)
    logic [22:0] frac23;

    begin
      if (q == 0) begin
        q16_16_to_fp32 = 32'h0000_0000; // +0
      end else begin
        // find MSB position (0..31)
        msb = 31;
        while (msb > 0 && q[msb] == 1'b0) msb--;

        // q is Q16.16 => actual exponent (base2) is (msb - 16)
        exp_u = (msb - 16) + 127;

        // normalize so that leading 1 ends up at bit 23 of mant24 (hidden 1 + 23 frac)
        // We want mant24 = q shifted such that MSB -> bit 23
        norm = {24'd0, q}; // widen
        if (msb > 23) norm = norm >> (msb - 23);
        else          norm = norm << (23 - msb);

        mant24 = norm[23:0];          // includes leading 1 at [23]
        frac23 = mant24[22:0];        // drop hidden 1

        // handle exponent under/overflow roughly
        if (exp_u <= 0) begin
          // underflow -> +0 (you can improve to subnormal if needed)
          q16_16_to_fp32 = 32'h0000_0000;
        end else if (exp_u >= 255) begin
          // overflow -> +inf
          q16_16_to_fp32 = 32'h7F80_0000;
        end else begin
          q16_16_to_fp32 = {1'b0, exp_u[7:0], frac23};
        end
      end
    end
  endfunction

  // ----------------------------
  // internal signals
  // ----------------------------
  logic signed [31:0] x_q16_16;
  logic        core_out_valid;
  logic [31:0] y_q16_16_u;

  always_comb begin
    x_q16_16 = fp32_to_q16_16(in_fp32);
  end

  // ----------------------------
  // exp core (NO LUT)
  // ----------------------------
  exp_q16_16_poly u_core (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .x_q16_16 (x_q16_16),
    .out_valid(core_out_valid),
    .y_q16_16 (y_q16_16_u)
  );

  // ----------------------------
  // output convert + align valid
  // ----------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      out_valid <= 1'b0;
      out_fp32  <= 32'h0000_0000;
    end else begin
      out_valid <= core_out_valid;
      if (core_out_valid) begin
        out_fp32 <= q16_16_to_fp32(y_q16_16_u);
      end
    end
  end

endmodule




`default_nettype wire
