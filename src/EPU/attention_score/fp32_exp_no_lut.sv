`ifndef FP32_EXP_NO_LUT_SV
`define FP32_EXP_NO_LUT_SV
`timescale 1ns/1ps
`default_nettype none

module fp32_exp_no_lut #(
  parameter int unsigned STEP_INV = 64,   // fixed at 64 for this LUT
  parameter int unsigned XMAX_Q   = 8,    // 0 to -8
  parameter string       LUT_MEMH = "./src/EPU/attention_score/exp_lut.memh"
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,     // x in FP32, expected range [0 .. -8]

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  // ----------------------------
  // helpers
  // ----------------------------
  function automatic logic [31:0] fp32_neg(input logic [31:0] x);
    fp32_neg = {~x[31], x[30:0]};
  endfunction

  // multiply by 2^k by exponent adjust (for normal numbers)
  function automatic logic [31:0] fp32_mul_pow2(input logic [31:0] f, input int k);
    logic sign;
    int   exp_u;
    logic [22:0] frac;
    int   e2;
    begin
      sign  = f[31];
      exp_u = f[30:23];
      frac  = f[22:0];

      if (exp_u == 8'hFF) begin
        fp32_mul_pow2 = f; // NaN/Inf passthrough
      end else if (exp_u == 8'h00) begin
        // subnormal/zero: keep (good enough for indexing around 0)
        fp32_mul_pow2 = f;
      end else begin
        e2 = exp_u + k;
        if (e2 <= 0)        fp32_mul_pow2 = 32'h0000_0000;
        else if (e2 >= 255) fp32_mul_pow2 = {sign, 8'hFF, 23'd0}; // +/-Inf
        else                fp32_mul_pow2 = {sign, e2[7:0], frac};
      end
    end
  endfunction

  // floor(fp32) -> int (same style as your original)
  function automatic int fp32_to_int_floor(input logic [31:0] f);
    logic sign;
    int   exp_u, exp;
    logic [23:0] mant;
    int   shift;
    int   val;
    begin
      sign  = f[31];
      exp_u = f[30:23];
      if (exp_u == 0) begin
        fp32_to_int_floor = (sign && (f[22:0]!=0)) ? -1 : 0;
      end else if (exp_u == 8'hFF) begin
        fp32_to_int_floor = sign ? -2147483648 : 2147483647;
      end else begin
        exp  = exp_u - 127;
        mant = {1'b1, f[22:0]};
        shift = exp - 23;

        if (exp < 0) begin
          fp32_to_int_floor = sign ? -1 : 0;
        end else if (exp > 30) begin
          fp32_to_int_floor = sign ? -2147483648 : 2147483647;
        end else begin
          if (shift >= 0) val = (mant << shift);
          else            val = (mant >> (-shift));

          if (!sign) begin
            fp32_to_int_floor = val;
          end else begin
            if (shift < 0) begin
              int frac_mask;
              frac_mask = (1 << (-shift)) - 1;
              fp32_to_int_floor = -val - (((mant & frac_mask) != 0) ? 1 : 0);
            end else begin
              fp32_to_int_floor = -val;
            end
          end
        end
      end
    end
  endfunction

  // ----------------------------
  // LUT: exp(-k/64), k=0..512  (513 entries)
  // idx = clamp(floor((-x)*64), 0..512)
  // ----------------------------
  localparam int unsigned LUT_SIZE = (XMAX_Q*STEP_INV + 1); // 8*64+1=513

  logic [31:0] lut [0:LUT_SIZE-1];
  initial begin
    $readmemh(LUT_MEMH, lut);
  end

  // ----------------------------
  // 1-cycle pipeline
  // ----------------------------
  logic        v_q;
  logic [31:0] x_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      v_q       <= 1'b0;
      x_q       <= 32'd0;
      out_valid <= 1'b0;
      out_fp32  <= 32'd0;
    end else begin
      // stage0 latch
      v_q <= in_valid;
      x_q <= in_fp32;

      // stage1 output
      out_valid <= v_q;
      if (v_q) begin
        logic [31:0] neg_x;
        logic [31:0] scaled; // (-x) * 64

        int idx;
        int q;

        neg_x  = fp32_neg(x_q);           // -x (so 0..8)
        scaled = fp32_mul_pow2(neg_x, 6); // *64 = 2^6

        q = fp32_to_int_floor(scaled);

        if (q < 0) q = 0;
        if (q > int'(LUT_SIZE-1)) q = int'(LUT_SIZE-1);

        idx = q;
        out_fp32 <= lut[idx];
      end
    end
  end

endmodule

`default_nettype wire
`endif
