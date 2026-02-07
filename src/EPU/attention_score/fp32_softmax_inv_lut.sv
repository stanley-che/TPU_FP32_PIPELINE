// ============================================================================
// fp32_softmax_inv_lut.sv
// Softmax-oriented FP32 reciprocal (1/x) approximator
// - Assumes x is positive, finite, normalized FP32 (sum of exp, so >0)
// - 1-cycle latency: in_valid -> out_valid next cycle
// - Approx: 1/x ≈ (1/mantissa) * 2^(127-exp)
//   where mantissa in [1,2). We LUT 1/mantissa using top M_BITS of frac.
// ----------------------------------------------------------------------------
// Notes:
// - If input is 0 or denorm, output = +inf (or a large number); here we clamp
//   to +inf for x==0, and treat denorm as +inf too.
// - If input is inf/NaN, output = 0 for inf, NaN passthrough optional.
// - This is "good enough" for softmax normalization in inference-style flow.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none
`timescale 1ns/1ps
`default_nettype none

module fp32_softmax_inv_lut #(
  parameter int unsigned M_BITS = 6
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  localparam int unsigned LUT_SIZE = (1 << M_BITS);

  // ----------------------------
  // fields
  // ----------------------------
  logic        s;
  logic [7:0]  e;
  logic [22:0] f;
  assign s = in_fp32[31];
  assign e = in_fp32[30:23];
  assign f = in_fp32[22:0];

  // index = top M_BITS of fraction
  logic [M_BITS-1:0] idx;
  assign idx = f[22 -: M_BITS];

  // ----------------------------
  // Build inv_mantissa LUT in Q(1.23) domain:
  // mantissa m = 1 + frac
  // pick midpoint frac = (idx + 0.5) / 2^M_BITS
  // m_q23 = (1<<23) + round(frac * 2^23)
  // inv_q23 = round((1<<46) / m_q23)  -> gives ~ (1/m) in Q(1.23)
  // Then convert inv_q23 to FP32 with exp=127:
  // inv_fp = 1.xxx where xxx = inv_q23 - (1<<23)   (because Q(1.23) includes hidden 1)
  // But note: 1/m is in [0.5,1], sometimes <1, so normalize:
  // if inv_q23 < (1<<23) => shift left 1 and remember exp adjust (-1)
  // We'll bake that into frac + exp adjust.
  // ----------------------------
  function automatic logic [23:0] inv_mant_q23(input logic [M_BITS-1:0] i);
    int unsigned m_q23;
    int unsigned inv_q23;
    int unsigned frac_q23;
    int unsigned mid_num;
    begin
      // frac midpoint in Q23: (i + 0.5) / 2^M_BITS
      // frac_q23 = round( (i*2 + 1) * 2^(23-M_BITS-1) )
      mid_num  = (i << 1) | 1; // (2*i + 1)
      frac_q23 = mid_num << (23 - M_BITS - 1);

      m_q23    = (1<<23) + frac_q23;            // Q(1.23)
      inv_q23  = (1<<46) / m_q23;               // Q(1.23), floor is fine

      inv_mant_q23 = inv_q23[23:0];             // keep 24 bits (includes hidden 1 possibly)
    end
  endfunction

  // inv mant + exp adjust
  logic [23:0] invq;
  logic        inv_shift;     // if inv <1 then normalize by <<1
  logic [22:0] inv_frac;
  logic [7:0]  inv_exp;

  always@(*) begin
    invq = inv_mant_q23(idx);

    // invq represents ~1/m in Q(1.23) but could be <1.0 (i.e., top bit 0)
    // Q(1.23) "1.0" corresponds to (1<<23)
    inv_shift = (invq < (1<<23));

    if (inv_shift) begin
      // normalize to [1,2): multiply by 2 => shift left 1
      // exponent should be 126 instead of 127 (because value doubled)
      inv_frac = {invq[21:0], 1'b0};   // hidden 1 removed implicitly by taking frac bits
    end else begin
      inv_frac = invq[22:0];
    end

    // exponent for reciprocal:
    // base exp = 254 - e
    // plus correction if we normalized inv_mant:
    // if inv_shift (inv was <1), we multiplied by 2, so need exp-1
    inv_exp = (8'd254 - e) - (inv_shift ? 8'd1 : 8'd0);
  end

  // ----------------------------
  // Special cases
  // ----------------------------
  localparam logic [31:0] FP32_PZERO = 32'h0000_0000;
  localparam logic [31:0] FP32_PINF  = 32'h7f80_0000;
  localparam logic [31:0] FP32_NAN   = 32'h7fc0_0000;

  logic [31:0] out_next;
  always_comb begin
    out_next = FP32_PZERO;

    if (e == 8'h00) begin
      out_next = {s, 8'hff, 23'h0};           // +/-inf
    end else if (e == 8'hff) begin
      if (f == 23'h0) out_next = {s, 8'h00, 23'h0}; // 1/inf = 0
      else            out_next = FP32_NAN;
    end else begin
      out_next = {s, inv_exp, inv_frac};
    end
  end

  // ----------------------------
  // 1-cycle latency
  // ----------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      out_valid <= 1'b0;
      out_fp32  <= 32'h0;
    end else begin
      out_valid <= in_valid;
      if (in_valid) out_fp32 <= out_next;
    end
  end

endmodule

`default_nettype wire
