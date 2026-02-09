`timescale 1ns/1ps
`default_nettype none

package fp32_iverilog_pkg;

  // FP32 -> real (IEEE754 single)
  function automatic real fp32_to_real(input logic [31:0] f);
    logic sign;
    int unsigned exp;
    int unsigned mant;
    real frac;
    real val;
    begin
      sign = f[31];
      exp  = f[30:23];
      mant = f[22:0];

      if (exp == 0) begin
        if (mant == 0) begin
          val = 0.0;
        end else begin
          // subnormal: (-1)^s * 2^(1-127) * (mant/2^23)
          frac = mant / 8388608.0; // 2^23
          val  = frac * (2.0 ** (-126));
        end
      end else if (exp == 255) begin
        // Inf/NaN -> clamp to huge (for demo)
        val = (mant == 0) ? 1.0e30 : 0.0;
      end else begin
        // normal: (-1)^s * 2^(exp-127) * (1 + mant/2^23)
        frac = 1.0 + (mant / 8388608.0);
        val  = frac * (2.0 ** (integer'(exp) - 127));
      end

      fp32_to_real = sign ? -val : val;
    end
  endfunction

  // real -> FP32 (approx pack, round-to-nearest)
  function automatic logic [31:0] real_to_fp32(input real r);
    logic sign;
    real  a, v;
    int   e;
    int   exp;
    int   mant;
    real  frac;
    begin
      // sign / abs
      sign = (r < 0.0);
      a    = sign ? -r : r;

      if (a == 0.0) begin
        real_to_fp32 = 32'h0000_0000;
      end else if (a > 3.4e38) begin
        // overflow -> +Inf
        real_to_fp32 = {sign, 8'hFF, 23'h0};
      end else begin
        // normalize to [1,2)
        v = a;
        e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end

        exp = e + 127;

        if (exp <= 0) begin
          // subnormal (very small): mant = a / 2^(1-127) * 2^23
          frac = a / (2.0 ** (-126));
          mant = integer'(frac * 8388608.0 + 0.5);
          if (mant <= 0) mant = 0;
          if (mant >  8388607) mant = 8388607;
          real_to_fp32 = {sign, 8'h00, mant[22:0]};
        end else if (exp >= 255) begin
          real_to_fp32 = {sign, 8'hFF, 23'h0};
        end else begin
          // normal mantissa
          frac = v - 1.0; // [0,1)
          mant = integer'(frac * 8388608.0 + 0.5); // round
          if (mant == 8388608) begin
            // rounding overflow -> bump exponent
            mant = 0;
            exp  = exp + 1;
            if (exp >= 255) begin
              real_to_fp32 = {sign, 8'hFF, 23'h0};
              return;
            end
          end
          real_to_fp32 = {sign, logic'(exp[7:0]), logic'(mant[22:0])};
        end
      end
    end
  endfunction

endpackage

`default_nettype wire
