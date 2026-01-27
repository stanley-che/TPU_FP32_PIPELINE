`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp_mul_driver.sv"
`timescale 1ns/1ps
`default_nettype none

module fp32_exp_no_lut (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  // ----------------------------
  // constants (FP32)
  // ----------------------------
  localparam logic [31:0] FP32_ONE   = 32'h3f800000;
  localparam logic [31:0] FP32_LOG2E = 32'h3fb8aa3b; // ~1.4426950
  localparam logic [31:0] FP32_LN2   = 32'h3f317218; // ~0.69314718

  // 1/2,1/6,1/24,1/120 in FP32
  localparam logic [31:0] C2 = 32'h3f000000; // 0.5
  localparam logic [31:0] C3 = 32'h3e2aaaab; // 0.16666667
  localparam logic [31:0] C4 = 32'h3d2aaaab; // 0.041666667
  localparam logic [31:0] C5 = 32'h3c088889; // 0.0083333338

  // ----------------------------
  // FP helpers
  // ----------------------------
  function automatic logic [31:0] fp32_neg(input logic [31:0] x);
    fp32_neg = {~x[31], x[30:0]};
  endfunction

  // floor(t) to int (signed) from FP32 bits, combinational decode
  // Assumes t is finite and |t| not huge. Good enough for exp range reduction.
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
        // subnormal or zero: |t|<2^-126 -> floor is 0 or -1 if negative and not zero
        fp32_to_int_floor = (sign && (f[22:0]!=0)) ? -1 : 0;
      end else if (exp_u == 8'hFF) begin
        // inf/nan: saturate
        fp32_to_int_floor = sign ? -2147483648 : 2147483647;
      end else begin
        exp  = exp_u - 127;
        mant = {1'b1, f[22:0]}; // 1.xxx in Q1.23

        // integer part magnitude = mant * 2^(exp-23)
        shift = exp - 23;

        if (exp < 0) begin
          // |t| < 1.0 -> floor = 0 or -1
          fp32_to_int_floor = sign ? -1 : 0;
        end else if (exp > 30) begin
          fp32_to_int_floor = sign ? -2147483648 : 2147483647;
        end else begin
          if (shift >= 0) val = (mant << shift);
          else            val = (mant >> (-shift));

          // apply sign
          if (!sign) begin
            fp32_to_int_floor = val;
          end else begin
            // if negative and has fractional bits, floor() is -(val) - 1
            // detect fractional: when shift < 0, bits shifted out were non-zero
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

  // build FP32 bits for 2^n (n is signed int)
  function automatic logic [31:0] fp32_pow2_int(input int n);
    int e;
    begin
      e = n + 127;
      if (e <= 0)        fp32_pow2_int = 32'h0000_0000; // underflow -> 0 (可改成 subnormal)
      else if (e >= 255) fp32_pow2_int = 32'h7f80_0000; // overflow -> +inf
      else               fp32_pow2_int = {1'b0, e[7:0], 23'd0};
    end
  endfunction

  // ----------------------------
  // FP mul / add drivers
  // ----------------------------
  logic mul_start, mul_busy, mul_done;
  logic [31:0] mul_a, mul_b, mul_z;

  logic add_start, add_busy, add_done;
  logic [31:0] add_a, add_b, add_z;

  fp_mul_driver u_mul (
    .clk   (clk),
    .rst   (~rst_n),
    .start (mul_start),
    .a_bits(mul_a),
    .b_bits(mul_b),
    .busy  (mul_busy),
    .done  (mul_done),
    .z_bits(mul_z)
  );

  fp_adder_driver_ba u_add (
    .clk   (clk),
    .rst   (~rst_n),
    .start (add_start),
    .a_bits(add_a),
    .b_bits(add_b),
    .busy  (add_busy),
    .done  (add_done),
    .z_bits(add_z)
  );

  // ----------------------------
  // FSM
  // ----------------------------
  typedef enum logic [4:0] {
    S_IDLE,
    S_T_MUL,          // t = x*log2e
    S_T_MUL_W,

    S_F_SUB,          // f = t - n (n as fp32)
    S_F_SUB_W,

    S_U_MUL,          // u = f*ln2
    S_U_MUL_W,

    // Horner for exp(u):
    // h = 1/120
    // h = 1/24 + u*h
    // h = 1/6  + u*h
    // h = 1/2  + u*h
    // h = 1    + u*h
    // e = 1 + u*h
    S_H1_MUL, S_H1_MUL_W, S_H1_ADD, S_H1_ADD_W,
    S_H2_MUL, S_H2_MUL_W, S_H2_ADD, S_H2_ADD_W,
    S_H3_MUL, S_H3_MUL_W, S_H3_ADD, S_H3_ADD_W,
    S_H4_MUL, S_H4_MUL_W, S_H4_ADD, S_H4_ADD_W,
    S_E_MUL,  S_E_MUL_W,  S_E_ADD,  S_E_ADD_W,

    S_SCALE_MUL,      // out = (2^n) * e
    S_SCALE_MUL_W,
    S_DONE
  } state_t;

  state_t st;

  // regs
  logic [31:0] x_fp, t_fp, f_fp, u_fp;
  int          n_int;
  logic [31:0] n_fp;     // n as fp32
  logic [31:0] pow2n_fp;
  logic [31:0] h_fp, e_fp;

  // simple int->fp32 for small integers (exact for |n| < 2^24)
  function automatic logic [31:0] int_to_fp32_exact(input int n);
    shortreal s;
    begin
      s = n;
      int_to_fp32_exact = $shortrealtobits(s);
    end
  endfunction

  // start pulses default
  always_comb begin
    mul_start = 1'b0; mul_a = 32'd0; mul_b = 32'd0;
    add_start = 1'b0; add_a = 32'd0; add_b = 32'd0;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st        <= S_IDLE;
      out_valid <= 1'b0;
      out_fp32  <= 32'd0;
      x_fp      <= 32'd0;
      t_fp      <= 32'd0;
      f_fp      <= 32'd0;
      u_fp      <= 32'd0;
      n_int     <= 0;
      n_fp      <= 32'd0;
      pow2n_fp  <= 32'd0;
      h_fp      <= C5;
      e_fp      <= 32'd0;
    end else begin
      out_valid <= 1'b0;

      unique case (st)
        S_IDLE: begin
          if (in_valid) begin
            x_fp <= in_fp32;
            st   <= S_T_MUL;
          end
        end

        // t = x * log2e
        S_T_MUL: begin
          mul_a     <= x_fp;
          mul_b     <= FP32_LOG2E;
          mul_start <= 1'b1;
          st        <= S_T_MUL_W;
        end
        S_T_MUL_W: if (mul_done) begin
          t_fp  <= mul_z;
          n_int <= fp32_to_int_floor(mul_z);
          n_fp  <= int_to_fp32_exact(fp32_to_int_floor(mul_z));
          pow2n_fp <= fp32_pow2_int(fp32_to_int_floor(mul_z));
          st    <= S_F_SUB;
        end

        // f = t - n
        S_F_SUB: begin
          add_a     <= t_fp;
          add_b     <= fp32_neg(n_fp);
          add_start <= 1'b1;
          st        <= S_F_SUB_W;
        end
        S_F_SUB_W: if (add_done) begin
          f_fp <= add_z;
          st   <= S_U_MUL;
        end

        // u = f * ln2
        S_U_MUL: begin
          mul_a     <= f_fp;
          mul_b     <= FP32_LN2;
          mul_start <= 1'b1;
          st        <= S_U_MUL_W;
        end
        S_U_MUL_W: if (mul_done) begin
          u_fp <= mul_z;
          h_fp <= C5;
          st   <= S_H1_MUL;
        end

        // h = C4 + u*h
        S_H1_MUL: begin
          mul_a     <= u_fp;
          mul_b     <= h_fp;
          mul_start <= 1'b1;
          st        <= S_H1_MUL_W;
        end
        S_H1_MUL_W: if (mul_done) begin
          add_a     <= C4;
          add_b     <= mul_z;
          add_start <= 1'b1;
          st        <= S_H1_ADD_W;
        end
        S_H1_ADD_W: if (add_done) begin
          h_fp <= add_z;
          st   <= S_H2_MUL;
        end

        // h = C3 + u*h
        S_H2_MUL: begin
          mul_a     <= u_fp;
          mul_b     <= h_fp;
          mul_start <= 1'b1;
          st        <= S_H2_MUL_W;
        end
        S_H2_MUL_W: if (mul_done) begin
          add_a     <= C3;
          add_b     <= mul_z;
          add_start <= 1'b1;
          st        <= S_H2_ADD_W;
        end
        S_H2_ADD_W: if (add_done) begin
          h_fp <= add_z;
          st   <= S_H3_MUL;
        end

        // h = C2 + u*h
        S_H3_MUL: begin
          mul_a     <= u_fp;
          mul_b     <= h_fp;
          mul_start <= 1'b1;
          st        <= S_H3_MUL_W;
        end
        S_H3_MUL_W: if (mul_done) begin
          add_a     <= C2;
          add_b     <= mul_z;
          add_start <= 1'b1;
          st        <= S_H3_ADD_W;
        end
        S_H3_ADD_W: if (add_done) begin
          h_fp <= add_z;
          st   <= S_H4_MUL;
        end

        // h = 1 + u*h
        S_H4_MUL: begin
          mul_a     <= u_fp;
          mul_b     <= h_fp;
          mul_start <= 1'b1;
          st        <= S_H4_MUL_W;
        end
        S_H4_MUL_W: if (mul_done) begin
          add_a     <= FP32_ONE;
          add_b     <= mul_z;
          add_start <= 1'b1;
          st        <= S_H4_ADD_W;
        end
        S_H4_ADD_W: if (add_done) begin
          h_fp <= add_z;
          st   <= S_E_MUL;
        end

        // e = 1 + u*h
        S_E_MUL: begin
          mul_a     <= u_fp;
          mul_b     <= h_fp;
          mul_start <= 1'b1;
          st        <= S_E_MUL_W;
        end
        S_E_MUL_W: if (mul_done) begin
          add_a     <= FP32_ONE;
          add_b     <= mul_z;
          add_start <= 1'b1;
          st        <= S_E_ADD_W;
        end
        S_E_ADD_W: if (add_done) begin
          e_fp <= add_z;
          st   <= S_SCALE_MUL;
        end

        // out = 2^n * e
        S_SCALE_MUL: begin
          mul_a     <= pow2n_fp;
          mul_b     <= e_fp;
          mul_start <= 1'b1;
          st        <= S_SCALE_MUL_W;
        end
        S_SCALE_MUL_W: if (mul_done) begin
          out_fp32  <= mul_z;
          st        <= S_DONE;
        end

        S_DONE: begin
          out_valid <= 1'b1;
          st        <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
