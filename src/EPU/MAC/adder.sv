/*
  ============================================================================
  FSM-Based Serial Floating-Point Adder (IEEE-754 FP32 Serial Adder)
  ----------------------------------------------------------------------------
  - Author : Shao Wei Chen
  - Source : Jonathan P. Dawson (2013)
  - Goal   : Fully IEEE-754 compliant single-precision floating-point addition
             (NaN / Infinity / Zero / Denormal, GRS rounding)
  - Architecture :
      * Serial FSM-based design
      * One operation at a time
      * Variable latency
  - Characteristics :
      * Alignment :
          - Exponent alignment by 1-bit right shift per cycle
          - Larger exponent differences result in longer latency
      * Normalization :
          - Leading-zero normalization by 1-bit left shift per cycle
          - More leading zeros result in longer latency
      * Rounding :
          - Round-to-nearest-even using Guard / Round / Sticky (GRS) bits
  - Suitable for :
      * Education
      * Golden reference models
      * FPU functional verification
  - Not suitable for :
      * Softmax / Attention
      * AI accelerators or other high-throughput datapaths
      * Performance-critical numerical workloads
  ============================================================================
=============================================================================
  fp_adder_driver : Usage and Interface Specification
  ----------------------------------------------------------------------------
  This module wraps an FSM-based IEEE-754 FP32 serial floating-point adder
  and provides a simplified start/busy/done handshake interface.

  ----------------------------------------------------------------------------
  INPUT FORMAT
  ----------------------------------------------------------------------------
  a_bits [31:0] :
    - IEEE-754 single-precision floating-point operand A
    - Format:
        [31]    Sign
        [30:23] Exponent (bias = 127)
        [22:0]  Fraction (mantissa)
    - Supported values:
        * Normalized numbers
        * Subnormal numbers
        * ±0, ±Infinity, NaN

  b_bits [31:0] :
    - IEEE-754 single-precision floating-point operand B
    - Same format and supported values as a_bits

  start :
    - Start request signal
    - Sampled only when busy == 0 (IDLE state)
    - Recommended usage:
        * Assert for exactly 1 clock cycle (pulse)
    - Level assertion is allowed, but must be deasserted once busy == 1

  rst :
    - Active-high synchronous reset
    - Resets internal FSM and clears busy/done flags

  ----------------------------------------------------------------------------
  OUTPUT FORMAT
  ----------------------------------------------------------------------------
  z_bits [31:0] :
    - IEEE-754 single-precision floating-point addition result
    - Fully IEEE-754 compliant
    - Valid when done == 1

  done :
    - Completion indicator (LEVEL, sticky)
    - Asserted when the result is available
    - Remains high until a new start request is accepted
    - User must NOT treat done as a pulse

  busy :
    - Indicates the driver is processing a request
    - busy == 1 : operation in progress
    - busy == 0 : driver ready to accept a new start

  ----------------------------------------------------------------------------
  OPERATION SEQUENCE (RECOMMENDED)
  ----------------------------------------------------------------------------
  1) Wait until busy == 0
  2) Drive a_bits and b_bits with valid FP32 operands
  3) Assert start for 1 clock cycle
  4) Wait until done == 1
  5) Read z_bits (result remains stable while done == 1)
  6) Repeat for next operation

  ----------------------------------------------------------------------------
  NOTES
  ----------------------------------------------------------------------------
  - Throughput is one operation at a time
  - Latency is variable and depends on:
      * Exponent difference (alignment)
      * Number of leading zeros (normalization)
  - This module is intended for:
      * Golden reference computation
      * Functional verification
      * Control-dominated datapaths
  - This module is NOT suitable for:
      * High-throughput AI accelerators
      * Softmax / Attention inner loops
=============================================================================
  ============================================================================
  序列式 FSM 浮點加法器（IEEE-754 FP32 Serial Adder）
  ----------------------------------------------------------------------------
  -author:Shao Wei Chen 
  -來源：Jonathan P. Dawson (2013)
  - 目標：完全符合 IEEE-754 單精度加法行為（含 NaN/Inf/Zero/Denormal、GRS rounding）
  - 架構：序列式 FSM（一次只處理一筆，latency 不固定）
  - 特性：
      * align：exponent 對齊採 1 bit/cycle 右移（差距越大越慢）
      * normalise：leading-zero 正規化採 1 bit/cycle 左移（越多 0 越慢）
      * round：支援 round-to-nearest-even（Guard/Round/Sticky）
  - 適用：
      * 教學、golden reference、FPU 行為驗證
  - 不適用：
      * Softmax / Attention / AI Accelerator 等高吞吐場景（overkill 且慢）
  ============================================================================
*/

`timescale 1ns/100ps
module adder(
        input_a,
        input_b,
        input_a_stb,
        input_b_stb,
        output_z_ack,
        clk,
        rst,
        output_z,
        output_z_stb,
        input_a_ack,
        input_b_ack);

  input     clk;
  input     rst;

  input     [31:0] input_a;
  input     input_a_stb;
  output    input_a_ack;

  input     [31:0] input_b;
  input     input_b_stb;
  output    input_b_ack;

  output    [31:0] output_z;
  output    output_z_stb;
  input     output_z_ack;

  reg       s_output_z_stb;
  reg       [31:0] s_output_z;
  reg       s_input_a_ack;
  reg       s_input_b_ack;

  reg       [3:0] state;
  parameter get_a         = 4'd0,
            get_b         = 4'd1,
            unpack        = 4'd2,
            special_cases = 4'd3,
            align         = 4'd4,
            add_0         = 4'd5,
            add_1         = 4'd6,
            normalise_1   = 4'd7,
            normalise_2   = 4'd8,
            round         = 4'd9,
            pack          = 4'd10,
            put_z         = 4'd11;

  reg       [31:0] a, b, z;
  reg       [26:0] a_m, b_m;
  reg       [23:0] z_m;
  reg       [9:0] a_e, b_e, z_e;
  reg       a_s, b_s, z_s;
  reg       guard, round_bit, sticky;
  reg       [27:0] sum;

  always @(posedge clk)
  begin

    case(state)

      get_a:
      begin
        s_input_a_ack <= 1;
        if (s_input_a_ack && input_a_stb) begin
          a <= input_a;
          s_input_a_ack <= 0;
          state <= get_b;
        end
      end

      get_b:
      begin
        s_input_b_ack <= 1;
        if (s_input_b_ack && input_b_stb) begin
          b <= input_b;
          s_input_b_ack <= 0;
          state <= unpack;
        end
      end

      unpack:
      begin
        a_m <= {a[22 : 0], 3'd0};
        b_m <= {b[22 : 0], 3'd0};
        a_e <= a[30 : 23] - 127;
        b_e <= b[30 : 23] - 127;
        a_s <= a[31];
        b_s <= b[31];
        state <= special_cases;
      end

      special_cases:
      begin
        //if a is NaN or b is NaN return NaN 
        if ((a_e == 128 && a_m != 0) || (b_e == 128 && b_m != 0)) begin
          z[31] <= 1;
          z[30:23] <= 255;
          z[22] <= 1;
          z[21:0] <= 0;
          state <= put_z;
        //if a is inf return inf
        end else if (a_e == 128) begin
          z[31] <= a_s;
          z[30:23] <= 255;
          z[22:0] <= 0;
          //if a is inf and signs don't match return nan
          if ((b_e == 128) && (a_s != b_s)) begin
              z[31] <= b_s;
              z[30:23] <= 255;
              z[22] <= 1;
              z[21:0] <= 0;
          end
          state <= put_z;
        //if b is inf return inf
        end else if (b_e == 128) begin
          z[31] <= b_s;
          z[30:23] <= 255;
          z[22:0] <= 0;
          state <= put_z;
        //if a is zero return b
        end else if ((($signed(a_e) == -127) && (a_m == 0)) && (($signed(b_e) == -127) && (b_m == 0))) begin
          z[31] <= a_s & b_s;
          z[30:23] <= b_e[7:0] + 127;
          z[22:0] <= b_m[26:3];
          state <= put_z;
        //if a is zero return b
        end else if (($signed(a_e) == -127) && (a_m == 0)) begin
          z[31] <= b_s;
          z[30:23] <= b_e[7:0] + 127;
          z[22:0] <= b_m[26:3];
          state <= put_z;
        //if b is zero return a
        end else if (($signed(b_e) == -127) && (b_m == 0)) begin
          z[31] <= a_s;
          z[30:23] <= a_e[7:0] + 127;
          z[22:0] <= a_m[26:3];
          state <= put_z;
        end else begin
          //Denormalised Number
          if ($signed(a_e) == -127) begin
            a_e <= -126;
          end else begin
            a_m[26] <= 1;
          end
          //Denormalised Number
          if ($signed(b_e) == -127) begin
            b_e <= -126;
          end else begin
            b_m[26] <= 1;
          end
          state <= align;
        end
      end

      align:
      begin
        if ($signed(a_e) > $signed(b_e)) begin
          b_e <= b_e + 1;
          b_m <= b_m >> 1;
          b_m[0] <= b_m[0] | b_m[1];
        end else if ($signed(a_e) < $signed(b_e)) begin
          a_e <= a_e + 1;
          a_m <= a_m >> 1;
          a_m[0] <= a_m[0] | a_m[1];
        end else begin
          state <= add_0;
        end
      end

      add_0:
      begin
        z_e <= a_e;
        if (a_s == b_s) begin
          sum <= a_m + b_m;
          z_s <= a_s;
        end else begin
          if (a_m >= b_m) begin
            sum <= a_m - b_m;
            z_s <= a_s;
          end else begin
            sum <= b_m - a_m;
            z_s <= b_s;
          end
        end
        state <= add_1;
      end

      add_1:
      begin
        if (sum[27]) begin
          z_m <= sum[27:4];
          guard <= sum[3];
          round_bit <= sum[2];
          sticky <= sum[1] | sum[0];
          z_e <= z_e + 1;
        end else begin
          z_m <= sum[26:3];
          guard <= sum[2];
          round_bit <= sum[1];
          sticky <= sum[0];
        end
        state <= normalise_1;
      end

      normalise_1:
      begin
        if (z_m[23] == 0 && $signed(z_e) > -126) begin
          z_e <= z_e - 1;
          z_m <= z_m << 1;
          z_m[0] <= guard;
          guard <= round_bit;
          round_bit <= 0;
        end else begin
          state <= normalise_2;
        end
      end

      normalise_2:
      begin
        if ($signed(z_e) < -126) begin
          z_e <= z_e + 1;
          z_m <= z_m >> 1;
          guard <= z_m[0];
          round_bit <= guard;
          sticky <= sticky | round_bit;
        end else begin
          state <= round;
        end
      end

      round:
      begin
        if (guard && (round_bit | sticky | z_m[0])) begin
          z_m <= z_m + 1;
          if (z_m == 24'hffffff) begin
            z_e <=z_e + 1;
          end
        end
        state <= pack;
      end

      pack:
      begin
        z[22 : 0] <= z_m[22:0];
        z[30 : 23] <= z_e[7:0] + 127;
        z[31] <= z_s;
        if ($signed(z_e) == -126 && z_m[23] == 0) begin
          z[30 : 23] <= 0;
        end
        if ($signed(z_e) == -126 && z_m[23:0] == 24'h0) begin
          z[31] <= 1'b0; // FIX SIGN BUG: -a + a = +0.
        end
        //if overflow occurs, return inf
        if ($signed(z_e) > 127) begin
          z[22 : 0] <= 0;
          z[30 : 23] <= 255;
          z[31] <= z_s;
        end
        state <= put_z;
      end

      put_z:
      begin
        s_output_z_stb <= 1;
        s_output_z <= z;
        if (s_output_z_stb && output_z_ack) begin
          s_output_z_stb <= 0;
          state <= get_a;
        end
      end

    endcase

    if (rst == 1) begin
      state <= get_a;
      s_input_a_ack <= 0;
      s_input_b_ack <= 0;
      s_output_z_stb <= 0;
    end

  end
  assign input_a_ack = s_input_a_ack;
  assign input_b_ack = s_input_b_ack;
  assign output_z_stb = s_output_z_stb;
  assign output_z = s_output_z;

endmodule
