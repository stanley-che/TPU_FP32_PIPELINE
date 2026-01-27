// ------------------------------------------------------------
// tb_fp32_exp_lut.sv  (step=1/128 scan, gold matches LUT quant)
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_exp.vvp ./test/tb_fp32_exp_lut.sv
//   vvp ./vvp/tb_exp.vvp
// ------------------------------------------------------------
`include "./src/EPU/attention_score/fp32_exp_lut.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_fp32_exp_lut_iverilog;

  // ----------------------------
  // clock/reset
  // ----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n;
  initial begin
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic        in_valid;
  logic [31:0] in_fp32;
  logic        out_valid;
  logic [31:0] out_fp32;

  fp32_exp_lut dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .in_fp32  (in_fp32),
    .out_valid(out_valid),
    .out_fp32 (out_fp32)
  );

  // ----------------------------
  // FP32 -> real (print/debug)
  // ----------------------------
  function automatic real fp32_to_real(input logic [31:0] b);
    int  s;
    int  e;
    int  frac;
    real mant;
    real val;
    begin
      s    = b[31];
      e    = b[30:23];
      frac = b[22:0];

      if (e == 0) begin
        if (frac == 0) val = 0.0;
        else begin
          mant = frac / 8388608.0;
          val  = mant * (2.0 ** (-126));
        end
      end else if (e == 255) begin
        if (frac == 0) val = 1.0/0.0;
        else           val = 0.0/0.0;
      end else begin
        mant = 1.0 + (frac / 8388608.0);
        val  = mant * (2.0 ** (e - 127));
      end

      fp32_to_real = s ? -val : val;
    end
  endfunction

  // ----------------------------
  // Q16.16 -> FP32 bits (TB only, no real)
  // ----------------------------
  function automatic logic [31:0] q16_16_to_fp32_bits(input logic signed [31:0] q);
    logic sign;
    logic [31:0] mag;
    int msb;
    int exp;
    logic [47:0] norm;
    logic [22:0] frac;
    begin
      if (q == 0) begin
        q16_16_to_fp32_bits = 32'h0000_0000;
      end else begin
        sign = q[31];
        mag  = sign ? $unsigned(-q) : $unsigned(q);

        // find msb
        msb = 31;
        while (msb > 0 && mag[msb] == 1'b0) msb--;

        // exponent: value = mag / 2^16  => exp2 = (msb-16)
        exp = (msb - 16) + 127;

        // normalize so that msb lands at bit 23 (implicit 1)
        norm = {mag, 16'b0};
        if (msb > 23) norm = norm >> (msb - 23);
        else          norm = norm << (23 - msb);

        frac = norm[22:0];
        q16_16_to_fp32_bits = {sign, exp[7:0], frac};
      end
    end
  endfunction

  // ----------------------------
  // GOLD: match LUT quant rule (STEP_INV=128)
  // y_gold = exp( -round((-x)*128)/128 )
  // clamp to [-8,0]
  // ----------------------------
  function automatic real gold_exp_lut(input real x);
    int idx;
    real xq;
    begin
      if (x >= 0.0) idx = 0;
      else if (x <= -8.0) idx = 1024;
      else idx = $rtoi((-x) * 128.0 + 0.5); // round
      xq = -idx / 128.0;
      gold_exp_lut = $pow(2.718281828, xq);
    end
  endfunction

  // ----------------------------
  // abs real
  // ----------------------------
  function automatic real abs_r(input real a);
    abs_r = (a < 0.0) ? -a : a;
  endfunction

  // ----------------------------
  // test helper
  // ----------------------------
  int pass_cnt, fail_cnt;

  task automatic send_one_fp32(
    input logic [31:0] x_bits,
    input real         tol,
    input string       tag
  );
    int  timeout;
    real x_real;
    real y_real;
    real gold;
    real diff;
    begin
      // drive 1-cycle pulse
      @(negedge clk);
      in_valid <= 1'b1;
      in_fp32  <= x_bits;

      @(negedge clk);
      in_valid <= 1'b0;
      in_fp32  <= 32'h0;

      // wait out_valid
      timeout = 0;
      while (out_valid !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 50) begin
          $display("[FAIL] %s: timeout waiting out_valid (out_valid=%b)", tag, out_valid);
          fail_cnt++;
          disable send_one_fp32;
        end
      end

      x_real = fp32_to_real(x_bits);
      y_real = fp32_to_real(out_fp32);
      gold   = gold_exp_lut(x_real);
      diff   = abs_r(y_real - gold);

      $display("[DBG] %s: x=%h (%f) -> y=%h (%f), gold=%f diff=%f",
               tag, x_bits, x_real, out_fp32, y_real, gold, diff);

      if (diff > tol) begin
        $display("[FAIL] %s: diff=%f > tol=%f", tag, diff, tol);
        fail_cnt++;
      end else begin
        pass_cnt++;
      end

      @(posedge clk);
    end
  endtask

  // ----------------------------
  // main
  // ----------------------------
  logic signed [31:0] xq;
  localparam int signed STEP_Q = 512; // 1/128 in Q16.16

  initial begin
    pass_cnt = 0;
    fail_cnt = 0;
    in_valid = 1'b0;
    in_fp32  = 32'h0;

    @(posedge rst_n);
    repeat (2) @(posedge clk);

    // scan x in [-1, 0], step = 1/128
    for (xq = -32'sh0008_0000; xq <= 0; xq += STEP_Q) begin
      send_one_fp32(
        q16_16_to_fp32_bits(xq),
        1e-6, // 因為 gold 跟 DUT 同規則，tol 可以很小
        $sformatf("x=%f", $itor($signed(xq))/65536.0)
      );
    end

    $display("------------------------------------------------------------");
    $display("TB DONE: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
    $display("------------------------------------------------------------");

    if (fail_cnt != 0) $fatal(1, "TEST FAILED");
    else $finish;
  end

endmodule

`default_nettype wire
