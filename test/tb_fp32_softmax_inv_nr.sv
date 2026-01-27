// ============================================================================
// tb_fp32_softmax_inv_nr.sv
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_inv_nr.vvp ./test/tb_fp32_softmax_inv_nr.sv
//   vvp ./vvp/tb_inv_nr.vvp
// ============================================================================

`include "./src/EPU/attention_score/fp32_softmax_inv_nr.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_fp32_softmax_inv_nr;

  // ----------------------------
  // clk/rst
  // ----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst_n;

  initial begin
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic        in_valid;
  logic [31:0] in_fp32;

  logic        out_valid;
  logic [31:0] out_fp32;

  fp32_softmax_inv_nr #(.M_BITS(6)) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .in_fp32  (in_fp32),
    .out_valid(out_valid),
    .out_fp32 (out_fp32)
  );

  // ----------------------------
  // Helpers: fp32<->real
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

  // TB reference reciprocal (real)
  function automatic real ref_inv(input real x);
    begin
      if (x == 0.0) ref_inv = 1.0/0.0;
      else         ref_inv = 1.0 / x;
    end
  endfunction

  // ----------------------------
  // Drive one transaction
  // ----------------------------
  task automatic do_one(input logic [31:0] x_bits);
    real x_r, inv_ref_r, inv_dut_r, prod_r;
    int unsigned timeout;
    begin
      // apply input for 1 cycle
      @(posedge clk);
      in_valid <= 1'b1;
      in_fp32  <= x_bits;

      @(posedge clk);
      in_valid <= 1'b0;
      in_fp32  <= 32'h0;

      // wait out_valid
      timeout = 0;
      while (!out_valid) begin
        @(posedge clk);
        timeout++;
        if (timeout > 5_000_000) begin
          $display("[TB][TIMEOUT] waiting out_valid for x=%08x", x_bits);
          $finish;
        end
      end

      // sample
      #1;
      x_r       = fp32_to_real(x_bits);
      inv_ref_r = ref_inv(x_r);
      inv_dut_r = fp32_to_real(out_fp32);
      prod_r    = x_r * inv_dut_r;

      $display("x=%08x (%f)  dut_inv=%08x (%f)  ref_inv=%f  x*dut_inv=%f  err=%f",
               x_bits, x_r,
               out_fp32, inv_dut_r,
               inv_ref_r,
               prod_r,
               (inv_dut_r - inv_ref_r));

      // gap 1 cycle (避免黏在一起)
      @(posedge clk);
    end
  endtask

  // ----------------------------
  // Main
  // ----------------------------
  initial begin
    // defaults
    in_valid = 1'b0;
    in_fp32  = 32'h0;

    // waveform (optional)
    $dumpfile("./vvp/tb_inv_nr.vcd");
    $dumpvars(0, tb_fp32_softmax_inv_nr);

    @(posedge rst_n);
    repeat (5) @(posedge clk);

    $display("=== TB: fp32_softmax_inv_nr ===");

    // ---- softmax常見：sum ~ 3.899139 (你 log 看到的) ----
    // 3.899139 的 bits 你 log 顯示是 0x40798b80
    do_one(32'h4079_8b80);

    // ---- 一些典型正數 ----
    do_one(32'h3f80_0000); // 1.0
    do_one(32'h4000_0000); // 2.0
    do_one(32'h4040_0000); // 3.0
    do_one(32'h4080_0000); // 4.0

    // ---- 接近 1 的值（模擬你 exp 後 sum）----
    do_one(32'h3f7f_ffff); // ~0.9999999
    do_one(32'h3f75_7000); // ~0.966171 (你 exp E 裡出現)
    do_one(32'h3f80_0000); // 1.0 again

    $display("[TB] DONE");
    $finish;
  end

endmodule

`default_nettype wire
