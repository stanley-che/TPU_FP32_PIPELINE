// ============================================================================
// tb_fp32_exp_no_lut_sweep.sv
// Sweep x from 0 to -8 with step -1/128, test fp32_exp_no_lut
//
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_exp_sweep.vvp ./test/tb_fp32_exp_no_lut_swee.sv
//   vvp ./vvp/tb_exp_sweep.vvp
//   gtkwave ./vvp/tb_exp_sweep.vcd
// ============================================================================

`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp_mul_driver.sv"
`include "./src/EPU/attention_score/fp32_exp_no_lut.sv"   
`timescale 1ns/1ps
`default_nettype none   


module tb_fp32_exp_no_lut_sweep_handshake_fixed;

  // ----------------------------
  // clk / rst
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

  fp32_exp_no_lut dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .in_fp32  (in_fp32),
    .out_valid(out_valid),
    .out_fp32 (out_fp32)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("./vvp/tb_exp_sweep.vcd");
    $dumpvars(0, tb_fp32_exp_no_lut_sweep_handshake_fixed);
  end

  // ----------------------------
  // fp32 bits -> real (manual decode)
  // ----------------------------
  function automatic real fp32_bits_to_real(input logic [31:0] b);
    logic sign;
    int   exp_u;
    int   exp;
    int   frac;
    real  mant;
    real  val;
    begin
      sign  = b[31];
      exp_u = b[30:23];
      frac  = b[22:0];

      if (exp_u == 0) begin
        if (frac == 0) begin
          fp32_bits_to_real = 0.0;
        end else begin
          mant = frac / 8388608.0; // 2^23
          val  = mant * $pow(2.0, -126);
          fp32_bits_to_real = sign ? -val : val;
        end
      end else if (exp_u == 8'hFF) begin
        fp32_bits_to_real = 0.0; // TB simplified
      end else begin
        exp  = exp_u - 127;
        mant = 1.0 + (frac / 8388608.0);
        val  = mant * $pow(2.0, exp);
        fp32_bits_to_real = sign ? -val : val;
      end
    end
  endfunction

  // ----------------------------
  // build FP32 bits for x = -i/128 exactly (i>=0)
  // ----------------------------
  function automatic logic [31:0] fp32_from_neg_i_over_128(input int i);
    int p;
    int exp2;
    int exp_u;
    logic [55:0] norm;
    logic [23:0] mant24;
    begin
      if (i == 0) begin
        fp32_from_neg_i_over_128 = 32'h0000_0000;
      end else begin
        p = 31;
        while (p > 0 && (((i >> p) & 1) == 0)) p--;

        exp2  = p - 7;
        exp_u = exp2 + 127;

        if (exp_u <= 0) begin
          fp32_from_neg_i_over_128 = 32'h8000_0000;
        end else if (exp_u >= 255) begin
          fp32_from_neg_i_over_128 = 32'hff80_0000;
        end else begin
          norm = i;
          if (p > 23) norm = norm >> (p - 23);
          else        norm = norm << (23 - p);

          mant24 = norm[23:0];
          fp32_from_neg_i_over_128 = {1'b1, exp_u[7:0], mant24[22:0]};
        end
      end
    end
  endfunction

  function automatic real ref_exp(input real x);
    real E;
    begin
      E = 2.71828182845904523536;
      ref_exp = $pow(E, x);
    end
  endfunction

  // ----------------------------
  // drive one input: clock-aligned valid pulse
  // ----------------------------
  task automatic drive_one_aligned(input logic [31:0] x_bits);
    begin
      in_fp32  = x_bits;
      in_valid = 1'b0;
      @(posedge clk);
      in_valid = 1'b1;
      @(posedge clk);
      in_valid = 1'b0;
    end
  endtask

  // ----------------------------
  // wait for out_valid with timeout
  // ----------------------------
  task automatic wait_out_valid_or_timeout(input int max_cycles);
    int c;
    begin
      c = 0;
      while (out_valid !== 1'b1) begin
        @(posedge clk);
        c++;
        if (c >= max_cycles) begin
          $display("[TIMEOUT] wait out_valid over %0d cycles at time %0t", max_cycles, $time);
          $display("[TIMEOUT] st=%0d in_valid=%b in_fp32=%h out_valid=%b out_fp32=%h",
                   dut.st, in_valid, in_fp32, out_valid, out_fp32);
          $display("[TIMEOUT] mul_req=%b mul_busy=%b mul_done=%b add_req=%b add_busy=%b add_done=%b",
                   dut.mul_req, dut.mul_busy, dut.mul_done,
                   dut.add_req, dut.add_busy, dut.add_done);
          $finish;
        end
      end
    end
  endtask

  // ----------------------------
  // sweep
  // ----------------------------
  localparam int N_STEPS   = 8*128 + 1; // 1025
  localparam int TIMEOUT_C = 200000;    // 放大一點，避免你的 FP core latency 很長

  int  i;
  real x_r, dut_r, ref_r;
  real abs_err, rel_err;
  real max_abs_err, max_rel_err;
  int  max_abs_i, max_rel_i;

  real ABS_TH, REL_TH;

  initial begin
    in_valid = 1'b0;
    in_fp32  = 32'd0;

    max_abs_err = 0.0;
    max_rel_err = 0.0;
    max_abs_i   = 0;
    max_rel_i   = 0;

    ABS_TH = 1e-3;
    REL_TH = 5e-3;

    // reset release + guard
    wait (rst_n === 1'b1);
    @(posedge clk);
    @(posedge clk);

    // ---- sanity: i=0 ----
    $display("[TB] send i=0");
    drive_one_aligned(fp32_from_neg_i_over_128(0));
    @(posedge clk);
    $display("[TB] after send: st=%0d in_valid=%b", dut.st, in_valid);

    wait_out_valid_or_timeout(TIMEOUT_C);
    $display("[TB] got first out_valid at t=%0t out=%h", $time, out_fp32);
    @(posedge clk); // consume pulse

    // ---- full sweep ----
    for (i = 0; i < N_STEPS; i++) begin
      x_r = 0.0 - (i / 128.0);

      drive_one_aligned(fp32_from_neg_i_over_128(i));
      wait_out_valid_or_timeout(TIMEOUT_C);

      dut_r = fp32_bits_to_real(out_fp32);
      ref_r = ref_exp(x_r);

      abs_err = dut_r - ref_r;
      if (abs_err < 0.0) abs_err = -abs_err;

      if (ref_r != 0.0) rel_err = abs_err / ref_r;
      else              rel_err = abs_err;

      if (abs_err > max_abs_err) begin
        max_abs_err = abs_err;
        max_abs_i   = i;
      end
      if (rel_err > max_rel_err) begin
        max_rel_err = rel_err;
        max_rel_i   = i;
      end

      if ((i % 128) == 0) begin
        $display("[SAMPLE] i=%0d x=%f dut=%f ref=%f abs=%e rel=%e out_bits=%h",
                 i, x_r, dut_r, ref_r, abs_err, rel_err, out_fp32);
      end

      if ((abs_err > ABS_TH) && (rel_err > REL_TH)) begin
        $display("[FAIL]   i=%0d x=%f dut=%f ref=%f abs=%e rel=%e out_bits=%h",
                 i, x_r, dut_r, ref_r, abs_err, rel_err, out_fp32);
      end

      @(posedge clk); // consume out_valid pulse
    end

    $display("==================================================");
    $display("Done. Points=%0d", N_STEPS);
    $display("Max ABS err=%e at i=%0d x=%f", max_abs_err, max_abs_i, -(max_abs_i/128.0));
    $display("Max REL err=%e at i=%0d x=%f", max_rel_err, max_rel_i, -(max_rel_i/128.0));
    $display("==================================================");
    $finish;
  end

endmodule

`default_nettype wire
