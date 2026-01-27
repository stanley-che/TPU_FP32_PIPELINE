// ============================================================================
// tb_fp32_exp_matrix_flat.sv
// Testbench for fp32_exp_matrix_flat (flattened 2D exp, sequential)
//
// Fills in_flat[idx] with x = -(idx)/128 (idx = r*COLS + c)
// Pulses start, waits done (with timeout), then checks all outputs vs real exp(x)
//
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_exp_mat_flat.vvp ./test/tb_fp32_exp_matrix_flat1.sv
//   vvp ./vvp/tb_exp_mat_flat.vvp
//   gtkwave ./vvp/tb_exp_mat_flat.vcd
// ============================================================================

`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp_mul_driver.sv"
`include "./src/EPU/attention_score/fp32_exp_no_lut.sv"
`include "./src/EPU/attention_score/fp32_exp_matrix_2d.sv"

`timescale 1ns/1ps
`default_nettype none


module tb_fp32_exp_matrix_flat_sweep_0_to_m8;

  // ----------------------------
  // Params
  // ----------------------------
  localparam int unsigned ROWS   = 1;
  localparam int unsigned COLS   = 1025;  // 8*128+1
  localparam int unsigned DATA_W = 32;
  localparam int unsigned N      = ROWS*COLS;

  // safety
  initial begin
    if (N != 1025) begin
      $display("[FATAL] Need N=1025 for 0..-8 step 1/128. Got N=%0d (ROWS=%0d COLS=%0d)", N, ROWS, COLS);
      $finish;
    end
  end

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
  logic start;
  logic busy, done;

  logic [DATA_W*N-1:0] in_flat;
  logic [DATA_W*N-1:0] out_flat;

  logic [((ROWS<=1)?1:$clog2(ROWS))-1:0] cur_r;
  logic [((COLS<=1)?1:$clog2(COLS))-1:0] cur_c;
  logic [((N   <=1)?1:$clog2(N   ))-1:0] cur_idx;

  fp32_exp_matrix_flat #(
    .ROWS  (ROWS),
    .COLS  (COLS),
    .DATA_W(DATA_W)
  ) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (start),
    .busy     (busy),
    .done     (done),
    .in_flat  (in_flat),
    .out_flat (out_flat),
    .cur_r    (cur_r),
    .cur_c    (cur_c),
    .cur_idx  (cur_idx)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("./vvp/tb_exp_mat_flat.vcd");
    $dumpvars(0, tb_fp32_exp_matrix_flat_sweep_0_to_m8);
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
        fp32_from_neg_i_over_128 = 32'h0000_0000; // +0
      end else begin
        p = 31;
        while (p > 0 && (((i >> p) & 1) == 0)) p--;

        exp2  = p - 7;        // divide by 128
        exp_u = exp2 + 127;

        if (exp_u <= 0) begin
          fp32_from_neg_i_over_128 = 32'h8000_0000; // -0 underflow
        end else if (exp_u >= 255) begin
          fp32_from_neg_i_over_128 = 32'hff80_0000; // -inf
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
  // wait done with timeout
  // ----------------------------
  task automatic wait_done_or_timeout(input int max_cycles);
    int c;
    begin
      c = 0;
      while (done !== 1'b1) begin
        @(posedge clk);
        c++;
        if (c >= max_cycles) begin
          $display("[TIMEOUT] done not asserted over %0d cycles @t=%0t", max_cycles, $time);
          $display("[TIMEOUT] busy=%b cur_r=%0d cur_c=%0d cur_idx=%0d", busy, cur_r, cur_c, cur_idx);
          $finish;
        end
      end
    end
  endtask

  // pretty print matrix (row-major)
  task automatic print_matrix_bits_real(
    input string title,
    input logic [DATA_W*N-1:0] flat_bus
  );
    int rr, cc, idx;
    real v;
    begin
      $display("\n================ %s ================", title);
      for (rr = 0; rr < ROWS; rr++) begin
        for (cc = 0; cc < COLS; cc++) begin
          idx = rr*COLS + cc;
          v   = fp32_bits_to_real(flat_bus[DATA_W*idx +: DATA_W]);
          // 為了不刷爆 terminal：每行印 8 個
          $write("(%0d,%0d) %h %8.5f   ", rr, cc,
                 flat_bus[DATA_W*idx +: DATA_W], v);
          if (((cc+1) % 8) == 0) $write("\n");
        end
        if ((COLS % 8) != 0) $write("\n");
      end
      $display("=====================================================\n");
    end
  endtask

  // ----------------------------
  // Main
  // ----------------------------
  real x_r, dut_r, ref_r;
  real abs_err, rel_err;
  real max_abs_err, max_rel_err;
  int  max_abs_i, max_rel_i;

  real ABS_TH, REL_TH;

  initial begin
    start   = 1'b0;
    in_flat = '0;

    max_abs_err = 0.0;
    max_rel_err = 0.0;
    max_abs_i   = 0;
    max_rel_i   = 0;

    ABS_TH = 1e-3;
    REL_TH = 5e-3;

    // wait reset
    wait (rst_n === 1'b1);
    repeat (5) @(posedge clk);

    // ------------------------------------------
    // Fill input: x = -idx/128, idx=0..1024
    // ------------------------------------------
    for (int idx = 0; idx < N; idx++) begin
      in_flat[DATA_W*idx +: DATA_W] = fp32_from_neg_i_over_128(idx);
    end

    // 印 input（會很長，但每行 8 個）
    print_matrix_bits_real("INPUT MATRIX (x = -idx/128)", in_flat);

    // ------------------------------------------
    // Start DUT (1-cycle pulse)
    // ------------------------------------------
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    // Wait done
    wait_done_or_timeout(2_000_000);

    // consume done pulse
    @(posedge clk);

    // 印 output
    print_matrix_bits_real("OUTPUT MATRIX (exp(x))", out_flat);

    // ------------------------------------------
    // Check outputs (also print samples)
    // ------------------------------------------
    $display("==== Check outputs (sample every 128) ====");
    for (int idx = 0; idx < N; idx++) begin
      x_r   = 0.0 - (idx / 128.0);
      dut_r = fp32_bits_to_real(out_flat[DATA_W*idx +: DATA_W]);
      ref_r = ref_exp(x_r);

      abs_err = dut_r - ref_r;
      if (abs_err < 0.0) abs_err = -abs_err;

      if (ref_r != 0.0) rel_err = abs_err / ref_r;
      else              rel_err = abs_err;

      if (abs_err > max_abs_err) begin
        max_abs_err = abs_err;
        max_abs_i   = idx;
      end
      if (rel_err > max_rel_err) begin
        max_rel_err = rel_err;
        max_rel_i   = idx;
      end

      if ((idx % 128) == 0) begin
        $display("[SAMPLE] idx=%0d x=%f out_bits=%h dut=%f ref=%f abs=%e rel=%e",
                 idx, x_r, out_flat[DATA_W*idx +: DATA_W], dut_r, ref_r, abs_err, rel_err);
      end

      if ((abs_err > ABS_TH) && (rel_err > REL_TH)) begin
        $display("[FAIL] idx=%0d x=%f out_bits=%h dut=%f ref=%f abs=%e rel=%e",
                 idx, x_r, out_flat[DATA_W*idx +: DATA_W], dut_r, ref_r, abs_err, rel_err);
      end
    end

    $display("==================================================");
    $display("Done. N=%0d (ROWS=%0d COLS=%0d)", N, ROWS, COLS);
    $display("Max ABS err=%e at idx=%0d x=%f", max_abs_err, max_abs_i, -(max_abs_i/128.0));
    $display("Max REL err=%e at idx=%0d x=%f", max_rel_err, max_rel_i, -(max_rel_i/128.0));
    $display("==================================================");
    $finish;
  end

endmodule

`default_nettype wire
