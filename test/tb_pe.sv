// tb_pe.sv
// iverilog -g2012 -Wall -I. -o ./vvp/tb_pe.vvp ./test/tb_pe.sv
// vvp ./vvp/tb_pe.vvp

`include "./src/pe.sv"
`timescale 1ns/1ps

module tb_pe;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // ---------------- DUT I/O ----------------
  logic        in_valid;
  logic [31:0] a_bits, b_bits, psum_in;

  logic        out_valid;
  logic [31:0] psum_out;

  pe dut (
    .clk      (clk),
    .rst      (rst),
    .in_valid (in_valid),
    .a_bits   (a_bits),
    .b_bits   (b_bits),
    .psum_in  (psum_in),
    .out_valid(out_valid),
    .psum_out (psum_out)
  );

  // ============================================================
  // Reference PE (bit-exact) using same drivers, SAME control as DUT
  // ============================================================
  typedef enum logic [2:0] {RIDLE, RSTART_MUL, RWAIT_MUL, RSTART_ADD, RWAIT_ADD} rst_t;
  rst_t rstt;

  logic [31:0] ra_r, rb_r, rpsum_r, rmul_r;

  logic        ref_mul_start, ref_add_start;
  logic        ref_mul_busy, ref_mul_done;
  logic [31:0] ref_mul_out;

  logic        ref_add_busy, ref_add_done;
  logic [31:0] ref_add_out;

  logic        ref_out_valid;
  logic [31:0] ref_psum_out;

  assign ref_mul_start = (rstt == RSTART_MUL) && !ref_mul_busy;
  assign ref_add_start = (rstt == RSTART_ADD) && !ref_add_busy;

  fp_mul_driver ref_mul (
    .clk   (clk),
    .rst   (rst),
    .start (ref_mul_start),
    .a_bits(ra_r),
    .b_bits(rb_r),
    .busy  (ref_mul_busy),
    .done  (ref_mul_done),
    .z_bits(ref_mul_out)
  );

  fp_adder_driver ref_add (
    .clk   (clk),
    .rst   (rst),
    .start (ref_add_start),
    .a_bits(rmul_r),
    .b_bits(rpsum_r),
    .busy  (ref_add_busy),
    .done  (ref_add_done),
    .z_bits(ref_add_out)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      rstt         <= RIDLE;
      ra_r         <= 32'd0;
      rb_r         <= 32'd0;
      rpsum_r      <= 32'd0;
      rmul_r       <= 32'd0;
      ref_out_valid<= 1'b0;
      ref_psum_out <= 32'd0;
    end else begin
      ref_out_valid <= 1'b0;

      case (rstt)
        RIDLE: begin
          if (in_valid) begin
            // 先鎖資料（與 DUT 一致）
            ra_r    <= a_bits;
            rb_r    <= b_bits;
            rpsum_r <= psum_in;
            rstt    <= RSTART_MUL;
          end
        end

        RSTART_MUL: begin
          if (!ref_mul_busy) rstt <= RWAIT_MUL;
        end

        RWAIT_MUL: begin
          if (ref_mul_done) begin
            rmul_r <= ref_mul_out;
            rstt   <= RSTART_ADD;
          end
        end

        RSTART_ADD: begin
          if (!ref_add_busy) rstt <= RWAIT_ADD;
        end

        RWAIT_ADD: begin
          if (ref_add_done) begin
            ref_psum_out  <= ref_add_out;
            ref_out_valid <= 1'b1;
            rstt          <= RIDLE;
          end
        end

        default: rstt <= RIDLE;
      endcase
    end
  end

  // ============================================================
  // Utilities
  // ============================================================

  task automatic apply_one(
    input logic [31:0] aa,
    input logic [31:0] bb,
    input logic [31:0] ps
  );
    begin
      // 將輸入在 negedge 更新，避免跟 posedge FF 競賽
      @(negedge clk);
      a_bits   <= aa;
      b_bits   <= bb;
      psum_in  <= ps;
      in_valid <= 1'b1;

      // 1-cycle pulse
      @(negedge clk);
      in_valid <= 1'b0;
    end
  endtask

  task automatic wait_both_done(
    input logic [31:0] aa,
    input logic [31:0] bb,
    input logic [31:0] ps
  );
    int unsigned timeout;
    begin
      timeout = 0;
      while ((out_valid !== 1'b1) || (ref_out_valid !== 1'b1)) begin
        @(posedge clk);
        timeout++;
        if (timeout > 200000) begin
          $fatal(1, "[TB] TIMEOUT waiting done. a=%08h b=%08h ps=%08h  dut_v=%b ref_v=%b",
                 aa, bb, ps, out_valid, ref_out_valid);
        end
      end
    end
  endtask

  task automatic check_one(
    input logic [31:0] aa,
    input logic [31:0] bb,
    input logic [31:0] ps
  );
    begin
      // 等雙方 done pulse 出現
      wait_both_done(aa, bb, ps);

      // 在 done 後一拍取樣（穩定）
      @(posedge clk);

      if (psum_out !== ref_psum_out) begin
        $display("[TB] FAIL a=%08h b=%08h ps=%08h got=%08h exp=%08h",
                 aa, bb, ps, psum_out, ref_psum_out);
        $fatal(1);
      end else begin
        $display("[TB] PASS a=%08h b=%08h ps=%08h out=%08h",
                 aa, bb, ps, psum_out);
      end
    end
  endtask

  task automatic run_one(
    input logic [31:0] aa,
    input logic [31:0] bb,
    input logic [31:0] ps
  );
    begin
      apply_one(aa, bb, ps);
      check_one(aa, bb, ps);
    end
  endtask

  // random helper: avoid NaN/Inf too often
  function automatic logic [31:0] scrub_fp(input logic [31:0] x);
    logic [31:0] y;
    begin
      y = x;
      if (y[30:23] == 8'hFF) y[30:23] = 8'h7E; // turn Inf/NaN into large finite
      scrub_fp = y;
    end
  endfunction

  // ============================================================
  // Main
  // ============================================================
  initial begin
    integer i;
    logic [31:0] ra, rb, rp;

    // init
    rst      = 1'b1;
    in_valid = 1'b0;
    a_bits   = 32'd0;
    b_bits   = 32'd0;
    psum_in  = 32'd0;

    // reset hold
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (3) @(posedge clk);

    $display("=== TB start: PE FP32 MAC (DUT vs REF drivers) ===");

    // -------- directed tests --------
    run_one(32'h0000_0000, 32'h0000_0000, 32'h0000_0000); // 0*0+0
    run_one(32'h4000_0000, 32'h4040_0000, 32'h0000_0000); // 2*3+0
    run_one(32'hBFC0_0000, 32'h4080_0000, 32'h4120_0000); // -1.5*4+10
    run_one(32'h3E80_0000, 32'hC100_0000, 32'hBF80_0000); // 0.25*-8 + -1
    run_one(32'h3F80_0000, 32'h3F80_0000, 32'h3F80_0000); // 1*1 + 1

    // -------- random tests --------
    for (i = 0; i < 200; i++) begin
      ra = scrub_fp($urandom());
      rb = scrub_fp($urandom());
      rp = scrub_fp($urandom());
      run_one(ra, rb, rp);
    end

    $display("[TB] ALL TESTS PASSED ✅");
    $finish;
  end

endmodule
