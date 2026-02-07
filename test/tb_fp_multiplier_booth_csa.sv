// Testbench for floating-point multiplier with Booth encoding and CSA
// iverilog -g2012 -Wall -I. -o tb_fp_mul_driver.vvp ./test/tb_fp_multiplier_booth_csa.sv
// vvp tb_fp_mul_driver.vvp
`include "fp_mul_driver.sv"
`timescale 1ns/1ps
module tb_fp_mul_driver;

  logic clk = 0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  logic        start;
  logic [31:0] a_bits, b_bits;

  wire         busy;
  wire         done;
  wire [31:0]  z_bits;
  logic [31:0] z;

  // DUT = driver wrapper
  fp_mul_driver dut (
    .clk   (clk),
    .rst   (rst),
    .start (start),
    .a_bits(a_bits),
    .b_bits(b_bits),
    .busy  (busy),
    .done  (done),
    .z_bits(z_bits)
  );

  // --- utility: one transaction (start -> wait done -> get z)
  task automatic do_mul(
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] z
  );
    int timeout;
    begin
      // wait until idle
      timeout = 0;
      while (busy) begin
        @(posedge clk);
        timeout++;
        if (timeout > 20000) begin
          $display("[TB] TIMEOUT waiting busy deassert");
          $finish;
        end
      end

      // set operands (use blocking to avoid NBA race)
      a_bits = a;
      b_bits = b;

      // pulse start 1 cycle (blocking at clock edges)
      @(posedge clk);
      start = 1'b1;
      @(posedge clk);
      start = 1'b0;

      // ---- robust wait done (works for pulse OR sticky-level done) ----
      // If driver uses sticky done, it might still be 1 from previous op.
      // After a new start is accepted, driver should clear done -> wait for that.
      timeout = 0;
      while (done) begin
        @(posedge clk);
        timeout++;
        if (timeout > 2000) begin
          $display("[TB] TIMEOUT waiting done to clear after start");
          $finish;
        end
      end

      // Now wait for done assertion (pulse or rising edge)
      timeout = 0;
      while (!done) begin
        @(posedge clk);
        timeout++;
        if (timeout > 200000) begin
          $display("[TB] TIMEOUT waiting done");
          $finish;
        end
      end
      // （或你也可以用：@(posedge done); 但 sticky done 已處理過也 OK）

      // capture result
      z = z_bits;
    end
  endtask

  // Optional debug
  initial begin
    if ($test$plusargs("DEBUG")) begin
      $display("[TB] DEBUG enabled");
      forever begin
        @(posedge clk);
        $display("t=%0t busy=%b done=%b start=%b a=%h b=%h z=%h",
                 $time, busy, done, start, a_bits, b_bits, z_bits);
      end
    end
  end

  // main
  initial begin
    rst    = 1'b1;
    start  = 1'b0;
    a_bits = 32'd0;
    b_bits = 32'd0;

    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // 1.0 * 1.0 = 1.0
    do_mul(32'h3f800000, 32'h3f800000, z);
    $display("[TB] 1.0*1.0 => z=0x%08x (expect 0x3f800000)", z);

    // 2.0 * 0.5 = 1.0
    do_mul(32'h40000000, 32'h3f000000, z);
    $display("[TB] 2.0*0.5 => z=0x%08x (expect 0x3f800000)", z);

    // -1.5 * 2.0 = -3.0
    do_mul(32'hbfc00000, 32'h40000000, z);
    $display("[TB] -1.5*2.0 => z=0x%08x (expect 0xc0400000)", z);

    // 4.1 * -3.2 = -13.12
    do_mul(32'h40833333, 32'hc04ccccd, z);
    $display("[TB] 4.1*-3.2 => z=0x%08x (expect 0xc151eb85)", z);

    $display("[TB] DONE");
    $finish;
  end

endmodule
