//iverilog -g2012 -Wall -I. -o tb_fp_adder_driver.vvp ./test/tb_fp_adder_driver.sv
//vvp tb_fp_adder_driver.vvp

`include "./src/EPU/attention_score/fp_adder_driver.sv"
`timescale 1ns/1ps
module tb_fp_adder_driver;

  logic clk = 0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  logic        start;
  logic [31:0] a_bits, b_bits;

  wire         busy;
  wire         done;
  wire [31:0]  z_bits;
  logic [31:0] z;

  fp_adder_driver dut (
    .clk   (clk),
    .rst   (rst),
    .start (start),
    .a_bits(a_bits),
    .b_bits(b_bits),
    .busy  (busy),
    .done  (done),
    .z_bits(z_bits)
  );

  task automatic do_add(
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] z
  );
    int timeout;
    begin
      // wait idle
      timeout = 0;
      while (busy) begin
        @(posedge clk);
        timeout++;
        if (timeout > 20000) begin
          $display("[TB] TIMEOUT waiting busy deassert");
          $finish;
        end
      end

      a_bits = a;
      b_bits = b;

      // start pulse
      @(posedge clk);
      start = 1'b1;
      @(posedge clk);
      start = 1'b0;

      // if done is sticky from previous op, wait it clears after start accepted
      timeout = 0;
      while (done) begin
        @(posedge clk);
        timeout++;
        if (timeout > 2000) begin
          $display("[TB] TIMEOUT waiting done clear");
          $finish;
        end
      end

      // wait done assert
      timeout = 0;
      while (!done) begin
        @(posedge clk);
        timeout++;
        if (timeout > 200000) begin
          $display("[TB] TIMEOUT waiting done");
          $finish;
        end
      end

      z = z_bits;
    end
  endtask

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

  initial begin
    $dumpfile("./vvp/tb_fp_adder_driver.vcd");
    $dumpvars(0, tb_fp_adder_driver);
    rst    = 1'b1;
    start  = 1'b0;
    a_bits = 32'd0;
    b_bits = 32'd0;

    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // 1.0 + 1.0 = 2.0
    do_add(32'h3f800000, 32'h3f800000, z);
    $display("[TB] 1.0+1.0 => z=0x%08x (expect 0x40000000)", z);

    // 2.0 + 0.5 = 2.5
    do_add(32'h40000000, 32'h3f000000, z);
    $display("[TB] 2.0+0.5 => z=0x%08x (expect 0x40200000)", z);

    // -1.5 + 2.0 = 0.5
    do_add(32'hbfc00000, 32'h40000000, z);
    $display("[TB] -1.5+2.0 => z=0x%08x (expect 0x3f000000)", z);

    // 4.1 + -3.2 = 0.9
    // 0.9 float = 0x3f666666 (常見近似；實際 rounding 可能 0x3f666667)
    do_add(32'h40833333, 32'hc04ccccd, z);
    $display("[TB] 4.1+(-3.2) => z=0x%08x (expect ~0x3f666666/67)", z);

    // -0.0 + +0.0 should be +0.0 (your adder has fix for -a+a)
    do_add(32'h80000000, 32'h00000000, z);
    $display("[TB] -0.0+0.0 => z=0x%08x (expect 0x00000000)", z);

    $display("[TB] DONE");
    $finish;
  end

endmodule
