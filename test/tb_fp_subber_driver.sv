// iverilog -g2012 -Wall -I. -o tb_fp_subber_driver.vvp ./test/tb_fp_subber_driver.sv
// vvp tb_fp_subber_driver.vvp

`include "fp_subber_driver.sv"
`timescale 1ns/1ps
module tb_fp_subber_driver;

  logic clk = 0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  logic        start;
  logic [31:0] a_bits, b_bits;

  wire         busy;
  wire         done;
  wire [31:0]  z_bits;
  logic [31:0] z;

  fp_subber_driver dut (
    .clk   (clk),
    .rst   (rst),
    .start (start),
    .a_bits(a_bits),
    .b_bits(b_bits),
    .busy  (busy),
    .done  (done),
    .z_bits(z_bits)
  );

  task automatic do_sub(
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

      // done is sticky from previous op; after start accepted, done should clear
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

  // Simple assert helper
  task automatic expect_hex(
    input string name,
    input logic [31:0] got,
    input logic [31:0] exp
  );
    begin
      if (got !== exp) begin
        $display("[TB] FAIL %s got=0x%08x expect=0x%08x", name, got, exp);
        $finish;
      end else begin
        $display("[TB] PASS %s => 0x%08x", name, got);
      end
    end
  endtask

  // NaN checker (payload may vary)
  function automatic bit is_nan32(input logic [31:0] x);
    return (x[30:23] == 8'hFF) && (x[22:0] != 0);
  endfunction

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
    rst    = 1'b1;
    start  = 1'b0;
    a_bits = 32'd0;
    b_bits = 32'd0;

    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // -----------------------------
    // Deterministic cases (exact hex)
    // -----------------------------

    // 2.0 - 1.0 = 1.0
    do_sub(32'h40000000, 32'h3f800000, z);
    expect_hex("2.0-1.0", z, 32'h3f800000);

    // 1.0 - 2.0 = -1.0
    do_sub(32'h3f800000, 32'h40000000, z);
    expect_hex("1.0-2.0", z, 32'hbf800000);

    // 3.5 - 1.25 = 2.25
    // 3.5  = 0x40600000
    // 1.25 = 0x3fa00000
    // 2.25 = 0x40100000
    do_sub(32'h40600000, 32'h3fa00000, z);
    expect_hex("3.5-1.25", z, 32'h40100000);

    // 1.0 - (-1.0) = 2.0
    do_sub(32'h3f800000, 32'hbf800000, z);
    expect_hex("1.0-(-1.0)", z, 32'h40000000);

    // (-1.0) - (1.0) = -2.0
    do_sub(32'hbf800000, 32'h3f800000, z);
    expect_hex("-1.0-1.0", z, 32'hc0000000);

    // 0.0 - 0.0 = +0.0
    do_sub(32'h00000000, 32'h00000000, z);
    // allow +0 only if you want strict; most designs accept either.
    // your core fixes cancellation to +0, so check +0:
    expect_hex("0-0", z, 32'h00000000);

    // 0.0 - 5.0 = -5.0
    // 5.0 = 0x40a00000
    do_sub(32'h00000000, 32'h40a00000, z);
    expect_hex("0-5", z, 32'hc0a00000);

    // 5.0 - 0.0 = 5.0
    do_sub(32'h40a00000, 32'h00000000, z);
    expect_hex("5-0", z, 32'h40a00000);

    // +inf - +inf = NaN  (payload may vary)
    do_sub(32'h7f800000, 32'h7f800000, z);
    if (!is_nan32(z)) begin
      $display("[TB] FAIL +inf-+inf expected NaN, got=0x%08x", z);
      $finish;
    end else begin
      $display("[TB] PASS +inf-+inf => NaN (0x%08x)", z);
    end

    // -----------------------------
    $display("[TB] DONE");
    $finish;
  end

endmodule
