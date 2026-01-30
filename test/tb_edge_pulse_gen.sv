// tb_edge_pulse_gen.sv
/*
iverilog -g2012 -Wall \
  -o ./vvp/tb_edge_pulse_gen.vvp \
  ./test/tb_edge_pulse_gen.sv

vvp ./vvp/tb_edge_pulse_gen.vvp
*/
`include "./src/AMOLED/video_in_timing_if/edge_pulse_gen.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_edge_pulse_gen;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;   // 100 MHz

  logic rst;

  // ------------------------------------------------------------
  // inputs
  // ------------------------------------------------------------
  logic vsync, hsync, de;

  // ------------------------------------------------------------
  // outputs
  // ------------------------------------------------------------
  logic vs_rise, vs_fall;
  logic hs_rise, hs_fall;
  logic de_rise, de_fall;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  edge_pulse_gen dut (
    .clk     (clk),
    .rst     (rst),
    .vsync   (vsync),
    .hsync   (hsync),
    .de      (de),
    .vs_rise (vs_rise),
    .vs_fall (vs_fall),
    .hs_rise (hs_rise),
    .hs_fall (hs_fall),
    .de_rise (de_rise),
    .de_fall (de_fall)
  );

  // ------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------
  task automatic tick(input int n = 1);
    repeat (n) @(posedge clk);
    #1ps; // NBA settle
  endtask

  task automatic expect_pulse(
    input string name,
    input logic pulse,
    input logic exp
  );
    if (pulse !== exp) begin
      $display("[FAIL] %s got=%b exp=%b @t=%0t", name, pulse, exp, $time);
      $fatal(1);
    end else begin
      $display("[ OK ] %s got=%b @t=%0t", name, pulse, $time);
    end
  endtask

  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $display("=== tb_edge_pulse_gen start ===");

    // init
    rst   = 1'b1;
    vsync = 0;
    hsync = 0;
    de    = 0;

    tick(3);
    rst = 1'b0;
    tick(2);

    // ============================================================
    // TEST 1: vsync rise / fall
    // ============================================================
    $display("\n--- TEST1: vsync rise / fall ---");

    @(negedge clk); vsync = 1'b1;
    tick(1);
    expect_pulse("vs_rise", vs_rise, 1'b1);
    expect_pulse("vs_fall", vs_fall, 1'b0);

    tick(1);
    expect_pulse("vs_rise clear", vs_rise, 1'b0);

    @(negedge clk); vsync = 1'b0;
    tick(1);
    expect_pulse("vs_fall", vs_fall, 1'b1);
    expect_pulse("vs_rise", vs_rise, 1'b0);

    tick(1);
    expect_pulse("vs_fall clear", vs_fall, 1'b0);

    // ============================================================
    // TEST 2: hsync rise / fall
    // ============================================================
    $display("\n--- TEST2: hsync rise / fall ---");

    @(negedge clk); hsync = 1'b1;
    tick(1);
    expect_pulse("hs_rise", hs_rise, 1'b1);
    expect_pulse("hs_fall", hs_fall, 1'b0);

    tick(1);
    expect_pulse("hs_rise clear", hs_rise, 1'b0);

    @(negedge clk); hsync = 1'b0;
    tick(1);
    expect_pulse("hs_fall", hs_fall, 1'b1);
    expect_pulse("hs_rise", hs_rise, 1'b0);

    tick(1);
    expect_pulse("hs_fall clear", hs_fall, 1'b0);

    // ============================================================
    // TEST 3: de rise / fall
    // ============================================================
    $display("\n--- TEST3: de rise / fall ---");

    @(negedge clk); de = 1'b1;
    tick(1);
    expect_pulse("de_rise", de_rise, 1'b1);
    expect_pulse("de_fall", de_fall, 1'b0);

    tick(1);
    expect_pulse("de_rise clear", de_rise, 1'b0);

    @(negedge clk); de = 1'b0;
    tick(1);
    expect_pulse("de_fall", de_fall, 1'b1);
    expect_pulse("de_rise", de_rise, 1'b0);

    tick(1);
    expect_pulse("de_fall clear", de_fall, 1'b0);

    // ============================================================
    // TEST 4: no double pulse (hold signal)
    // ============================================================
    $display("\n--- TEST4: no double pulse when holding ---");

    @(negedge clk); vsync = 1'b1;
    tick(5);
    if (vs_rise !== 1'b0 || vs_fall !== 1'b0) begin
      $display("[FAIL] unexpected pulse while holding vsync");
      $fatal(1);
    end
    $display("[ OK ] no spurious pulse while holding");

    // ============================================================
    $display("\n=== ALL TESTS PASSED ===");
    $finish;
  end

endmodule

`default_nettype wire
