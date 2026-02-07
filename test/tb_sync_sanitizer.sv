// tb_sync_sanitizer.sv
/*
iverilog -g2012 -Wall -o ./vvp/tb_sync_sanitizer.vvp ./test/tb_sync_sanitizer.sv
vvp ./vvp/tb_sync_sanitizer.vvp
*/
`include "./src/AMOLED/video_in_timing_if/sync_sanitizer.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_sync_sanitizer;

  // -----------------------------
  // clock / reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;   // 100MHz

  logic rst;

  // -----------------------------
  // inputs
  // -----------------------------
  logic vsync_i, hsync_i, de_i;

  // -----------------------------
  // DUT outputs
  // -----------------------------
  logic vs0, hs0, de0; // no filter
  logic vs1, hs1, de1; // with filter

  // -----------------------------
  // DUT instances
  // -----------------------------
  sync_sanitizer #(
    .VS_INV(1'b0),
    .HS_INV(1'b0),
    .DE_INV(1'b0),
    .USE_DEGLITCH(1'b0),
    .STABLE_CYCLES(2)
  ) u_nofilter (
    .clk(clk), .rst(rst),
    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i),
    .vsync(vs0), .hsync(hs0), .de(de0)
  );

  sync_sanitizer #(
    .VS_INV(1'b0),
    .HS_INV(1'b0),
    .DE_INV(1'b0),
    .USE_DEGLITCH(1'b1),
    .STABLE_CYCLES(2)
  ) u_filter (
    .clk(clk), .rst(rst),
    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i),
    .vsync(vs1), .hsync(hs1), .de(de1)
  );

  // -----------------------------
  // simple scoreboard for u_nofilter (2FF delay check)
  // expected = input delayed by 2 clocks (after polarity)
  // -----------------------------
  logic vs_d1, vs_d2;
  logic hs_d1, hs_d2;
  logic de_d1, de_d2;

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_d1 <= 0; vs_d2 <= 0;
      hs_d1 <= 0; hs_d2 <= 0;
      de_d1 <= 0; de_d2 <= 0;
    end else begin
      vs_d1 <= vsync_i; vs_d2 <= vs_d1;
      hs_d1 <= hsync_i; hs_d2 <= hs_d1;
      de_d1 <= de_i;    de_d2 <= de_d1;
    end
  end

  // -----------------------------
  // helper tasks
  // -----------------------------
  task automatic tick(input int n=1);
    repeat(n) @(posedge clk);
  endtask

  // 等 NBA 更新完成再取樣（重點修正）
  task automatic tick_and_settle(input int n=1);
    tick(n);
    #1ps;
  endtask

  task automatic expect_eq(input string name, input logic got, input logic exp);
    if (got !== exp) begin
      $display("[FAIL] %s got=%b exp=%b @t=%0t", name, got, exp, $time);
      $fatal(1);
    end else begin
      $display("[ OK ] %s got=%b exp=%b @t=%0t", name, got, exp, $time);
    end
  endtask

  // -----------------------------
  // Main
  // -----------------------------
  initial begin
    $display("=== tb_sync_sanitizer start ===");
    rst     = 1'b1;
    vsync_i = 1'b0;
    hsync_i = 1'b0;
    de_i    = 1'b0;

    tick_and_settle(5);
    rst = 1'b0;
    tick_and_settle(2);

    // ------------------------------------------------------------
    // TEST 1: pure 2FF delay (edge-safe)
    // ------------------------------------------------------------
    $display("\n--- TEST1: 2FF delay check (u_nofilter) ---");

    // drive input safely before clock edge
    @(negedge clk);
    vsync_i = 1'b1;

    // 1st capture
    @(posedge clk); #1ps;
    expect_eq("u_nofilter.vsync after 1 clk", vs0, 1'b0);

    // 2nd capture -> output should update
    @(posedge clk); #1ps;
    expect_eq("u_nofilter.vsync after 2 clk", vs0, 1'b1);

    // toggle back
    @(negedge clk);
    vsync_i = 1'b0;

    @(posedge clk); #1ps;
    expect_eq("u_nofilter.vsync fall after 1 clk", vs0, 1'b1);

    @(posedge clk); #1ps;
    expect_eq("u_nofilter.vsync fall after 2 clk", vs0, 1'b0);

    // ------------------------------------------------------------
    // TEST 2: make a 1-cycle glitch, u_filter should suppress it
    //         (STABLE_CYCLES=2)
    // ------------------------------------------------------------
    $display("\n--- TEST2: 1-cycle glitch should be filtered (u_filter) ---");

    // ensure steady low
    vsync_i = 1'b0;
    tick_and_settle(3);
    expect_eq("u_filter.vsync steady low", vs1, 1'b0);

    // 1-cycle glitch high then low (用 negedge 避免撞到取樣點)
    @(negedge clk); vsync_i = 1'b1;
    tick_and_settle(1);
    @(negedge clk); vsync_i = 1'b0;
    tick_and_settle(1);

    // wait a few cycles; filtered output should remain 0
    tick_and_settle(4);
    expect_eq("u_filter.vsync after glitch", vs1, 1'b0);

    // ------------------------------------------------------------
    // TEST 3: stable change for >= STABLE_CYCLES should pass
    // ------------------------------------------------------------
    $display("\n--- TEST3: stable transition should propagate (u_filter) ---");

    // hold input high long enough
    @(negedge clk);
    vsync_i = 1'b1;

    // filter pipeline has 2FF + stable counter
    tick_and_settle(8);
    if (vs1 !== 1'b1) begin
      $display("[FAIL] u_filter.vsync did not become 1 after stable high @t=%0t", $time);
      $fatal(1);
    end else begin
      $display("[ OK ] u_filter.vsync became 1 after stable high @t=%0t", $time);
    end

    // bring back low stable
    @(negedge clk);
    vsync_i = 1'b0;

    tick_and_settle(8);
    if (vs1 !== 1'b0) begin
      $display("[FAIL] u_filter.vsync did not become 0 after stable low @t=%0t", $time);
      $fatal(1);
    end else begin
      $display("[ OK ] u_filter.vsync became 0 after stable low @t=%0t", $time);
    end

    // ------------------------------------------------------------
    // TEST 4: continuous check for u_nofilter matches 2-cycle delay regs
    // ------------------------------------------------------------
    $display("\n--- TEST4: continuous scoreboard for u_nofilter ---");
    repeat (20) begin
      @(negedge clk);
      vsync_i = $urandom_range(0,1);
      hsync_i = $urandom_range(0,1);
      de_i    = $urandom_range(0,1);

      tick_and_settle(1);

      // compare outputs to delayed-by-2 (vs_d2/hs_d2/de_d2)
      if (vs0 !== vs_d2) begin
        $display("[FAIL] vs0 mismatch got=%b exp=%b @t=%0t", vs0, vs_d2, $time);
        $fatal(1);
      end
      if (hs0 !== hs_d2) begin
        $display("[FAIL] hs0 mismatch got=%b exp=%b @t=%0t", hs0, hs_d2, $time);
        $fatal(1);
      end
      if (de0 !== de_d2) begin
        $display("[FAIL] de0 mismatch got=%b exp=%b @t=%0t", de0, de_d2, $time);
        $fatal(1);
      end
    end
    $display("[ OK ] u_nofilter scoreboard passed");

    $display("\n=== ALL TESTS PASSED ===");
    $finish;
  end

endmodule

`default_nettype wire
