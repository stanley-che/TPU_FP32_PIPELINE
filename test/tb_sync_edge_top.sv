// tb_sync_edge_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_sync_edge_top.vvp \
  ./test/tb_sync_edge_top.sv

vvp ./vvp/tb_sync_edge_top.vvp
*/

`include "./src/AMOLED/video_in_timing_if/sync_edge_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_sync_edge_top;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;   // 100 MHz
  logic rst;

  // ------------------------------------------------------------
  // inputs
  // ------------------------------------------------------------
  logic vsync_i, hsync_i, de_i;

  // ------------------------------------------------------------
  // DUT outputs (no filter)
  // ------------------------------------------------------------
  logic vs0, hs0, de0;
  logic vs0_rise, vs0_fall, hs0_rise, hs0_fall, de0_rise, de0_fall;

  // ------------------------------------------------------------
  // DUT outputs (with filter)
  // ------------------------------------------------------------
  logic vs1, hs1, de1;
  logic vs1_rise, vs1_fall, hs1_rise, hs1_fall, de1_rise, de1_fall;

  // ------------------------------------------------------------
  // DUT instances
  // ------------------------------------------------------------
  sync_edge_top #(
    .VS_INV(1'b0),
    .HS_INV(1'b0),
    .DE_INV(1'b0),
    .USE_DEGLITCH(1'b0),
    .STABLE_CYCLES(2)
  ) u_nofilter (
    .clk(clk), .rst(rst),
    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i),
    .vsync(vs0), .hsync(hs0), .de(de0),
    .vs_rise(vs0_rise), .vs_fall(vs0_fall),
    .hs_rise(hs0_rise), .hs_fall(hs0_fall),
    .de_rise(de0_rise), .de_fall(de0_fall)
  );

  sync_edge_top #(
    .VS_INV(1'b0),
    .HS_INV(1'b0),
    .DE_INV(1'b0),
    .USE_DEGLITCH(1'b1),
    .STABLE_CYCLES(2)
  ) u_filter (
    .clk(clk), .rst(rst),
    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i),
    .vsync(vs1), .hsync(hs1), .de(de1),
    .vs_rise(vs1_rise), .vs_fall(vs1_fall),
    .hs_rise(hs1_rise), .hs_fall(hs1_fall),
    .de_rise(de1_rise), .de_fall(de1_fall)
  );

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("tb_sync_edge_top.vcd");
    $dumpvars(0, tb_sync_edge_top);
  end

  // ------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------
  task automatic tick(input int n=1);
    repeat (n) @(posedge clk);
    #1ps; // NBA settle
  endtask

  task automatic expect_sig(
    input string name,
    input logic got,
    input logic exp
  );
    if (got !== exp) begin
      $display("[FAIL] %s got=%b exp=%b @t=%0t", name, got, exp, $time);
      $fatal(1);
    end else begin
      $display("[ OK ] %s got=%b exp=%b @t=%0t", name, got, exp, $time);
    end
  endtask

  task automatic expect_pulse(
    input string name,
    input logic got,
    input logic exp
  );
    if (got !== exp) begin
      $display("[FAIL] %s got=%b exp=%b @t=%0t", name, got, exp, $time);
      $fatal(1);
    end else begin
      $display("[ OK ] %s got=%b @t=%0t", name, got, $time);
    end
  endtask

  // ------------------------------------------------------------
  // Expected model for u_nofilter outputs:
  //   vs0/hs0/de0 == input delayed by 2 cycles
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  logic vs0_prev, hs0_prev, de0_prev;
  logic exp_vs_rise, exp_vs_fall, exp_hs_rise, exp_hs_fall, exp_de_rise, exp_de_fall;
  logic vs0_prev1, vs0_prev2;
  logic hs0_prev1, hs0_prev2; 
  logic de0_prev1, de0_prev2;
  initial begin
    $display("=== tb_sync_edge_top start ===");

    rst     = 1'b1;
    vsync_i = 1'b0;
    hsync_i = 1'b0;
    de_i    = 1'b0;

    tick(3);
    rst = 1'b0;
    tick(2);

    // ============================================================
    // TEST0: reset / no spurious pulses
    // ============================================================
    $display("\n--- TEST0: reset release no spurious pulses ---");
    expect_pulse("u0.vs_rise", vs0_rise, 1'b0);
    expect_pulse("u0.vs_fall", vs0_fall, 1'b0);
    expect_pulse("u0.hs_rise", hs0_rise, 1'b0);
    expect_pulse("u0.hs_fall", hs0_fall, 1'b0);
    expect_pulse("u0.de_rise", de0_rise, 1'b0);
    expect_pulse("u0.de_fall", de0_fall, 1'b0);

    // ============================================================
    // TEST1: u_nofilter vsync rise / fall (pulse +1clk)
    // ============================================================
    $display("\n--- TEST1: u_nofilter vsync rise / fall (pulse +1clk) ---");

    @(negedge clk); vsync_i = 1'b1;

    tick(1);
    expect_sig  ("u0.vsync after 1clk", vs0, 1'b0);
    expect_pulse("u0.vs_rise", vs0_rise, 1'b0);
    expect_pulse("u0.vs_fall", vs0_fall, 1'b0);

    tick(1);
    expect_sig  ("u0.vsync after 2clk", vs0, 1'b1);
    expect_pulse("u0.vs_rise (still 0)", vs0_rise, 1'b0);

    tick(1);
    expect_pulse("u0.vs_rise (delayed pulse)", vs0_rise, 1'b1);

    tick(1);
    expect_pulse("u0.vs_rise clear", vs0_rise, 1'b0);

    @(negedge clk); vsync_i = 1'b0;

    tick(1);
    expect_sig  ("u0.vsync fall after 1clk", vs0, 1'b1);
    expect_pulse("u0.vs_fall (still 0)", vs0_fall, 1'b0);

    tick(1);
    expect_sig  ("u0.vsync fall after 2clk", vs0, 1'b0);
    expect_pulse("u0.vs_fall (still 0)", vs0_fall, 1'b0);

    tick(1);
    expect_pulse("u0.vs_fall (delayed pulse)", vs0_fall, 1'b1);

    tick(1);
    expect_pulse("u0.vs_fall clear", vs0_fall, 1'b0);

    // ============================================================
    // TEST2: u_nofilter hsync / de quick check (pulse +1clk)
    // ============================================================
    $display("\n--- TEST2: u_nofilter hsync/de rise/fall (pulse +1clk) ---");

    // HS rise
    @(negedge clk); hsync_i = 1'b1;
    tick(2);
    expect_sig  ("u0.hsync after 2clk", hs0, 1'b1);
    expect_pulse("u0.hs_rise (still 0)", hs0_rise, 1'b0);
    tick(1);
    expect_pulse("u0.hs_rise (delayed)", hs0_rise, 1'b1);
    tick(1);
    expect_pulse("u0.hs_rise clear", hs0_rise, 1'b0);

    // HS fall
    @(negedge clk); hsync_i = 1'b0;
    tick(2);
    expect_sig  ("u0.hsync fall after 2clk", hs0, 1'b0);
    expect_pulse("u0.hs_fall (still 0)", hs0_fall, 1'b0);
    tick(1);
    expect_pulse("u0.hs_fall (delayed)", hs0_fall, 1'b1);
    tick(1);
    expect_pulse("u0.hs_fall clear", hs0_fall, 1'b0);

    // DE rise
    @(negedge clk); de_i = 1'b1;
    tick(2);
    expect_sig  ("u0.de after 2clk", de0, 1'b1);
    expect_pulse("u0.de_rise (still 0)", de0_rise, 1'b0);
    tick(1);
    expect_pulse("u0.de_rise (delayed)", de0_rise, 1'b1);
    tick(1);
    expect_pulse("u0.de_rise clear", de0_rise, 1'b0);

    // DE fall
    @(negedge clk); de_i = 1'b0;
    tick(2);
    expect_sig  ("u0.de fall after 2clk", de0, 1'b0);
    expect_pulse("u0.de_fall (still 0)", de0_fall, 1'b0);
    tick(1);
    expect_pulse("u0.de_fall (delayed)", de0_fall, 1'b1);
    tick(1);
    expect_pulse("u0.de_fall clear", de0_fall, 1'b0);

    // ============================================================
    // TEST3: u_filter glitch suppression (vsync)
    // ============================================================
    $display("\n--- TEST3: u_filter 1-cycle glitch suppressed ---");

    vsync_i = 1'b0;
    tick(5);
    expect_sig("u1.vsync steady low", vs1, 1'b0);

    @(negedge clk); vsync_i = 1'b1;
    tick(1);
    @(negedge clk); vsync_i = 1'b0;
    tick(1);

    tick(6);
    expect_sig  ("u1.vsync after glitch", vs1, 1'b0);
    expect_pulse("u1.vs_rise after glitch", vs1_rise, 1'b0);
    expect_pulse("u1.vs_fall after glitch", vs1_fall, 1'b0);

    // ============================================================
    // TEST4: u_filter stable transition should propagate
    // ============================================================
    $display("\n--- TEST4: u_filter stable transition propagates ---");

    @(negedge clk); vsync_i = 1'b1;
    tick(12);
    if (vs1 !== 1'b1) begin
      $display("[FAIL] u_filter.vsync did not become 1 @t=%0t", $time);
      $fatal(1);
    end else begin
      $display("[ OK ] u_filter.vsync became 1 @t=%0t", $time);
    end

    if (vs1_rise === 1'b1) begin
      tick(1);
      if (vs1_rise !== 1'b0) begin
        $display("[FAIL] u_filter.vs_rise stuck high @t=%0t", $time);
        $fatal(1);
      end
    end

    @(negedge clk); vsync_i = 1'b0;
    tick(12);
    if (vs1 !== 1'b0) begin
      $display("[FAIL] u_filter.vsync did not become 0 @t=%0t", $time);
      $fatal(1);
    end else begin
      $display("[ OK ] u_filter.vsync became 0 @t=%0t", $time);
    end

    if (vs1_fall === 1'b1) begin
      tick(1);
      if (vs1_fall !== 1'b0) begin
        $display("[FAIL] u_filter.vs_fall stuck high @t=%0t", $time);
        $fatal(1);
      end
    end

    // ============================================================
    // TEST5: u_nofilter random scoreboard (FIXED)
    // - outputs vs0/hs0/de0 must equal delayed-by-2 regs
    // - pulses must match edges between successive sampled vs0/hs0/de0
    //   using software "prev" variables (avoid NBA timing pitfall)
    // ============================================================
    $display("\n--- TEST5: u_nofilter random scoreboard ---");

    
    vs0_prev1 = vs0;  vs0_prev2 = vs0;
    hs0_prev1 = hs0;  hs0_prev2 = hs0;
    de0_prev1 = de0;  de0_prev2 = de0;

    repeat (50) begin
        @(negedge clk);
        vsync_i = $urandom_range(0,1);
        hsync_i = $urandom_range(0,1);
        de_i    = $urandom_range(0,1);

        @(posedge clk); #1ps;

        // outputs match delayed-by-2
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

        // IMPORTANT:
        // edge_pulse_gen generates pulse 1 cycle AFTER vs0 changes:
        // pulse(t) = edge(vs0 at t-1 vs t-2)
        exp_vs_rise =  (vs0_prev1 & ~vs0_prev2);
        exp_vs_fall = (~vs0_prev1 &  vs0_prev2);

        exp_hs_rise =  (hs0_prev1 & ~hs0_prev2);
        exp_hs_fall = (~hs0_prev1 &  hs0_prev2);

        exp_de_rise =  (de0_prev1 & ~de0_prev2);
        exp_de_fall = (~de0_prev1 &  de0_prev2);

        if (vs0_rise !== exp_vs_rise) begin
            $display("[FAIL] vs0_rise mismatch got=%b exp=%b @t=%0t", vs0_rise, exp_vs_rise, $time);
            $fatal(1);
        end
        if (vs0_fall !== exp_vs_fall) begin
            $display("[FAIL] vs0_fall mismatch got=%b exp=%b @t=%0t", vs0_fall, exp_vs_fall, $time);
            $fatal(1);
        end
        if (hs0_rise !== exp_hs_rise) begin
            $display("[FAIL] hs0_rise mismatch got=%b exp=%b @t=%0t", hs0_rise, exp_hs_rise, $time);
            $fatal(1);
        end
        if (hs0_fall !== exp_hs_fall) begin
            $display("[FAIL] hs0_fall mismatch got=%b exp=%b @t=%0t", hs0_fall, exp_hs_fall, $time);
            $fatal(1);
        end
        if (de0_rise !== exp_de_rise) begin
            $display("[FAIL] de0_rise mismatch got=%b exp=%b @t=%0t", de0_rise, exp_de_rise, $time);
            $fatal(1);
        end
        if (de0_fall !== exp_de_fall) begin
            $display("[FAIL] de0_fall mismatch got=%b exp=%b @t=%0t", de0_fall, exp_de_fall, $time);
            $fatal(1);
        end

        // shift history AFTER checks
        vs0_prev2 = vs0_prev1;
        vs0_prev1 = vs0;

        hs0_prev2 = hs0_prev1;
        hs0_prev1 = hs0;

        de0_prev2 = de0_prev1;
        de0_prev1 = de0;
    end

    $display("[ OK ] u_nofilter scoreboard passed");
    // ============================================================
    // TEST6: u_nofilter fast toggle stress (010101...)
    // ============================================================
    $display("\n--- TEST6: u_nofilter fast toggle stress ---");

    vsync_i = 0;
    hsync_i = 0;
    de_i    = 0;
    tick(4);

    repeat (10) begin
        @(negedge clk);
        vsync_i = ~vsync_i;
        hsync_i = ~hsync_i;
        de_i    = ~de_i;

        @(posedge clk); #1ps;

        // still must match delayed-by-2
        if (vs0 !== vs_d2) $fatal(1, "TEST6 vs0 mismatch");
        if (hs0 !== hs_d2) $fatal(1, "TEST6 hs0 mismatch");
        if (de0 !== de_d2) $fatal(1, "TEST6 de0 mismatch");

    end

    $display("[ OK ] TEST6 passed");
    // ============================================================
    // TEST7: u_filter exact STABLE_CYCLES pass (2 cycles)
    // ============================================================
    $display("\n--- TEST7: u_filter exact STABLE_CYCLES pass ---");

    vsync_i = 0;
    tick(5);

    @(negedge clk); vsync_i = 1;
    tick(2);   // exactly 2 stable cycles

    // 再等 pipeline 出來
    tick(6);

    if (vs1 !== 1'b1) begin
        $display("[FAIL] TEST7 vs1 should become 1 @t=%0t", $time);
        $fatal(1);
    end

    $display("[ OK ] TEST7 passed");
    // ============================================================
    // TEST8: u_filter < STABLE_CYCLES suppressed
    // ============================================================
    $display("\n--- TEST8: u_filter short pulse suppressed ---");

    vsync_i = 0;
    tick(5);

    @(negedge clk); vsync_i = 1;
    tick(1);              // only 1 cycle
    @(negedge clk); vsync_i = 0;

    tick(8);

    if (vs1 !== 1'b0) begin
        $display("[FAIL] TEST8 vs1 glitch leaked @t=%0t", $time);
        $fatal(1);
    end
    if (vs1_rise || vs1_fall) begin
        $display("[FAIL] TEST8 pulse should not occur @t=%0t", $time);
        $fatal(1);
    end

    $display("[ OK ] TEST8 passed");
    // ============================================================
    // TEST9: reset asserted during active signal
    // ============================================================
    $display("\n--- TEST9: reset mid-active ---");
    // assert reset while vsync active
    @(negedge clk);
    rst = 1'b1;

    // 同時把 inputs 拉到 0，避免 reset 解除後又被同步回 1
    vsync_i = 1'b0;
    hsync_i = 1'b0;
    de_i    = 1'b0;

    tick(2);

    // reset 期間應該清 0（這時 input 也為 0）
    if (vs0 !== 1'b0) $fatal(1, "TEST9 vs0 not cleared during reset");

    rst = 1'b0;
    tick(4); // 給同步器兩拍以上時間穩定

    // reset 後也應維持 0（因為 input 現在是 0）
    if (vs0 !== 1'b0) $fatal(1, "TEST9 vs0 not 0 after reset release");

    // 不應有假 pulse
    if (vs0_rise || vs0_fall) begin
        $display("[FAIL] TEST9 spurious pulse after reset @t=%0t", $time);
        $fatal(1);
    end

    $display("[ OK ] TEST9 passed");

    $display("\n=== ALL TESTS PASSED ===");
    $finish;
  end

endmodule

`default_nettype wire
