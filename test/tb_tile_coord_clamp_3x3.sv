// ============================================================
// tb_tile_coord_clamp_3x3.sv  (IVERILOG-SAFE)
// - Tests tile_coord_clamp_3x3 (pipelined 1-deep)
// - Checks:
//   * 3x3 mapping + clamp at borders
//   * nb_is_center one-hot at idx=4
//   * Ready/valid stability under backpressure (hold outputs)
//   * Overwrite-on-pop behavior (accept new center on same cycle as out_fire)
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_tile_coord_clamp_3x3.vvp ./test/tb_tile_coord_clamp_3x3.sv
// Run:
//   vvp ./vvp/tb_tile_coord_clamp_3x3.vvp
// ============================================================
`include "./src/AMOLED/tile_neighborhood_fetch/tile_coord_clamp_3x3.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_tile_coord_clamp_3x3;

  // ----------------------------
  // Params (small for TB)
  // ----------------------------
  localparam int unsigned TILES_X = 8;
  localparam int unsigned TILES_Y = 4;
  localparam int unsigned TAG_W   = 16;

  localparam int unsigned IW = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW = (TILES_X <= 1) ? 1 : $clog2(TILES_X);

  // ----------------------------
  // Clock/reset
  // ----------------------------
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                  center_valid;
  wire                 center_ready;
  reg  [IW-1:0]         center_i;
  reg  [JW-1:0]         center_j;
  reg  [TAG_W-1:0]      center_tag;

  wire                 nb_valid;
  reg                  nb_ready;

  wire [IW-1:0] nb_i0; wire [JW-1:0] nb_j0;
  wire [IW-1:0] nb_i1; wire [JW-1:0] nb_j1;
  wire [IW-1:0] nb_i2; wire [JW-1:0] nb_j2;
  wire [IW-1:0] nb_i3; wire [JW-1:0] nb_j3;
  wire [IW-1:0] nb_i4; wire [JW-1:0] nb_j4;
  wire [IW-1:0] nb_i5; wire [JW-1:0] nb_j5;
  wire [IW-1:0] nb_i6; wire [JW-1:0] nb_j6;
  wire [IW-1:0] nb_i7; wire [JW-1:0] nb_j7;
  wire [IW-1:0] nb_i8; wire [JW-1:0] nb_j8;

  wire [TAG_W-1:0] nb_tag;
  wire [8:0]       nb_is_center;

  tile_coord_clamp_3x3 #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W(TAG_W)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i(center_i),
    .center_j(center_j),
    .center_tag(center_tag),

    .nb_valid(nb_valid),
    .nb_ready(nb_ready),

    .nb_i0(nb_i0), .nb_j0(nb_j0),
    .nb_i1(nb_i1), .nb_j1(nb_j1),
    .nb_i2(nb_i2), .nb_j2(nb_j2),
    .nb_i3(nb_i3), .nb_j3(nb_j3),
    .nb_i4(nb_i4), .nb_j4(nb_j4),
    .nb_i5(nb_i5), .nb_j5(nb_j5),
    .nb_i6(nb_i6), .nb_j6(nb_j6),
    .nb_i7(nb_i7), .nb_j7(nb_j7),
    .nb_i8(nb_i8), .nb_j8(nb_j8),

    .nb_tag(nb_tag),
    .nb_is_center(nb_is_center)
  );

  // ----------------------------
  // Dump
  // ----------------------------
  initial begin
    $dumpfile("tb_tile_coord_clamp_3x3.vcd");
    $dumpvars(0, tb_tile_coord_clamp_3x3);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  function integer clamp_int(input integer v, input integer lo, input integer hi);
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  task expect_eq_int(input integer got, input integer exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s  got=%0d exp=%0d", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_hex(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s  got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_onehot_center;
    begin
      if (nb_is_center !== 9'b000010000) begin
        $display("[%0t] FAIL: nb_is_center not onehot idx=4  got=%b", $time, nb_is_center);
        $fatal(1);
      end
    end
  endtask

  // Read current nb outputs into ints (for stability checks)
  integer hold_i0, hold_i1, hold_i2, hold_i3, hold_i4, hold_i5, hold_i6, hold_i7, hold_i8;
  integer hold_j0, hold_j1, hold_j2, hold_j3, hold_j4, hold_j5, hold_j6, hold_j7, hold_j8;
  reg [TAG_W-1:0] hold_tag;

  task snap_outputs;
    begin
      hold_tag = nb_tag;
      hold_i0 = nb_i0; hold_j0 = nb_j0;
      hold_i1 = nb_i1; hold_j1 = nb_j1;
      hold_i2 = nb_i2; hold_j2 = nb_j2;
      hold_i3 = nb_i3; hold_j3 = nb_j3;
      hold_i4 = nb_i4; hold_j4 = nb_j4;
      hold_i5 = nb_i5; hold_j5 = nb_j5;
      hold_i6 = nb_i6; hold_j6 = nb_j6;
      hold_i7 = nb_i7; hold_j7 = nb_j7;
      hold_i8 = nb_i8; hold_j8 = nb_j8;
    end
  endtask

  task expect_outputs_held(input [200*8-1:0] msg);
    begin
      expect_eq_hex(nb_tag, hold_tag, {msg, " tag hold"});
      expect_eq_int(nb_i0, hold_i0, {msg," i0 hold"}); expect_eq_int(nb_j0, hold_j0, {msg," j0 hold"});
      expect_eq_int(nb_i1, hold_i1, {msg," i1 hold"}); expect_eq_int(nb_j1, hold_j1, {msg," j1 hold"});
      expect_eq_int(nb_i2, hold_i2, {msg," i2 hold"}); expect_eq_int(nb_j2, hold_j2, {msg," j2 hold"});
      expect_eq_int(nb_i3, hold_i3, {msg," i3 hold"}); expect_eq_int(nb_j3, hold_j3, {msg," j3 hold"});
      expect_eq_int(nb_i4, hold_i4, {msg," i4 hold"}); expect_eq_int(nb_j4, hold_j4, {msg," j4 hold"});
      expect_eq_int(nb_i5, hold_i5, {msg," i5 hold"}); expect_eq_int(nb_j5, hold_j5, {msg," j5 hold"});
      expect_eq_int(nb_i6, hold_i6, {msg," i6 hold"}); expect_eq_int(nb_j6, hold_j6, {msg," j6 hold"});
      expect_eq_int(nb_i7, hold_i7, {msg," i7 hold"}); expect_eq_int(nb_j7, hold_j7, {msg," j7 hold"});
      expect_eq_int(nb_i8, hold_i8, {msg," i8 hold"}); expect_eq_int(nb_j8, hold_j8, {msg," j8 hold"});
    end
  endtask

  // Drive one center request and wait until accepted
  task send_center(input integer i, input integer j, input [TAG_W-1:0] tag);
    begin
      @(negedge clk);
      center_i     = i[IW-1:0];
      center_j     = j[JW-1:0];
      center_tag   = tag;
      center_valid = 1'b1;

      while (!(center_valid && center_ready)) begin
        @(negedge clk);
      end

      @(negedge clk);
      center_valid = 1'b0;
    end
  endtask

  // Wait for one nb bundle and check expected mapping
  task recv_and_check_bundle(input integer ci, input integer cj, input [TAG_W-1:0] tag);
    integer ei, ej;
    begin
      // wait until nb_valid
      while (!nb_valid) @(posedge clk);

      // tag check + onehot check
      expect_eq_hex(nb_tag, tag, "nb_tag mismatch");
      expect_onehot_center();

      // compute expected and compare
      // idx 0 (-1,-1)
      ei = clamp_int(ci-1, 0, TILES_Y-1); ej = clamp_int(cj-1, 0, TILES_X-1);
      expect_eq_int(nb_i0, ei, "idx0 i"); expect_eq_int(nb_j0, ej, "idx0 j");

      // idx 1 (-1,0)
      ei = clamp_int(ci-1, 0, TILES_Y-1); ej = clamp_int(cj,   0, TILES_X-1);
      expect_eq_int(nb_i1, ei, "idx1 i"); expect_eq_int(nb_j1, ej, "idx1 j");

      // idx 2 (-1,+1)
      ei = clamp_int(ci-1, 0, TILES_Y-1); ej = clamp_int(cj+1, 0, TILES_X-1);
      expect_eq_int(nb_i2, ei, "idx2 i"); expect_eq_int(nb_j2, ej, "idx2 j");

      // idx 3 (0,-1)
      ei = clamp_int(ci,   0, TILES_Y-1); ej = clamp_int(cj-1, 0, TILES_X-1);
      expect_eq_int(nb_i3, ei, "idx3 i"); expect_eq_int(nb_j3, ej, "idx3 j");

      // idx 4 (0,0)
      ei = clamp_int(ci,   0, TILES_Y-1); ej = clamp_int(cj,   0, TILES_X-1);
      expect_eq_int(nb_i4, ei, "idx4 i"); expect_eq_int(nb_j4, ej, "idx4 j");

      // idx 5 (0,+1)
      ei = clamp_int(ci,   0, TILES_Y-1); ej = clamp_int(cj+1, 0, TILES_X-1);
      expect_eq_int(nb_i5, ei, "idx5 i"); expect_eq_int(nb_j5, ej, "idx5 j");

      // idx 6 (+1,-1)
      ei = clamp_int(ci+1, 0, TILES_Y-1); ej = clamp_int(cj-1, 0, TILES_X-1);
      expect_eq_int(nb_i6, ei, "idx6 i"); expect_eq_int(nb_j6, ej, "idx6 j");

      // idx 7 (+1,0)
      ei = clamp_int(ci+1, 0, TILES_Y-1); ej = clamp_int(cj,   0, TILES_X-1);
      expect_eq_int(nb_i7, ei, "idx7 i"); expect_eq_int(nb_j7, ej, "idx7 j");

      // idx 8 (+1,+1)
      ei = clamp_int(ci+1, 0, TILES_Y-1); ej = clamp_int(cj+1, 0, TILES_X-1);
      expect_eq_int(nb_i8, ei, "idx8 i"); expect_eq_int(nb_j8, ej, "idx8 j");

      // consume bundle
      @(negedge clk);
      nb_ready = 1'b1;
      @(negedge clk);
      nb_ready = 1'b0;
    end
  endtask

  // ----------------------------
  // Main test
  // ----------------------------
  initial begin
    // init
    en = 1'b0;
    rst = 1'b1;

    center_valid = 1'b0;
    center_i = '0;
    center_j = '0;
    center_tag = '0;

    nb_ready = 1'b0;

    repeat (5) @(posedge clk);
    en = 1'b1;
    rst = 1'b0;

    $display("=== TB start: tile_coord_clamp_3x3 ===");

    // 1) Basic interior check (ci=2,cj=3)
    send_center(2, 3, 16'h1234);
    recv_and_check_bundle(2, 3, 16'h1234);
    $display("PASS: interior mapping");

    // 2) Clamp at top-left corner (0,0)
    send_center(0, 0, 16'h00A1);
    recv_and_check_bundle(0, 0, 16'h00A1);
    $display("PASS: clamp top-left");

    // 3) Clamp at bottom-right corner (TILES_Y-1, TILES_X-1)
    send_center(TILES_Y-1, TILES_X-1, 16'hBEEF);
    recv_and_check_bundle(TILES_Y-1, TILES_X-1, 16'hBEEF);
    $display("PASS: clamp bottom-right");

    // 4) Backpressure hold test:
    //   - send a request
    //   - wait for nb_valid, then deassert nb_ready for several cycles
    //   - verify outputs hold stable
    send_center(1, 1, 16'h7777);

    while (!nb_valid) @(posedge clk);
    snap_outputs();
    expect_onehot_center();

    nb_ready = 1'b0;
    repeat (5) begin
      @(posedge clk);
      expect_outputs_held("backpressure hold");
      if (!nb_valid) begin
        $display("[%0t] FAIL: nb_valid dropped under backpressure", $time);
        $fatal(1);
      end
    end

    // now release
    @(negedge clk);
    nb_ready = 1'b1;
    @(negedge clk);
    nb_ready = 1'b0;

    $display("PASS: backpressure hold");

    // 5) Overwrite-on-pop test:
    //   - stall output (nb_ready=0) so full=1 blocks center_ready
    //   - then in the cycle we pop (nb_ready=1), also present a new center_valid
    //     and confirm it is accepted same cycle (center_ready=1)
    // Step A: fill with one
    send_center(2, 6, 16'h1111);
    while (!nb_valid) @(posedge clk);

    // Step B: hold output (keep nb_ready=0), try to send another center -> should stall
    nb_ready = 1'b0;

    @(negedge clk);
    center_i = 0;
    center_j = 7;
    center_tag = 16'h2222;
    center_valid = 1'b1;

    // wait a couple cycles and ensure not accepted while full & nb_ready=0
    repeat (3) begin
      @(negedge clk);
      if (center_ready) begin
        $display("[%0t] FAIL: center_ready asserted while full and nb_ready=0", $time);
        $fatal(1);
      end
    end

    // Step C: pop + accept new in same cycle
    @(negedge clk);
    nb_ready = 1'b1;   // will pop at next posedge if nb_valid=1
    // keep center_valid=1 this cycle -> should be accepted because out_fire will be true
    @(posedge clk);
    if (!(center_valid && center_ready)) begin
      $display("[%0t] FAIL: overwrite-on-pop did not accept new center", $time);
      $fatal(1);
    end

    @(negedge clk);
    center_valid = 1'b0;
    nb_ready = 1'b0;

    // Now we should see the *new* tag bundle next (after pop)
    // First, consume old one (already popped), so wait for nb_valid again for new content
    while (!nb_valid) @(posedge clk);
    expect_eq_hex(nb_tag, 16'h2222, "overwrite tag mismatch");
    $display("PASS: overwrite-on-pop");

    // consume last bundle
    @(negedge clk); nb_ready = 1'b1;
    @(negedge clk); nb_ready = 1'b0;

    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
