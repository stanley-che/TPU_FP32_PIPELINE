// ============================================================
// tb_tile_nb_fetch_3x3_to_9req_inst.sv  (IVERILOG-SAFE)
// - Tests integrated module: tile_nb_fetch_3x3_to_9req_inst
//   (tile_coord_clamp_3x3 + nb_fetch_scheduler_9req by INSTANCE)
//
// Checks:
//   1) For each center(i,j,tag), issues exactly 9 rd requests
//   2) rd_tile_i/j follow 3x3 row-major order with CLAMP at borders
//   3) rd_tag packing: {center_tag[TAG_W-1:IDX_W], token_idx}
//   4) Backpressure: rd_* held stable while rd_valid=1 && rd_ready=0
//   5) Back-to-back centers: second center can be accepted later; stream continues correctly
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_tile_nb_fetch_3x3_to_9req_inst.vvp ./test/tb_tile_nb_fetch_3x3_to_9req_inst.sv
// Run:
//   vvp ./vvp/tb_tile_nb_fetch_3x3_to_9req_inst.vvp
// ============================================================

`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_fetch_3x3_to_9req_inst.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_tile_nb_fetch_3x3_to_9req_inst;

  // ----------------------------
  // Params (small TB)
  // ----------------------------
  localparam int unsigned TILES_X = 8;
  localparam int unsigned TILES_Y = 4;
  localparam int unsigned TAG_W   = 16;
  localparam int unsigned IDX_W   = 4;

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

  wire                 rd_valid;
  reg                  rd_ready;
  wire [IW-1:0]         rd_tile_i;
  wire [JW-1:0]         rd_tile_j;
  wire [TAG_W-1:0]      rd_tag;

  wire                 sched_busy;
  wire                 sched_done_pulse;
  wire [8:0]            nb_is_center;

  tile_nb_fetch_3x3_to_9req_inst #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W  (TAG_W),
    .IDX_W  (IDX_W)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i(center_i),
    .center_j(center_j),
    .center_tag(center_tag),

    .rd_valid(rd_valid),
    .rd_ready(rd_ready),
    .rd_tile_i(rd_tile_i),
    .rd_tile_j(rd_tile_j),
    .rd_tag(rd_tag),

    .sched_busy(sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .nb_is_center(nb_is_center)
  );

  // ----------------------------
  // Dump
  // ----------------------------
  initial begin
    $dumpfile("tb_tile_nb_fetch_3x3_to_9req_inst.vcd");
    $dumpvars(0, tb_tile_nb_fetch_3x3_to_9req_inst);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  task fatal_msg(input [220*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $fatal(1);
    end
  endtask

  task expect_eq_int(input integer got, input integer exp, input [220*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%0d exp=%0d", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_hex(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [220*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  function integer clamp_int(input integer v, input integer lo, input integer hi);
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  // expected rd_tag pack: {center_tag[TAG_W-1:IDX_W], token_idx}
  function [TAG_W-1:0] pack_tag;
    input [TAG_W-1:0] base;
    input integer token;
    begin
      if (TAG_W > IDX_W) pack_tag = { base[TAG_W-1:IDX_W], token[IDX_W-1:0] };
      else               pack_tag = token[TAG_W-1:0];
    end
  endfunction

  // hold check for rd_* stability under backpressure
  reg [IW-1:0]    hold_i;
  reg [JW-1:0]    hold_j;
  reg [TAG_W-1:0] hold_tag;

  task snap_rd;
    begin
      hold_i   = rd_tile_i;
      hold_j   = rd_tile_j;
      hold_tag = rd_tag;
    end
  endtask

  task expect_rd_held(input [220*8-1:0] msg);
    begin
      if (rd_tile_i !== hold_i) fatal_msg({msg, " rd_tile_i changed while stalled"});
      if (rd_tile_j !== hold_j) fatal_msg({msg, " rd_tile_j changed while stalled"});
      if (rd_tag    !== hold_tag) fatal_msg({msg, " rd_tag changed while stalled"});
    end
  endtask

  // ----------------------------
  // Drive one center and wait handshake
  // ----------------------------
  task send_center(input integer i, input integer j, input [TAG_W-1:0] tag_in);
    begin
      @(negedge clk);
      center_i     = i[IW-1:0];
      center_j     = j[JW-1:0];
      center_tag   = tag_in;
      center_valid = 1'b1;

      while (!(center_valid && center_ready)) @(negedge clk);

      @(negedge clk);
      center_valid = 1'b0;
    end
  endtask

  // ----------------------------
  // Consume and check 9 rd req for a given center
  // 3x3 row-major:
  //   0(-1,-1) 1(-1,0) 2(-1,+1)
  //   3(0,-1)  4(0,0)  5(0,+1)
  //   6(+1,-1) 7(+1,0) 8(+1,+1)
  // with clamp to [0..TILES_Y-1], [0..TILES_X-1]
  // ----------------------------
  integer seen_cnt;
  integer rr;
  integer stall_cycles;
  reg     done_seen;

  task consume_9req_for_center(input integer ci, input integer cj, input [TAG_W-1:0] tag_in);
    integer ei, ej;
    integer token;
    begin
      seen_cnt = 0;
      stall_cycles = 0;
      done_seen = 1'b0;

      while (seen_cnt < 9) begin
        @(posedge clk);

        if (sched_done_pulse) done_seen = 1'b1;

        // random rd_ready (more often ready)
        rr = $random;
        rd_ready <= ((rr % 10) < 7) ? 1'b1 : 1'b0;

        // hold check when stalled
        if (rd_valid && !rd_ready) begin
          if (stall_cycles == 0) snap_rd();
          else expect_rd_held("rd hold");
          stall_cycles = stall_cycles + 1;
        end else begin
          stall_cycles = 0;
        end

        if (rd_valid && rd_ready) begin
          token = seen_cnt;

          // compute expected for this token
          case (token)
            0: begin ei = clamp_int(ci-1,0,TILES_Y-1); ej = clamp_int(cj-1,0,TILES_X-1); end
            1: begin ei = clamp_int(ci-1,0,TILES_Y-1); ej = clamp_int(cj  ,0,TILES_X-1); end
            2: begin ei = clamp_int(ci-1,0,TILES_Y-1); ej = clamp_int(cj+1,0,TILES_X-1); end
            3: begin ei = clamp_int(ci  ,0,TILES_Y-1); ej = clamp_int(cj-1,0,TILES_X-1); end
            4: begin ei = clamp_int(ci  ,0,TILES_Y-1); ej = clamp_int(cj  ,0,TILES_X-1); end
            5: begin ei = clamp_int(ci  ,0,TILES_Y-1); ej = clamp_int(cj+1,0,TILES_X-1); end
            6: begin ei = clamp_int(ci+1,0,TILES_Y-1); ej = clamp_int(cj-1,0,TILES_X-1); end
            7: begin ei = clamp_int(ci+1,0,TILES_Y-1); ej = clamp_int(cj  ,0,TILES_X-1); end
            default: begin ei = clamp_int(ci+1,0,TILES_Y-1); ej = clamp_int(cj+1,0,TILES_X-1); end
          endcase

          expect_eq_int(rd_tile_i, ei, "rd_tile_i mismatch");
          expect_eq_int(rd_tile_j, ej, "rd_tile_j mismatch");
          expect_eq_hex(rd_tag, pack_tag(tag_in, token), "rd_tag mismatch");

          $display("[%0t] RD_FIRE tok=%0d i=%0d j=%0d tag=%h",
                   $time, token, rd_tile_i, rd_tile_j, rd_tag);

          seen_cnt = seen_cnt + 1;
        end
      end

      // ensure done pulse observed (allow a few cycles)
      rr = 0;
      while (!done_seen && rr < 6) begin
        @(posedge clk);
        if (sched_done_pulse) done_seen = 1'b1;
        rr = rr + 1;
      end
      if (!done_seen) fatal_msg("sched_done_pulse not observed after 9 req");

      $display("PASS: 9req for center(%0d,%0d) tag=%h", ci, cj, tag_in);
    end
  endtask

  // ----------------------------
  // Wait busy drop helper
  // ----------------------------
  task wait_busy_drop(input integer max_cycles, input [220*8-1:0] msg);
    integer k;
    begin
      k = 0;
      while (sched_busy !== 1'b0) begin
        @(posedge clk);
        k = k + 1;
        if (k >= max_cycles) begin
          $display("[%0t] TIMEOUT: %s sched_busy=%b", $time, msg, sched_busy);
          $fatal(1);
        end
      end
    end
  endtask

  // ----------------------------
  // Main
  // ----------------------------
  integer k;
  initial begin
    // init
    en = 1'b0;
    rst = 1'b1;

    center_valid = 1'b0;
    center_i     = '0;
    center_j     = '0;
    center_tag   = '0;

    rd_ready = 1'b0;

    repeat (5) @(posedge clk);
    en = 1'b1;
    rst = 1'b0;

    // sanity: nb_is_center fixed
    if (nb_is_center !== 9'b000010000) fatal_msg("nb_is_center not fixed onehot");

    $display("=== TB start: tile_nb_fetch_3x3_to_9req_inst ===");

    // --------------------------------
    // Test 1: interior point
    // --------------------------------
    send_center(2, 3, 16'h1230);
    if (!sched_busy) begin
      // allow a couple cycles to become busy
      repeat (2) @(posedge clk);
    end
    consume_9req_for_center(2, 3, 16'h1230);
    rd_ready <= 1'b0;
    wait_busy_drop(50, "after test1");

    // --------------------------------
    // Test 2: top-left corner clamp
    // --------------------------------
    send_center(0, 0, 16'h00A0);
    consume_9req_for_center(0, 0, 16'h00A0);
    rd_ready <= 1'b0;
    wait_busy_drop(50, "after test2");

    // --------------------------------
    // Test 3: bottom-right corner clamp
    // --------------------------------
    send_center(TILES_Y-1, TILES_X-1, 16'hBEE0);
    consume_9req_for_center(TILES_Y-1, TILES_X-1, 16'hBEE0);
    rd_ready <= 1'b0;
    wait_busy_drop(50, "after test3");

    // --------------------------------
    // Test 4: back-to-back centers (apply 2nd while 1st still issuing)
    // - We keep 2nd center_valid high until accepted (like a real upstream)
    // --------------------------------
    // Send A normally
    send_center(1, 6, 16'hA1C0);

    // While A is being consumed, try to inject B by holding center_valid until ready
    // (This will only be accepted when clamp stage can accept)
    fork
      begin : CONSUME_A
        consume_9req_for_center(1, 6, 16'hA1C0);
      end
      begin : INJECT_B
        integer bi, bj;
        reg [TAG_W-1:0] btag;
        bi = 0; bj = 7; btag = 16'hB2D0;

        // wait a little then start asserting center_valid for B
        repeat (3) @(posedge clk);

        @(negedge clk);
        center_i     = bi[IW-1:0];
        center_j     = bj[JW-1:0];
        center_tag   = btag;
        center_valid = 1'b1;

        // hold until accepted
        while (!(center_valid && center_ready)) @(negedge clk);

        @(negedge clk);
        center_valid = 1'b0;
      end
    join

    // Now consume B (it will be the next stream)
    // Wait until scheduler busy again (or rd_valid appears)
    k = 0;
    while (!rd_valid && k < 50) begin
      @(posedge clk);
      k = k + 1;
    end
    if (!rd_valid) fatal_msg("back-to-back: rd_valid never asserted for B");

    consume_9req_for_center(0, 7, 16'hB2D0);
    rd_ready <= 1'b0;
    wait_busy_drop(80, "after test4");

    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
