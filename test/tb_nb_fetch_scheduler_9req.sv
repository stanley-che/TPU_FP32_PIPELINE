// ============================================================
// tb_nb_fetch_scheduler_9req.sv  (IVERILOG-SAFE)
// - Tests nb_fetch_scheduler_9req
// Checks:
//   1) Bundle handshake (nb_valid/nb_ready) + sched_busy behavior
//   2) Issues exactly 9 read requests per bundle, token_idx=0..8
//   3) rd_tile_i/j sequence matches bundle coords
//   4) rd_tag packing: rd_tag = {nb_tag[TAG_W-1:IDX_W], token_idx}
//   5) Backpressure on rd_ready: holds rd_* stable while stalled
//   6) Back-to-back bundles: accept next bundle on finishing_last
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_nb_fetch_scheduler_9req.vvp ./test/tb_nb_fetch_scheduler_9req.sv
// Run:
//   vvp ./vvp/tb_nb_fetch_scheduler_9req.vvp
// ============================================================
`include "./src/AMOLED/tile_neighborhood_fetch/nb_fetch_scheduler_9req.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_nb_fetch_scheduler_9req;

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
  reg                 nb_valid;
  wire                nb_ready;

  reg  [IW-1:0] nb_i0; reg [JW-1:0] nb_j0;
  reg  [IW-1:0] nb_i1; reg [JW-1:0] nb_j1;
  reg  [IW-1:0] nb_i2; reg [JW-1:0] nb_j2;
  reg  [IW-1:0] nb_i3; reg [JW-1:0] nb_j3;
  reg  [IW-1:0] nb_i4; reg [JW-1:0] nb_j4;
  reg  [IW-1:0] nb_i5; reg [JW-1:0] nb_j5;
  reg  [IW-1:0] nb_i6; reg [JW-1:0] nb_j6;
  reg  [IW-1:0] nb_i7; reg [JW-1:0] nb_j7;
  reg  [IW-1:0] nb_i8; reg [JW-1:0] nb_j8;

  reg  [TAG_W-1:0] nb_tag;

  wire                rd_valid;
  reg                 rd_ready;
  wire [IW-1:0]        rd_tile_i;
  wire [JW-1:0]        rd_tile_j;
  wire [TAG_W-1:0]     rd_tag;

  wire                sched_busy;
  wire                sched_done_pulse;

  nb_fetch_scheduler_9req #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W(TAG_W),
    .IDX_W(IDX_W)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

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

    .rd_valid(rd_valid),
    .rd_ready(rd_ready),
    .rd_tile_i(rd_tile_i),
    .rd_tile_j(rd_tile_j),
    .rd_tag(rd_tag),

    .sched_busy(sched_busy),
    .sched_done_pulse(sched_done_pulse)
  );

  // ----------------------------
  // Dump
  // ----------------------------
  initial begin
    $dumpfile("tb_nb_fetch_scheduler_9req.vcd");
    $dumpvars(0, tb_nb_fetch_scheduler_9req);
  end

  // ----------------------------
  // Scoreboard / helpers
  // ----------------------------
  task fatal_msg(input [200*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $fatal(1);
    end
  endtask

  task expect_eq_int(input integer got, input integer exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%0d exp=%0d", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_hex(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  // hold check variables for rd_* stability under backpressure
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

  task expect_rd_held(input [200*8-1:0] msg);
    begin
      if (rd_tile_i !== hold_i) fatal_msg({msg, " rd_tile_i changed while stalled"});
      if (rd_tile_j !== hold_j) fatal_msg({msg, " rd_tile_j changed while stalled"});
      if (rd_tag    !== hold_tag) fatal_msg({msg, " rd_tag changed while stalled"});
    end
  endtask

  // expected arrays for CURRENT bundle being checked
  integer exp_i [0:8];
  integer exp_j [0:8];

  // expected arrays for NEXT bundle (used in back-to-back injection)
  integer exp_i_B [0:8];
  integer exp_j_B [0:8];

  // expected tag pack: {nb_tag[TAG_W-1:IDX_W], token_idx}
  function [TAG_W-1:0] pack_tag;
    input [TAG_W-1:0] base;
    input integer token;
    begin
      if (TAG_W > IDX_W) pack_tag = { base[TAG_W-1:IDX_W], token[IDX_W-1:0] };
      else               pack_tag = token[TAG_W-1:0];
    end
  endfunction

  // ----------------------------
  // Drive a bundle and wait handshake
  // ----------------------------
  task send_bundle(input [TAG_W-1:0] tag_in);
    begin
      @(negedge clk);
      nb_tag   = tag_in;
      nb_valid = 1'b1;

      while (!(nb_valid && nb_ready)) @(negedge clk);

      @(negedge clk);
      nb_valid = 1'b0;
    end
  endtask

  // ----------------------------
  // Calculate 9 coords
  // ----------------------------
  task calc_bundle_exp(
    input  integer base_i, input integer base_j,
    output integer oi0, output integer oj0,
    output integer oi1, output integer oj1,
    output integer oi2, output integer oj2,
    output integer oi3, output integer oj3,
    output integer oi4, output integer oj4,
    output integer oi5, output integer oj5,
    output integer oi6, output integer oj6,
    output integer oi7, output integer oj7,
    output integer oi8, output integer oj8
  );
    begin
      oi0 = (base_i+0) % TILES_Y; oj0 = (base_j+0) % TILES_X;
      oi1 = (base_i+0) % TILES_Y; oj1 = (base_j+1) % TILES_X;
      oi2 = (base_i+0) % TILES_Y; oj2 = (base_j+2) % TILES_X;
      oi3 = (base_i+1) % TILES_Y; oj3 = (base_j+0) % TILES_X;
      oi4 = (base_i+1) % TILES_Y; oj4 = (base_j+1) % TILES_X;
      oi5 = (base_i+1) % TILES_Y; oj5 = (base_j+2) % TILES_X;
      oi6 = (base_i+2) % TILES_Y; oj6 = (base_j+0) % TILES_X;
      oi7 = (base_i+2) % TILES_Y; oj7 = (base_j+1) % TILES_X;
      oi8 = (base_i+2) % TILES_Y; oj8 = (base_j+2) % TILES_X;
    end
  endtask

  // ----------------------------
  // Load bundle into exp_i/exp_j AND drive nb_i*/nb_j*
  // ----------------------------
  task load_bundle_values(input integer base_i, input integer base_j);
    integer t0,t1,t2,t3,t4,t5,t6,t7,t8;
    integer u0,u1,u2,u3,u4,u5,u6,u7,u8;
    begin
      calc_bundle_exp(base_i, base_j,
        t0,u0, t1,u1, t2,u2, t3,u3, t4,u4, t5,u5, t6,u6, t7,u7, t8,u8
      );

      exp_i[0]=t0; exp_j[0]=u0;
      exp_i[1]=t1; exp_j[1]=u1;
      exp_i[2]=t2; exp_j[2]=u2;
      exp_i[3]=t3; exp_j[3]=u3;
      exp_i[4]=t4; exp_j[4]=u4;
      exp_i[5]=t5; exp_j[5]=u5;
      exp_i[6]=t6; exp_j[6]=u6;
      exp_i[7]=t7; exp_j[7]=u7;
      exp_i[8]=t8; exp_j[8]=u8;

      nb_i0 = exp_i[0][IW-1:0]; nb_j0 = exp_j[0][JW-1:0];
      nb_i1 = exp_i[1][IW-1:0]; nb_j1 = exp_j[1][JW-1:0];
      nb_i2 = exp_i[2][IW-1:0]; nb_j2 = exp_j[2][JW-1:0];
      nb_i3 = exp_i[3][IW-1:0]; nb_j3 = exp_j[3][JW-1:0];
      nb_i4 = exp_i[4][IW-1:0]; nb_j4 = exp_j[4][JW-1:0];
      nb_i5 = exp_i[5][IW-1:0]; nb_j5 = exp_j[5][JW-1:0];
      nb_i6 = exp_i[6][IW-1:0]; nb_j6 = exp_j[6][JW-1:0];
      nb_i7 = exp_i[7][IW-1:0]; nb_j7 = exp_j[7][JW-1:0];
      nb_i8 = exp_i[8][IW-1:0]; nb_j8 = exp_j[8][JW-1:0];
    end
  endtask

  // ----------------------------
  // Prepare NEXT bundle into exp_i_B/exp_j_B AND drive nb_i*/nb_j*,
  // but DO NOT touch exp_i/exp_j
  // ----------------------------
  task prep_next_bundle_only(input integer base_i, input integer base_j);
    integer t0,t1,t2,t3,t4,t5,t6,t7,t8;
    integer u0,u1,u2,u3,u4,u5,u6,u7,u8;
    begin
      calc_bundle_exp(base_i, base_j,
        t0,u0, t1,u1, t2,u2, t3,u3, t4,u4, t5,u5, t6,u6, t7,u7, t8,u8
      );

      exp_i_B[0]=t0; exp_j_B[0]=u0;
      exp_i_B[1]=t1; exp_j_B[1]=u1;
      exp_i_B[2]=t2; exp_j_B[2]=u2;
      exp_i_B[3]=t3; exp_j_B[3]=u3;
      exp_i_B[4]=t4; exp_j_B[4]=u4;
      exp_i_B[5]=t5; exp_j_B[5]=u5;
      exp_i_B[6]=t6; exp_j_B[6]=u6;
      exp_i_B[7]=t7; exp_j_B[7]=u7;
      exp_i_B[8]=t8; exp_j_B[8]=u8;

      nb_i0 = exp_i_B[0][IW-1:0]; nb_j0 = exp_j_B[0][JW-1:0];
      nb_i1 = exp_i_B[1][IW-1:0]; nb_j1 = exp_j_B[1][JW-1:0];
      nb_i2 = exp_i_B[2][IW-1:0]; nb_j2 = exp_j_B[2][JW-1:0];
      nb_i3 = exp_i_B[3][IW-1:0]; nb_j3 = exp_j_B[3][JW-1:0];
      nb_i4 = exp_i_B[4][IW-1:0]; nb_j4 = exp_j_B[4][JW-1:0];
      nb_i5 = exp_i_B[5][IW-1:0]; nb_j5 = exp_j_B[5][JW-1:0];
      nb_i6 = exp_i_B[6][IW-1:0]; nb_j6 = exp_j_B[6][JW-1:0];
      nb_i7 = exp_i_B[7][IW-1:0]; nb_j7 = exp_j_B[7][JW-1:0];
      nb_i8 = exp_i_B[8][IW-1:0]; nb_j8 = exp_j_B[8][JW-1:0];
    end
  endtask

  // ----------------------------
  // Consume 9 requests with random rd_ready stalls (CURRENT exp_i/exp_j)
  // ----------------------------
  integer seen_cnt;
  integer token;
  integer rr;
  integer stall_cycles;
  reg     done_seen;

  task consume_9req_check(input [TAG_W-1:0] base_tag);
    begin
      seen_cnt = 0;
      stall_cycles = 0;
      done_seen = 1'b0;

      while (seen_cnt < 9) begin
        @(posedge clk);

        if (sched_done_pulse) done_seen = 1'b1;

        rr = $random;
        rd_ready <= ((rr % 10) < 6) ? 1'b1 : 1'b0;

        if (rd_valid && !rd_ready) begin
          if (stall_cycles == 0) snap_rd();
          else expect_rd_held("stall hold");
          stall_cycles = stall_cycles + 1;
        end else begin
          stall_cycles = 0;
        end

        if (rd_valid && rd_ready) begin
          token = seen_cnt;

          expect_eq_int(rd_tile_i, exp_i[token], "rd_tile_i mismatch");
          expect_eq_int(rd_tile_j, exp_j[token], "rd_tile_j mismatch");
          expect_eq_hex(rd_tag, pack_tag(base_tag, token), "rd_tag mismatch");

          $display("[%0t] RD_FIRE idx=%0d i=%0d j=%0d tag=%h",
                   $time, token, rd_tile_i, rd_tile_j, rd_tag);

          seen_cnt = seen_cnt + 1;
        end
      end

      rr = 0;
      while (!done_seen && rr < 4) begin
        @(posedge clk);
        if (sched_done_pulse) done_seen = 1'b1;
        rr = rr + 1;
      end

      if (!done_seen) fatal_msg("sched_done_pulse not observed after 9 req");

      $display("PASS: 9req issued + done pulse");
    end
  endtask
task wait_busy_drop(input integer max_cycles, input [200*8-1:0] msg);
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
  integer cnt2;
  reg injected;
  reg b_accepted;

  reg [TAG_W-1:0] tag_A;
  reg [TAG_W-1:0] tag_B;
  integer base_iA, base_jA;
  integer base_iB, base_jB;

  initial begin
    // init
    en = 1'b0;
    rst = 1'b1;

    nb_valid = 1'b0;
    nb_tag   = '0;

    nb_i0='0; nb_j0='0; nb_i1='0; nb_j1='0; nb_i2='0; nb_j2='0;
    nb_i3='0; nb_j3='0; nb_i4='0; nb_j4='0; nb_i5='0; nb_j5='0;
    nb_i6='0; nb_j6='0; nb_i7='0; nb_j7='0; nb_i8='0; nb_j8='0;

    rd_ready = 1'b0;

    repeat (5) @(posedge clk);
    en = 1'b1;
    rst = 1'b0;

    $display("=== TB start: nb_fetch_scheduler_9req ===");

    // ----------------------------
    // Test 1: single bundle
    // ----------------------------
    load_bundle_values(0, 0);
    tag_A = 16'hA5C0;
    send_bundle(tag_A);
    rd_ready <= 1'b0;
    repeat (2) @(posedge clk);
    if (!sched_busy) fatal_msg("sched_busy not asserted after bundle");

    consume_9req_check(tag_A);

    rd_ready <= 1'b0;
    wait_busy_drop(50, "after bundle A");

    $display("PASS: single bundle");

    // ----------------------------
    // Test 2: back-to-back bundles
    // ----------------------------
    base_iA = 1; base_jA = 2; tag_A = 16'h1230;
    base_iB = 2; base_jB = 4; tag_B = 16'h55F0;

    // bundle A expectations + drive A inputs
    load_bundle_values(base_iA, base_jA);
    send_bundle(tag_A);

    injected     = 1'b0;
    b_accepted   = 1'b0;
    seen_cnt     = 0;
    stall_cycles = 0;
    done_seen    = 1'b0;

    // Consume bundle A; prepare bundle B early; keep nb_valid until accepted (once)
    while (seen_cnt < 9) begin
      @(posedge clk);

      if (sched_done_pulse) done_seen = 1'b1;

      rr = $random;
      rd_ready <= ((rr % 10) < 7) ? 1'b1 : 1'b0;

      if (rd_valid && !rd_ready) begin
        if (stall_cycles == 0) snap_rd();
        else expect_rd_held("stall hold (b2b)");
        stall_cycles = stall_cycles + 1;
      end else begin
        stall_cycles = 0;
      end

      if (rd_valid && rd_ready) begin
        token = seen_cnt;

        expect_eq_int(rd_tile_i, exp_i[token], "b2b(A) rd_tile_i mismatch");
        expect_eq_int(rd_tile_j, exp_j[token], "b2b(A) rd_tile_j mismatch");
        expect_eq_hex(rd_tag, pack_tag(tag_A, token), "b2b(A) rd_tag mismatch");

        seen_cnt = seen_cnt + 1;
      end

      // prepare+assert nb_valid for B when we're safely before the end
      if (!injected && (seen_cnt == 7)) begin
        prep_next_bundle_only(base_iB, base_jB);
        @(negedge clk);
        nb_tag   = tag_B;
        nb_valid = 1'b1;
        injected = 1'b1;
      end

      // accept exactly once; drop on next negedge (avoid holding a full extra cycle)
      if (injected && nb_valid && nb_ready && !b_accepted) begin
        b_accepted = 1'b1;
        @(negedge clk);
        nb_valid = 1'b0;
      end
    end

    if (!b_accepted) fatal_msg("b2b: bundle B was never accepted (nb_fire missing)");

    // Ensure done pulse for bundle A observed (allow immediate or few cycles later)
    rr = 0;
    while (!done_seen && rr < 6) begin
      @(posedge clk);
      if (sched_done_pulse) done_seen = 1'b1;
      rr = rr + 1;
    end
    if (!done_seen) fatal_msg("b2b: done pulse not seen for bundle A");

    // Scheduler should already be busy again (bundle B accepted on finishing_last)
    rd_ready <= 1'b0;
    repeat (2) @(posedge clk);
    if (!sched_busy) fatal_msg("b2b: sched not busy for bundle B");

    // Switch expected arrays to bundle B now
    for (cnt2 = 0; cnt2 < 9; cnt2 = cnt2 + 1) begin
        exp_i[cnt2] = exp_i_B[cnt2];
        exp_j[cnt2] = exp_j_B[cnt2];
    end

    // 可選：等到 rd_valid 確認已經是 bundle B 在送
    while (!rd_valid) @(posedge clk);

    // 交給 consume_9req_check 重新 random rd_ready
    consume_9req_check(tag_B);


    rd_ready <= 1'b0;
    wait_busy_drop(50, "after bundle B");

    $display("PASS: back-to-back bundles");
    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
