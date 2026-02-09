// ============================================================
// tb_nb_feat_bundle_assembler_9_v4.sv  (IVERILOG-SAFE, 修正 pulse miss)
// - 關鍵修正：所有 bundle 都採用「先 out_ready=0 鎖住」
//   避免 out_valid 拉高的同一拍就 out_fire，被 wait_out_valid() 吃掉。
// - 流程：
//   1) out_ready=0
//   2) 等 out_valid=1
//   3) check_bundle / optional hold
//   4) out_ready=1
//   5) 等 out_fire_p pulse
// ============================================================

`include "./src/AMOLED/tile_neighborhood_fetch/nb_feat_bundle_assembler_9.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_nb_feat_bundle_assembler_9_v4;

  localparam int unsigned FEAT_W = 64;
  localparam int unsigned TAG_W  = 16;
  localparam int unsigned IDX_W  = 4;
  localparam bit Q_FROM_CENTER_IDX4 = 1'b1;

  localparam int unsigned TIMEOUT_CYC = 60000;

  // clock/reset
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  // DUT I/O
  reg                   feat_valid;
  wire                  feat_ready;
  reg  [FEAT_W-1:0]      feat_vec;
  reg  [TAG_W-1:0]       feat_tag;

  wire                  out_valid;
  reg                   out_ready;

  wire [FEAT_W-1:0]      kv0,kv1,kv2,kv3,kv4,kv5,kv6,kv7,kv8;
  wire [9*FEAT_W-1:0]    kv_bus;
  wire [FEAT_W-1:0]      q_vec;
  wire [TAG_W-1:IDX_W]   out_group_tag;

  nb_feat_bundle_assembler_9 #(
    .FEAT_W(FEAT_W),
    .TAG_W (TAG_W),
    .IDX_W (IDX_W),
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en (en),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),
    .feat_vec  (feat_vec),
    .feat_tag  (feat_tag),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .kv0(kv0), .kv1(kv1), .kv2(kv2), .kv3(kv3), .kv4(kv4),
    .kv5(kv5), .kv6(kv6), .kv7(kv7), .kv8(kv8),
    .kv_bus(kv_bus),

    .q_vec(q_vec),
    .out_group_tag(out_group_tag)
  );

  // VCD
  initial begin
    $dumpfile("tb_nb_feat_bundle_assembler_9_v4.vcd");
    $dumpvars(0, tb_nb_feat_bundle_assembler_9_v4);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  task fatal_msg(input [240*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  feat_v=%0d feat_r=%0d out_v=%0d out_r=%0d group=%h tag=%h vec=%h",
               feat_valid, feat_ready, out_valid, out_ready, out_group_tag, feat_tag, feat_vec);
      $fatal(1);
    end
  endtask

  task expect_eq_feat(input [FEAT_W-1:0] got, input [FEAT_W-1:0] exp, input [240*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_grp(input [TAG_W-1:IDX_W] got, input [TAG_W-1:IDX_W] exp, input [240*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  function automatic [FEAT_W-1:0] mk_vec(input [TAG_W-1:IDX_W] grp, input int token);
    reg [63:0] x;
    begin
      x = 64'({grp, token[IDX_W-1:0]}) ^ 64'hA5A5_0000_0000_5A5A;
      mk_vec = x[FEAT_W-1:0];
    end
  endfunction

  function automatic [TAG_W-1:0] mk_tag(input [TAG_W-1:IDX_W] grp, input int token);
    begin
      mk_tag = {grp, token[IDX_W-1:0]};
    end
  endfunction

  // ----------------------------
  // Protocol tracking (scoreboard)
  // ----------------------------
  wire feat_fire_p = (en && !rst) && feat_valid && feat_ready;
  wire out_fire_p  = (en && !rst) && out_valid && out_ready;

  // R1/R2 guard
  reg     guard_active;
  integer accept_since_pop;
  reg     pop_seen;

  always @(posedge clk) begin
    if (rst || !en) begin
      guard_active     <= 1'b0;
      accept_since_pop <= 0;
      pop_seen         <= 1'b0;
    end else begin
      if (pop_seen) begin
        if (out_valid !== 1'b0)
          fatal_msg("R1 FAIL: out_valid not cleared next posedge after consume");
      end
      pop_seen <= out_fire_p;

      if (out_fire_p) begin
        guard_active     <= 1'b1;
        accept_since_pop <= 0;
      end else if (guard_active) begin
        if (feat_fire_p) accept_since_pop <= accept_since_pop + 1;

        if (accept_since_pop < 9) begin
          if (out_valid !== 1'b0)
            fatal_msg("R2 FAIL: out_valid reasserted before 9 new accepts");
        end else begin
          guard_active <= 1'b0;
        end
      end
    end
  end

  // ----------------------------
  // Drive tasks
  // ----------------------------
  task drive_idle(input int n);
    integer k;
    begin
      for (k=0;k<n;k=k+1) begin
        @(negedge clk);
        feat_valid = 1'b0;
        @(posedge clk);
      end
    end
  endtask

  task send_token_wait_ready(input [TAG_W-1:IDX_W] grp, input int token, input bit bubble_before);
    integer k;
    begin
      if (bubble_before) drive_idle(1);

      @(negedge clk);
      feat_vec   = mk_vec(grp, token);
      feat_tag   = mk_tag(grp, token);
      feat_valid = 1'b1;

      k = 0;
      while (1) begin
        @(posedge clk);
        if (feat_ready) begin
          @(negedge clk);
          feat_valid = 1'b0;
          disable send_token_wait_ready;
        end
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg("timeout waiting feat_ready");
      end
    end
  endtask

  task wait_out_valid(input [240*8-1:0] why);
    integer k;
    begin
      k = 0;
      while (out_valid !== 1'b1) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg({why, " timeout waiting out_valid"});
      end
    end
  endtask

  task wait_out_fire_pulse(input [240*8-1:0] why);
    integer k;
    begin
      k = 0;
      while (out_fire_p !== 1'b1) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg({why, " timeout waiting out_fire pulse"});
      end
    end
  endtask

  task check_bundle(input [TAG_W-1:IDX_W] grp, input [240*8-1:0] why);
    begin
      expect_eq_grp(out_group_tag, grp, {why, " group_tag"});
      expect_eq_feat(kv0, mk_vec(grp,0), {why, " kv0"});
      expect_eq_feat(kv1, mk_vec(grp,1), {why, " kv1"});
      expect_eq_feat(kv2, mk_vec(grp,2), {why, " kv2"});
      expect_eq_feat(kv3, mk_vec(grp,3), {why, " kv3"});
      expect_eq_feat(kv4, mk_vec(grp,4), {why, " kv4"});
      expect_eq_feat(kv5, mk_vec(grp,5), {why, " kv5"});
      expect_eq_feat(kv6, mk_vec(grp,6), {why, " kv6"});
      expect_eq_feat(kv7, mk_vec(grp,7), {why, " kv7"});
      expect_eq_feat(kv8, mk_vec(grp,8), {why, " kv8"});
      if (Q_FROM_CENTER_IDX4) expect_eq_feat(q_vec, mk_vec(grp,4), {why, " q_vec"});
    end
  endtask

  task check_hold_stable(input int cycles, input [240*8-1:0] why);
    reg [FEAT_W-1:0] h0,h1,h2,h3,h4,h5,h6,h7,h8,hq;
    reg [TAG_W-1:IDX_W] hgrp;
    integer k;
    begin
      if (out_valid !== 1'b1) fatal_msg({why, " out_valid not 1 at hold start"});
      if (out_ready !== 1'b0) fatal_msg({why, " out_ready not 0 at hold start"});

      h0 = kv0; h1 = kv1; h2 = kv2; h3 = kv3; h4 = kv4;
      h5 = kv5; h6 = kv6; h7 = kv7; h8 = kv8;
      hq = q_vec;
      hgrp = out_group_tag;

      for (k=0;k<cycles;k=k+1) begin
        @(posedge clk);
        if (out_valid !== 1'b1) fatal_msg({why, " out_valid dropped during hold"});
        if (kv0 !== h0 || kv1 !== h1 || kv2 !== h2 || kv3 !== h3 || kv4 !== h4 ||
            kv5 !== h5 || kv6 !== h6 || kv7 !== h7 || kv8 !== h8) fatal_msg({why, " kv changed during hold"});
        if (q_vec !== hq) fatal_msg({why, " q_vec changed during hold"});
        if (out_group_tag !== hgrp) fatal_msg({why, " group_tag changed during hold"});
        if (feat_ready !== 1'b0) fatal_msg({why, " feat_ready should be 0 during hold"});
      end
    end
  endtask

  // 一個「標準 bundle consume」流程：先鎖、再檢查、再放行 consume
  task consume_bundle_locked(
    input [TAG_W-1:IDX_W] grp,
    input int hold_cycles,
    input [240*8-1:0] why
  );
    begin
      // lock to avoid immediate consume
      @(negedge clk) out_ready = 1'b0;

      wait_out_valid({why, "_wait_v"});
      check_bundle(grp, {why, "_check"});

      if (hold_cycles > 0)
        check_hold_stable(hold_cycles, {why, "_hold"});

      // release + wait pulse
      @(negedge clk) out_ready = 1'b1;
      wait_out_fire_pulse({why, "_fire"});
    end
  endtask

  // ----------------------------
  // Random helpers
  // ----------------------------
  integer seed;
  function automatic int urand(input int mod);
    int r;
    begin
      r = $random(seed);
      if (r < 0) r = -r;
      urand = (mod == 0) ? 0 : (r % mod);
    end
  endfunction

  task shuffle9(output integer a0,a1,a2,a3,a4,a5,a6,a7,a8);
    integer arr[0:8];
    integer i,j,tmp;
    begin
      for (i=0;i<9;i=i+1) arr[i]=i;
      for (i=8;i>0;i=i-1) begin
        j = urand(i+1);
        tmp = arr[i]; arr[i]=arr[j]; arr[j]=tmp;
      end
      a0=arr[0]; a1=arr[1]; a2=arr[2]; a3=arr[3]; a4=arr[4];
      a5=arr[5]; a6=arr[6]; a7=arr[7]; a8=arr[8];
    end
  endtask

  // ----------------------------
  // Main
  // ----------------------------
  integer t, riter;
  integer o0,o1,o2,o3,o4,o5,o6,o7,o8;
  reg [TAG_W-1:IDX_W] grpA, grpB;

  initial begin
    seed = 32'h1234_5678;

    en = 1'b0;
    rst = 1'b1;

    feat_valid = 1'b0;
    feat_vec   = '0;
    feat_tag   = '0;

    out_ready  = 1'b1;

    repeat (6) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    $display("=== TB start: nb_feat_bundle_assembler_9 (v4 lock-then-consume) ===");

    // Test1: in-order
    $display("[%0t] Test1: in-order", $time);
    for (t=0;t<9;t=t+1) send_token_wait_ready(12'h123, t, 1'b0);
    consume_bundle_locked(12'h123, 0, "Test1");
    $display("PASS: Test1");

    // Test2: out-of-order
    $display("[%0t] Test2: out-of-order", $time);
    send_token_wait_ready(12'h2AA, 4, 0);
    send_token_wait_ready(12'h2AA, 0, 0);
    send_token_wait_ready(12'h2AA, 8, 0);
    send_token_wait_ready(12'h2AA, 1, 0);
    send_token_wait_ready(12'h2AA, 7, 0);
    send_token_wait_ready(12'h2AA, 3, 0);
    send_token_wait_ready(12'h2AA, 6, 0);
    send_token_wait_ready(12'h2AA, 2, 0);
    send_token_wait_ready(12'h2AA, 5, 0);
    consume_bundle_locked(12'h2AA, 0, "Test2");
    $display("PASS: Test2");

    // Test3: backpressure hold
    $display("[%0t] Test3: backpressure hold", $time);
    for (t=0;t<9;t=t+1) send_token_wait_ready(12'h0F0, t, 1'b0);
    consume_bundle_locked(12'h0F0, 12, "Test3");
    $display("PASS: Test3");

    // Test4: upstream bubbles
    $display("[%0t] Test4: upstream bubbles", $time);
    for (t=0;t<9;t=t+1) send_token_wait_ready(12'hABC, t, (t%2));
    consume_bundle_locked(12'hABC, 0, "Test4");
    $display("PASS: Test4");

    // Test5: group switch early -> flush/restart
    $display("[%0t] Test5: group switch early (flush)", $time);
    send_token_wait_ready(12'h111, 0, 0);
    send_token_wait_ready(12'h111, 1, 0);
    send_token_wait_ready(12'h111, 2, 0);
    send_token_wait_ready(12'h222, 0, 0);
    for (t=1;t<9;t=t+1) send_token_wait_ready(12'h222, t, 0);
    consume_bundle_locked(12'h222, 0, "Test5");
    $display("PASS: Test5");

    // Test6: back-to-back bundles + mixed backpressure
    $display("[%0t] Test6: back-to-back bundles", $time);
    grpA = 12'h31A;
    grpB = 12'h31B;

    for (t=0;t<9;t=t+1) send_token_wait_ready(grpA, t, (t==2)||(t==5));
    consume_bundle_locked(grpA, 6, "Test6_A");

    for (t=0;t<9;t=t+1) send_token_wait_ready(grpB, t, (t==1)||(t==7));
    consume_bundle_locked(grpB, 0, "Test6_B");

    $display("PASS: Test6");

    // Test7: random stress
    $display("[%0t] Test7: random stress (200 iters)", $time);
    for (riter=0; riter<200; riter=riter+1) begin
      grpA = urand(4096);

      // token 順序亂
      shuffle9(o0,o1,o2,o3,o4,o5,o6,o7,o8);

      // 在 token 階段可亂 out_ready（不影響收 token，只影響 out_valid 產生後是否立即被吃）
      // 但在 consume_bundle_locked 內會強制先 out_ready=0 鎖住
      @(negedge clk) out_ready = (urand(3)!=0);

      send_token_wait_ready(grpA, o0, (urand(4)==0));
      send_token_wait_ready(grpA, o1, (urand(4)==0));
      send_token_wait_ready(grpA, o2, (urand(4)==0));
      send_token_wait_ready(grpA, o3, (urand(4)==0));
      send_token_wait_ready(grpA, o4, (urand(4)==0));
      send_token_wait_ready(grpA, o5, (urand(4)==0));
      send_token_wait_ready(grpA, o6, (urand(4)==0));
      send_token_wait_ready(grpA, o7, (urand(4)==0));
      send_token_wait_ready(grpA, o8, (urand(4)==0));

      consume_bundle_locked(grpA, (urand(2)? (urand(8)+1) : 0), "Test7");
      @(negedge clk) out_ready = 1'b1;
    end
    $display("PASS: Test7");

    $display("=== TB PASS (v4) ===");
    $finish;
  end

endmodule

`default_nettype wire
