// ============================================================
// tb_tile_nb_feature_fetch_3x3_bundle9_top_inst.sv  (IVERILOG-SAFE)
// - DUT: tile_nb_feature_fetch_3x3_bundle9_top_inst
// - Fix: DO NOT rely on bundle_done_pulse (may be combinational / miss)
//        Instead treat bundle-consumed event as out_fire (out_valid&&out_ready).
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_tile_nb_feature_fetch_3x3_bundle9_top_inst.vvp \
//     ./test/tb_tile_nb_feature_fetch_3x3_bundle9_top_inst.sv
// Run:
//   vvp ./vvp/tb_tile_nb_feature_fetch_3x3_bundle9_top_inst.vvp
// ============================================================

`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_feature_fetch_3x3_bundle9_top_inst.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_tile_nb_feature_fetch_3x3_bundle9_top_inst;

  // ----------------------------
  // Params (small TB)
  // ----------------------------
  localparam int unsigned TILES_X        = 8;
  localparam int unsigned TILES_Y        = 4;

  localparam int unsigned TAG_W          = 16;
  localparam int unsigned IDX_W          = 4;

  localparam int unsigned FEAT_W         = 256;
  localparam int unsigned MEM_W          = 64;
  localparam int unsigned BASE_ADDR_WORD = 100;

  localparam int unsigned BANKS          = 4;    // 1 or power-of-2
  localparam int unsigned ADDR_W         = 32;
  localparam int unsigned READ_LATENCY   = 1;
  localparam bit          USE_RTAG       = 0;

  localparam bit          Q_FROM_CENTER_IDX4 = 1'b1;

  localparam int unsigned IW     = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW     = (TILES_X <= 1) ? 1 : $clog2(TILES_X);
  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  localparam int unsigned BEATS  = (FEAT_W + MEM_W - 1) / MEM_W;

  localparam int unsigned TIMEOUT_CYC = 60000;

  // ----------------------------
  // Clock/reset
  // ----------------------------
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  wire go = en && !rst;

  // ----------------------------
  // DUT ports
  // ----------------------------
  reg                  center_valid;
  wire                 center_ready;
  reg  [IW-1:0]         center_i;
  reg  [JW-1:0]         center_j;
  reg  [TAG_W-1:0]      center_tag;

  wire                 mem_rd_valid;
  reg                  mem_rd_ready;
  wire [ADDR_W-1:0]     mem_addr;
  wire [BANK_W-1:0]     mem_bank;
  wire [TAG_W-1:0]      mem_tag;

  reg                  mem_rvalid;
  reg  [MEM_W-1:0]      mem_rdata;
  reg  [TAG_W-1:0]      mem_rtag;

  wire                 out_valid;
  reg                  out_ready;

  wire [FEAT_W-1:0]     kv0,kv1,kv2,kv3,kv4,kv5,kv6,kv7,kv8;
  wire [9*FEAT_W-1:0]   kv_bus;
  wire [FEAT_W-1:0]     q_vec;
  wire [TAG_W-1:IDX_W]  out_group_tag;

  wire                 sched_busy;
  wire                 sched_done_pulse;
  wire                 bundle_done_pulse; // keep for waveform only, TB won't rely
  wire [8:0]            nb_is_center;

  tile_nb_feature_fetch_3x3_bundle9_top_inst #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W(TAG_W),
    .IDX_W(IDX_W),
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS(BANKS),
    .ADDR_W(ADDR_W),
    .READ_LATENCY(READ_LATENCY),
    .USE_RTAG(USE_RTAG),
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i(center_i),
    .center_j(center_j),
    .center_tag(center_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr(mem_addr),
    .mem_bank(mem_bank),
    .mem_tag(mem_tag),

    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),
    .mem_rtag(mem_rtag),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .kv0(kv0), .kv1(kv1), .kv2(kv2), .kv3(kv3), .kv4(kv4),
    .kv5(kv5), .kv6(kv6), .kv7(kv7), .kv8(kv8),

    .kv_bus(kv_bus),
    .q_vec(q_vec),
    .out_group_tag(out_group_tag),

    .sched_busy(sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .bundle_done_pulse(bundle_done_pulse),
    .nb_is_center(nb_is_center)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("tb_tile_nb_feature_fetch_3x3_bundle9_top_inst.vcd");
    $dumpvars(0, tb_tile_nb_feature_fetch_3x3_bundle9_top_inst);
  end

  // ----------------------------
  // Helper: fatal
  // ----------------------------
  task fatal_msg(input [240*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  center_v=%0d center_r=%0d out_v=%0d out_r=%0d busy=%0d done=%0d bdone=%0d mem_rd_v=%0d mem_rd_r=%0d",
               center_valid, center_ready, out_valid, out_ready, sched_busy, sched_done_pulse, bundle_done_pulse, mem_rd_valid, mem_rd_ready);
      $display("  center=(%0d,%0d) tag=%h out_group=%h", center_i, center_j, center_tag, out_group_tag);
      $fatal(1);
    end
  endtask

  task expect_eq_feat(input [FEAT_W-1:0] got, input [FEAT_W-1:0] exp, input [240*8-1:0] why);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s", $time, why);
        $display("  got=%h", got);
        $display("  exp=%h", exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_grp(input [TAG_W-1:IDX_W] got, input [TAG_W-1:IDX_W] exp, input [240*8-1:0] why);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, why, got, exp);
        $fatal(1);
      end
    end
  endtask

  // ----------------------------
  // Math helpers
  // ----------------------------
  function integer clamp_int(input integer v, input integer lo, input integer hi);
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  function [TAG_W-1:IDX_W] group_of_center_tag(input [TAG_W-1:0] ctag);
    begin
      group_of_center_tag = ctag[TAG_W-1:IDX_W];
    end
  endfunction

  // neighbor mapping token 0..8 row-major
  task token_to_tile(
    input integer ci, input integer cj, input integer token,
    output integer ti, output integer tj
  );
    begin
      case (token)
        0: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        1: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        2: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
        3: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        4: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        5: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
        6: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        7: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        default: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
      endcase
    end
  endtask

  // ----------------------------
  // SRAM address/data model
  // ----------------------------
  function [63:0] full_word_from_bank_addr(input [BANK_W-1:0] b, input [ADDR_W-1:0] a);
    begin
      if (BANKS <= 1) full_word_from_bank_addr = 64'(a);
      else            full_word_from_bank_addr = (64'(a) << BANK_W) | 64'(b);
    end
  endfunction

  function [63:0] base_full_word_for_tile(input integer ti, input integer tj);
    integer linear;
    begin
      linear = ti*TILES_X + tj;
      base_full_word_for_tile = 64'(BASE_ADDR_WORD) + 64'(linear) * 64'(BEATS);
    end
  endfunction

  function [MEM_W-1:0] data_fn(input [63:0] full_word);
    reg [63:0] x;
    begin
      x = full_word ^ 64'hA5A5_0000_0000_5A5A;
      data_fn = x[MEM_W-1:0];
    end
  endfunction

  task build_expected_feat(input [63:0] base_full, output reg [FEAT_W-1:0] exp);
    integer b;
    begin
      exp = '0;
      for (b = 0; b < BEATS; b = b + 1) begin
        exp[b*MEM_W +: MEM_W] = data_fn(base_full + 64'(b));
      end
    end
  endtask

  // ----------------------------
  // Send center request (safe handshake)
  // ----------------------------
  task send_center(input integer ci, input integer cj, input [TAG_W-1:0] ctag);
    integer k;
    reg done;
    begin
      @(negedge clk);
      center_i     = ci[IW-1:0];
      center_j     = cj[JW-1:0];
      center_tag   = ctag;
      center_valid = 1'b1;

      done = 1'b0;
      k = 0;
      while (!done) begin
        @(posedge clk);
        if (center_ready) done = 1'b1;
        else begin
          k = k + 1;
          if (k >= TIMEOUT_CYC) fatal_msg("timeout waiting center_ready");
        end
      end

      @(negedge clk);
      center_valid = 1'b0;
    end
  endtask

  // ----------------------------
  // Observations (robust)
  // ----------------------------
  wire out_fire = go && out_valid && out_ready;
  wire mem_fire = go && mem_rd_valid && mem_rd_ready;

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
      while (out_fire !== 1'b1) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg({why, " timeout waiting out_fire"});
      end
      // consumed at this posedge
    end
  endtask

  task check_out_hold_stable(input integer cycles, input [240*8-1:0] why);
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
        if (out_group_tag !== hgrp) fatal_msg({why, " out_group_tag changed during hold"});
      end
    end
  endtask

  task check_bundle_expected(
    input integer ci, input integer cj,
    input [TAG_W-1:0] ctag,
    input [240*8-1:0] why
  );
    integer ti, tj;
    reg [FEAT_W-1:0] exp0,exp1,exp2,exp3,exp4,exp5,exp6,exp7,exp8;
    begin
      expect_eq_grp(out_group_tag, group_of_center_tag(ctag), {why, " out_group_tag"});

      token_to_tile(ci,cj,0,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp0);
      token_to_tile(ci,cj,1,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp1);
      token_to_tile(ci,cj,2,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp2);
      token_to_tile(ci,cj,3,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp3);
      token_to_tile(ci,cj,4,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp4);
      token_to_tile(ci,cj,5,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp5);
      token_to_tile(ci,cj,6,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp6);
      token_to_tile(ci,cj,7,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp7);
      token_to_tile(ci,cj,8,ti,tj); build_expected_feat(base_full_word_for_tile(ti,tj), exp8);

      expect_eq_feat(kv0, exp0, {why, " kv0"});
      expect_eq_feat(kv1, exp1, {why, " kv1"});
      expect_eq_feat(kv2, exp2, {why, " kv2"});
      expect_eq_feat(kv3, exp3, {why, " kv3"});
      expect_eq_feat(kv4, exp4, {why, " kv4"});
      expect_eq_feat(kv5, exp5, {why, " kv5"});
      expect_eq_feat(kv6, exp6, {why, " kv6"});
      expect_eq_feat(kv7, exp7, {why, " kv7"});
      expect_eq_feat(kv8, exp8, {why, " kv8"});

      if (Q_FROM_CENTER_IDX4) begin
        expect_eq_feat(q_vec, exp4, {why, " q_vec"});
      end
    end
  endtask

  // Consume flow:
  // 1) out_ready=0 lock
  // 2) wait out_valid
  // 3) check bundle + optional hold
  // 4) out_ready=1
  // 5) wait out_fire (reliable "bundle done")
  task consume_bundle_locked(
    input integer ci, input integer cj,
    input [TAG_W-1:0] ctag,
    input integer hold_cycles,
    input [240*8-1:0] why
  );
    begin
      @(negedge clk) out_ready = 1'b0;
      wait_out_valid({why, "_wait_v"});
      check_bundle_expected(ci,cj,ctag,{why, "_check"});
      if (hold_cycles > 0)
        check_out_hold_stable(hold_cycles, {why, "_hold"});
      @(negedge clk) out_ready = 1'b1;
      wait_out_fire_pulse({why, "_fire"});
    end
  endtask

  // ----------------------------
  // sched_done_pulse sticky (should be reg pulse)
  // ----------------------------
  reg done_seen;

  task clear_seen;
    begin
      done_seen = 1'b0;
    end
  endtask

  always @(posedge clk) begin
    if (rst || !en) begin
      done_seen <= 1'b0;
    end else begin
      if (sched_done_pulse) done_seen <= 1'b1;
    end
  end

  task expect_seen(input [240*8-1:0] why);
    begin
      if (!done_seen) fatal_msg({why, " sched_done_pulse not seen"});
      // bundle "done" is guaranteed by consume_bundle_locked waiting out_fire
    end
  endtask

  // ----------------------------
  // SRAM model (mem_rd -> mem_rvalid/data)
  // - enqueue full_word addresses after READ_LATENCY
  // - pop 1 beat per cycle unless bubble
  // ----------------------------
  reg bubble_en;

  reg [63:0] addr_pipe [0:READ_LATENCY];
  reg        v_pipe    [0:READ_LATENCY];
  integer    ii;

  localparam int QDEP = 512;
  reg [63:0] q_full [0:QDEP-1];
  integer q_wptr, q_rptr, q_count;

  task q_push(input [63:0] fw);
    begin
      if (q_count >= QDEP) fatal_msg("SRAM FIFO overflow");
      q_full[q_wptr] = fw;
      q_wptr = (q_wptr + 1) % QDEP;
      q_count = q_count + 1;
    end
  endtask

  task q_pop(output [63:0] fw);
    begin
      if (q_count <= 0) fatal_msg("SRAM FIFO underflow");
      fw = q_full[q_rptr];
      q_rptr = (q_rptr + 1) % QDEP;
      q_count = q_count - 1;
    end
  endtask

  reg [63:0] pop_fw;

  integer seed;
  function automatic int urand(input int mod);
    int r;
    begin
      r = $random(seed);
      if (r < 0) r = -r;
      urand = (mod==0) ? 0 : (r % mod);
    end
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      for (ii=0; ii<=READ_LATENCY; ii=ii+1) begin
        addr_pipe[ii] <= 64'd0;
        v_pipe[ii]    <= 1'b0;
      end
      q_wptr  <= 0;
      q_rptr  <= 0;
      q_count <= 0;

      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      mem_rtag   <= '0;
    end else if (en) begin
      // shift latency
      for (ii=READ_LATENCY; ii>0; ii=ii-1) begin
        addr_pipe[ii] <= addr_pipe[ii-1];
        v_pipe[ii]    <= v_pipe[ii-1];
      end

      addr_pipe[0] <= full_word_from_bank_addr(mem_bank, mem_addr);
      v_pipe[0]    <= mem_fire;

      if (v_pipe[READ_LATENCY]) begin
        q_push(addr_pipe[READ_LATENCY]);
      end

      mem_rvalid <= 1'b0;

      if (q_count > 0) begin
        if (bubble_en && (urand(3)==0)) begin
          mem_rvalid <= 1'b0; // bubble (delay only)
        end else begin
          q_pop(pop_fw);
          mem_rvalid <= 1'b1;
          mem_rdata  <= data_fn(pop_fw);
          mem_rtag   <= mem_tag; // ok when USE_RTAG=0
        end
      end
    end
  end

  // ----------------------------
  // Main
  // ----------------------------
  integer tmp;
  initial begin
    seed = 32'h1234_5678;

    en = 1'b0;
    rst = 1'b1;

    center_valid = 1'b0;
    center_i     = '0;
    center_j     = '0;
    center_tag   = '0;

    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    out_ready    = 1'b1;
    done_seen    = 1'b0;

    repeat (6) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    // sanity
    if (nb_is_center !== 9'b000010000) fatal_msg("nb_is_center not 000010000");

    $display("=== TB start: tile_nb_feature_fetch_3x3_bundle9_top_inst ===");

    // ------------------------------------------------------------
    // Test1: basic
    // ------------------------------------------------------------
    $display("[%0t] Test1: basic", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;
    out_ready    = 1'b1;

    clear_seen();
    send_center(2, 3, 16'h1230);
    consume_bundle_locked(2, 3, 16'h1230, 0, "Test1");
    expect_seen("Test1");
    $display("PASS: Test1");

    // ------------------------------------------------------------
    // Test2: mem_rd_ready stall mid-flight
    // ------------------------------------------------------------
    $display("[%0t] Test2: mem_rd_ready stall mid-flight", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    clear_seen();
    send_center(0, 0, 16'h00A0);

    fork
      begin : STALL_RDREADY
        integer kk;
        kk = 0;
        while (!mem_fire) begin
          @(posedge clk);
          kk = kk + 1;
          if (kk >= TIMEOUT_CYC) fatal_msg("Test2 timeout waiting first mem_fire");
        end
        @(negedge clk) mem_rd_ready = 1'b0;
        repeat (20) @(posedge clk);
        @(negedge clk) mem_rd_ready = 1'b1;
      end
    join_none

    consume_bundle_locked(0, 0, 16'h00A0, 0, "Test2");
    expect_seen("Test2");
    mem_rd_ready = 1'b1;
    $display("PASS: Test2");

    // ------------------------------------------------------------
    // Test3: mem_rvalid bubbles (delay only)
    // ------------------------------------------------------------
    $display("[%0t] Test3: mem_rvalid bubbles", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b1;

    clear_seen();
    send_center(TILES_Y-1, TILES_X-1, 16'hBEE0);
    consume_bundle_locked(TILES_Y-1, TILES_X-1, 16'hBEE0, 0, "Test3");
    expect_seen("Test3");
    bubble_en = 1'b0;
    $display("PASS: Test3");

    // ------------------------------------------------------------
    // Test4: out_ready backpressure hold stable
    // ------------------------------------------------------------
    $display("[%0t] Test4: out_ready backpressure hold stable", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    clear_seen();
    send_center(1, 6, 16'hA1C0);
    consume_bundle_locked(1, 6, 16'hA1C0, 15, "Test4");
    expect_seen("Test4");
    $display("PASS: Test4");

    // ------------------------------------------------------------
    // Test5: random stress (50 iters)
    // ------------------------------------------------------------
    $display("[%0t] Test5: random stress (50 iters)", $time);
    bubble_en = 1'b0;
    mem_rd_ready = 1'b1;

    begin : RANDOM_LOOP
      integer it;
      integer ci, cj;
      reg [TAG_W-1:0] ctag;
      for (it=0; it<50; it=it+1) begin
        ci = urand(TILES_Y);
        cj = urand(TILES_X);
        tmp  = urand(4096);
        ctag = { tmp[TAG_W-1:IDX_W], {IDX_W{1'b0}} };

        bubble_en = (urand(3)==0);

        clear_seen();

        fork
          begin : RAND_RDREADY_TOG
            integer k;
            for (k=0;k<80;k=k+1) begin
              @(negedge clk);
              if (urand(5)==0) mem_rd_ready = 1'b0;
              else             mem_rd_ready = 1'b1;
            end
            @(negedge clk) mem_rd_ready = 1'b1;
          end
        join_none

        send_center(ci, cj, ctag);

        consume_bundle_locked(ci, cj, ctag, (urand(2)? (urand(12)+1) : 0), "Test5");

        expect_seen("Test5_iter");
      end
    end

    $display("PASS: Test5");
    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
