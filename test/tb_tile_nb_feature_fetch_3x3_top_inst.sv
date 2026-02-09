// ============================================================
// tb_tile_nb_feature_fetch_3x3_top_inst.sv  (IVERILOG-SAFE, FULL)
// - Testbench for: tile_nb_feature_fetch_3x3_top_inst
//
// End-to-end flow:
//   center(i,j,tag) -> 9 neighbor tile reqs -> SRAM burst reads -> feat_vec out (9 vectors)
//
// Checks:
//   1) Exactly 9 feature outputs per center, in row-major 3x3 order (with clamp)
//   2) feat_tag == {center_tag[TAG_W-1:IDX_W], token_idx}  (token_idx=0..8)
//   3) feat_vec matches data_fn(full_word_addr) packed by beats
//   4) mem_rd_ready stall mid-burst does NOT break address sequence / packing
//   5) mem_rvalid bubbles (delay only, never drop) still packs correctly
//   6) feat_ready backpressure: feat_* holds stable while feat_valid=1 && feat_ready=0
//   7) sched_done_pulse is a 1-cycle pulse; TB captures it using sticky done_seen
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_tile_nb_feature_fetch_3x3_top_inst.vvp ./test/tb_tile_nb_feature_fetch_3x3_top_inst.sv
// Run:
//   vvp ./vvp/tb_tile_nb_feature_fetch_3x3_top_inst.vvp
// ============================================================

`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_feature_fetch_3x3_top_inst.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_tile_nb_feature_fetch_3x3_top_inst;

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

  localparam int unsigned BANKS          = 4;   // try 1 or 4 (power-of-2)
  localparam int unsigned ADDR_W         = 32;
  localparam int unsigned READ_LATENCY   = 1;
  localparam bit          USE_RTAG       = 0;

  localparam int unsigned IW     = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW     = (TILES_X <= 1) ? 1 : $clog2(TILES_X);
  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  localparam int unsigned BEATS = (FEAT_W + MEM_W - 1) / MEM_W;

  localparam int unsigned TIMEOUT_CYC = 20000;

  // ----------------------------
  // Clock/reset
  // ----------------------------
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

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

  wire                 feat_valid;
  reg                  feat_ready;
  wire [FEAT_W-1:0]     feat_vec;
  wire [TAG_W-1:0]      feat_tag;

  wire                 sched_busy;
  wire                 sched_done_pulse;
  wire [8:0]            nb_is_center;

  tile_nb_feature_fetch_3x3_top_inst #(
    .TILES_X       (TILES_X),
    .TILES_Y       (TILES_Y),
    .TAG_W         (TAG_W),
    .IDX_W         (IDX_W),
    .FEAT_W        (FEAT_W),
    .MEM_W         (MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS         (BANKS),
    .ADDR_W        (ADDR_W),
    .READ_LATENCY  (READ_LATENCY),
    .USE_RTAG      (USE_RTAG)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i    (center_i),
    .center_j    (center_j),
    .center_tag  (center_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr    (mem_addr),
    .mem_bank    (mem_bank),
    .mem_tag     (mem_tag),

    .mem_rvalid(mem_rvalid),
    .mem_rdata (mem_rdata),
    .mem_rtag  (mem_rtag),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),
    .feat_vec  (feat_vec),
    .feat_tag  (feat_tag),

    .sched_busy      (sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .nb_is_center    (nb_is_center)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("tb_tile_nb_feature_fetch_3x3_top_inst.vcd");
    $dumpvars(0, tb_tile_nb_feature_fetch_3x3_top_inst);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  task fatal_msg(input [240*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  center_v=%0d center_r=%0d mem_rd_v=%0d mem_rd_r=%0d mem_bank=%0d mem_addr=%0d mem_rv=%0d feat_v=%0d feat_r=%0d busy=%0d done=%0d",
               center_valid, center_ready, mem_rd_valid, mem_rd_ready, mem_bank, mem_addr, mem_rvalid, feat_valid, feat_ready, sched_busy, sched_done_pulse);
      $fatal(1);
    end
  endtask

  task expect_eq_hex(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [240*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_feat(input [FEAT_W-1:0] got, input [FEAT_W-1:0] exp, input [240*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s", $time, msg);
        $display("  got=%h", got);
        $display("  exp=%h", exp);
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

  function [TAG_W-1:0] pack_tag(input [TAG_W-1:0] base, input integer token);
    begin
      if (TAG_W > IDX_W) pack_tag = { base[TAG_W-1:IDX_W], token[IDX_W-1:0] };
      else               pack_tag = token[TAG_W-1:0];
    end
  endfunction

  // ----------------------------
  // Address/data model for SRAM
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
  // Send center request (safe)
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
  // Wait feat_valid / feat_fire
  // ----------------------------
  task wait_feat_valid;
    integer k;
    begin
      k = 0;
      while (feat_valid !== 1'b1) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg("timeout waiting feat_valid");
      end
    end
  endtask

  task wait_feat_fire(input [240*8-1:0] why);
    integer k;
    begin
      k = 0;
      while (!(feat_valid && feat_ready)) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg({why, " timeout waiting feat_fire"});
      end
    end
  endtask

  // ----------------------------
  // Done pulse capture (sticky)
  // - sched_done_pulse is 1-cycle; TB latches it into done_seen
  // ----------------------------
  reg done_seen;

  always @(posedge clk) begin
    if (rst) begin
      done_seen <= 1'b0;
    end else if (en) begin
      if (sched_done_pulse) done_seen <= 1'b1;
    end
  end

  task clear_done_seen;
    begin
      @(negedge clk);
      done_seen = 1'b0;
    end
  endtask

  task wait_done_seen(input [240*8-1:0] why);
    integer k;
    begin
      k = 0;
      while (done_seen !== 1'b1) begin
        @(posedge clk);
        k = k + 1;
        if (k >= 4000) fatal_msg({why, " timeout waiting done_seen"});
      end
    end
  endtask

  // ----------------------------
  // SRAM model: enqueue reads on mem_fire, return with latency + bubbles
  // ----------------------------
  reg bubble_en;

  reg [63:0] addr_pipe [0:READ_LATENCY];
  reg        v_pipe    [0:READ_LATENCY];
  integer    ii;

  localparam int QDEP = 256;
  reg [63:0] q_full [0:QDEP-1];
  integer q_wptr, q_rptr, q_count;

  task q_push(input [63:0] fw);
    begin
      if (q_count >= QDEP) fatal_msg("SRAM model FIFO overflow");
      q_full[q_wptr] = fw;
      q_wptr = (q_wptr + 1) % QDEP;
      q_count = q_count + 1;
    end
  endtask

  task q_pop(output [63:0] fw);
    begin
      if (q_count <= 0) fatal_msg("SRAM model FIFO underflow");
      fw = q_full[q_rptr];
      q_rptr = (q_rptr + 1) % QDEP;
      q_count = q_count - 1;
    end
  endtask

  reg [63:0] pop_fw;

  wire go = en && !rst;
  wire mem_fire = go && mem_rd_valid && mem_rd_ready;

  always @(posedge clk) begin
    if (rst) begin
      for (ii = 0; ii <= READ_LATENCY; ii = ii + 1) begin
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
      // shift latency pipe
      for (ii = READ_LATENCY; ii > 0; ii = ii - 1) begin
        addr_pipe[ii] <= addr_pipe[ii-1];
        v_pipe[ii]    <= v_pipe[ii-1];
      end

      addr_pipe[0] <= full_word_from_bank_addr(mem_bank, mem_addr);
      v_pipe[0]    <= mem_fire;

      // enqueue after latency
      if (v_pipe[READ_LATENCY]) begin
        q_push(addr_pipe[READ_LATENCY]);
      end

      // default no return
      mem_rvalid <= 1'b0;

      // return 1 beat/cycle unless bubble
      if (q_count > 0) begin
        if (bubble_en && (($random % 3) == 0)) begin
          mem_rvalid <= 1'b0; // delay only, never drop
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
  // Consume 1 feature output and check against expected tile/token
  // (hold feat_ready low while checking, then pulse high to consume)
  // ----------------------------
  task consume_and_check_one(
    input integer ci,
    input integer cj,
    input [TAG_W-1:0] ctag,
    input integer token
  );
    integer ti, tj;
    reg [63:0] base_full;
    reg [FEAT_W-1:0] exp_feat;
    reg [TAG_W-1:0]  exp_t;
    begin
      // expected neighbor tile (row-major 3x3 with clamp)
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

      exp_t     = pack_tag(ctag, token);
      base_full = base_full_word_for_tile(ti, tj);
      build_expected_feat(base_full, exp_feat);

      @(negedge clk);
      feat_ready = 1'b0;

      wait_feat_valid();

      expect_eq_hex (feat_tag, exp_t,   "feat_tag mismatch");
      expect_eq_feat(feat_vec, exp_feat,"feat_vec mismatch");

      @(negedge clk);
      feat_ready = 1'b1;
      wait_feat_fire("consume_one");
      @(negedge clk);
      feat_ready = 1'b0;

      $display("[%0t] FEAT_OK tok=%0d tile=(%0d,%0d) tag=%h", $time, token, ti, tj, exp_t);
    end
  endtask

  // ----------------------------
  // Backpressure hold check for one output
  // ----------------------------
  task check_feat_hold_stable(input integer hold_cycles);
    reg [FEAT_W-1:0] hv;
    reg [TAG_W-1:0]  ht;
    integer k;
    begin
      wait_feat_valid();
      hv = feat_vec;
      ht = feat_tag;

      for (k = 0; k < hold_cycles; k = k + 1) begin
        @(posedge clk);
        if (feat_valid !== 1'b1) fatal_msg("feat_valid dropped during backpressure");
        if (feat_vec   !== hv)   fatal_msg("feat_vec changed during backpressure");
        if (feat_tag   !== ht)   fatal_msg("feat_tag changed during backpressure");
      end
    end
  endtask

  // ----------------------------
  // Main
  // ----------------------------
  integer tok;
  initial begin
    // init
    en = 1'b0;
    rst = 1'b1;

    center_valid = 1'b0;
    center_i     = '0;
    center_j     = '0;
    center_tag   = '0;

    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    feat_ready   = 1'b0;
    done_seen    = 1'b0;

    repeat (6) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    // sanity: center index should be at bit[4]
    if (nb_is_center !== 9'b000010000) fatal_msg("nb_is_center not 000010000");

    $display("=== TB start: tile_nb_feature_fetch_3x3_top_inst ===");

    // ============================================================
    // Test1: basic (no stalls, no bubbles)
    // ============================================================
    $display("[%0t] Test1: basic", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    clear_done_seen();
    send_center(2, 3, 16'h1230);

    for (tok = 0; tok < 9; tok = tok + 1) begin
      consume_and_check_one(2, 3, 16'h1230, tok);
    end

    wait_done_seen("Test1");
    $display("PASS: Test1");

    // ============================================================
    // Test2: mem_rd_ready stall mid-burst
    // ============================================================
    $display("[%0t] Test2: mem_rd_ready stall mid-burst", $time);
    bubble_en = 1'b0;

    clear_done_seen();
    send_center(0, 0, 16'h00A0);

    fork
      begin : STALL_RDREADY
        integer kk;
        kk = 0;
        while (!mem_fire) begin
          @(posedge clk);
          kk = kk + 1;
          if (kk >= TIMEOUT_CYC) fatal_msg("Test2: timeout waiting first mem_fire");
        end

        @(negedge clk);
        mem_rd_ready = 1'b0;
        repeat (12) @(posedge clk);
        @(negedge clk);
        mem_rd_ready = 1'b1;
      end
    join_none

    for (tok = 0; tok < 9; tok = tok + 1) begin
      consume_and_check_one(0, 0, 16'h00A0, tok);
    end

    wait_done_seen("Test2");
    mem_rd_ready = 1'b1;
    $display("PASS: Test2");

    // ============================================================
    // Test3: mem_rvalid bubbles (delay only)
    // ============================================================
    $display("[%0t] Test3: mem_rvalid bubbles", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b1;

    clear_done_seen();
    send_center(TILES_Y-1, TILES_X-1, 16'hBEE0);

    for (tok = 0; tok < 9; tok = tok + 1) begin
      consume_and_check_one(TILES_Y-1, TILES_X-1, 16'hBEE0, tok);
    end

    wait_done_seen("Test3");
    bubble_en = 1'b0;
    $display("PASS: Test3");

    // ============================================================
    // Test4: feat_ready backpressure hold stable
    // ============================================================
    $display("[%0t] Test4: feat_ready backpressure hold", $time);
    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    @(negedge clk);
    feat_ready = 1'b0;

    clear_done_seen();
    send_center(1, 6, 16'hA1C0);

    check_feat_hold_stable(12);

    consume_and_check_one(1, 6, 16'hA1C0, 0);
    for (tok = 1; tok < 9; tok = tok + 1) begin
      consume_and_check_one(1, 6, 16'hA1C0, tok);
    end

    wait_done_seen("Test4");
    $display("PASS: Test4");

    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
