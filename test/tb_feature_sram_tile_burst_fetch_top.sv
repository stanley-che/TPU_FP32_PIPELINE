// ============================================================
// tb_feature_sram_tile_burst_fetch_top.sv  (IVERILOG-SAFE)
// - Test for feature_sram_tile_burst_fetch_top (instance-integrated)
//
// Checks:
//  1) Basic tile burst: address sequence + packing + tag
//  2) mem_rd_ready stall mid-burst: address sequence still correct
//  3) mem_rvalid bubbles (delay only, never drop): still packs correctly
//  4) feat_ready backpressure: feat_* holds stable until ready
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_feature_sram_tile_burst_fetch_top.vvp ./test/tb_feature_sram_tile_burst_fetch_top.sv
// Run:
//   vvp ./vvp/tb_feature_sram_tile_burst_fetch_top.vvp
// ============================================================
`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_tile_burst_fetch_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_feature_sram_tile_burst_fetch_top;

  // ----------------------------
  // Parameters (small for TB)
  // ----------------------------
  localparam int unsigned TILES_X        = 8;
  localparam int unsigned TILES_Y        = 4;

  localparam int unsigned FEAT_W         = 256;
  localparam int unsigned MEM_W          = 64;
  localparam int unsigned BASE_ADDR_WORD = 100;

  localparam int unsigned BANKS          = 4;   // try 1 or 4
  localparam int unsigned TAG_W          = 16;
  localparam int unsigned ADDR_W         = 32;
  localparam int unsigned READ_LATENCY   = 1;
  localparam bit          USE_RTAG       = 0;

  localparam int unsigned IW = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW = (TILES_X <= 1) ? 1 : $clog2(TILES_X);
  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  localparam int unsigned BEATS = (FEAT_W + MEM_W - 1) / MEM_W;

  localparam int unsigned TIMEOUT_CYC = 5000;

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                  rd_valid;
  wire                 rd_ready_in;
  reg  [IW-1:0]         rd_tile_i;
  reg  [JW-1:0]         rd_tile_j;
  reg  [TAG_W-1:0]      rd_tag;

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

  feature_sram_tile_burst_fetch_top #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS(BANKS),
    .TAG_W(TAG_W),
    .ADDR_W(ADDR_W),
    .READ_LATENCY(READ_LATENCY),
    .USE_RTAG(USE_RTAG)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .rd_valid(rd_valid),
    .rd_ready_in(rd_ready_in),
    .rd_tile_i(rd_tile_i),
    .rd_tile_j(rd_tile_j),
    .rd_tag(rd_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr(mem_addr),
    .mem_bank(mem_bank),
    .mem_tag(mem_tag),

    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),
    .mem_rtag(mem_rtag),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),
    .feat_vec(feat_vec),
    .feat_tag(feat_tag)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("tb_feature_sram_tile_burst_fetch_top.vcd");
    $dumpvars(0, tb_feature_sram_tile_burst_fetch_top);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  task fatal_msg(input [200*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  rd_v=%0d rd_r=%0d mem_rd_v=%0d mem_rd_r=%0d mem_bank=%0d mem_addr=%0d mem_rv=%0d feat_v=%0d feat_r=%0d",
               rd_valid, rd_ready_in, mem_rd_valid, mem_rd_ready, mem_bank, mem_addr, mem_rvalid, feat_valid, feat_ready);
      $fatal(1);
    end
  endtask

  task expect_eq_tag(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task expect_eq_feat(input [FEAT_W-1:0] got, input [FEAT_W-1:0] exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s", $time, msg);
        $display("  got=%h", got);
        $display("  exp=%h", exp);
        $fatal(1);
      end
    end
  endtask

  // Drive a single tile request (drive on negedge, handshake at posedge)
  task send_tile_req(input int ti, input int tj, input [TAG_W-1:0] tagv);
    integer k;
    reg done;
    begin
      @(negedge clk);
      rd_tile_i = IW'(ti);
      rd_tile_j = JW'(tj);
      rd_tag    = tagv;
      rd_valid  = 1'b1;

      done = 1'b0;
      k    = 0;
      while (!done) begin
        @(posedge clk);
        if (rd_ready_in) done = 1'b1;
        else begin
          k = k + 1;
          if (k >= TIMEOUT_CYC) fatal_msg("timeout waiting rd_ready_in");
        end
      end

      @(negedge clk);
      rd_valid = 1'b0;
    end
  endtask

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

  task wait_feat_fire(input [200*8-1:0] why);
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
  // Address / data model
  // ----------------------------
  function [63:0] base_word_full_fn(input int ti, input int tj);
    int linear;
    begin
      linear = ti*TILES_X + tj;
      base_word_full_fn = 64'(BASE_ADDR_WORD) + 64'(linear) * 64'(BEATS);
    end
  endfunction

  function [63:0] full_word_from_bank_addr(input [BANK_W-1:0] b, input [ADDR_W-1:0] a);
    begin
      if (BANKS <= 1) full_word_from_bank_addr = 64'(a);
      else            full_word_from_bank_addr = (64'(a) << BANK_W) | 64'(b);
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
  // SRAM return queue (bubble-safe)
  // - enqueue full_word on mem_fire
  // - pop and return (or bubble delay)
  // ----------------------------
  reg bubble_en;

  localparam int QDEP = 64;
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

  // ----------------------------
  // Address checker: expected full-word sequence on each mem_fire
  // ----------------------------
  integer beat_idx;
  reg in_burst;
  reg [63:0] exp_base_full;

  wire mem_fire = (en && !rst) && mem_rd_valid && mem_rd_ready;

  always @(posedge clk) begin
    if (rst) begin
      beat_idx      <= 0;
      in_burst      <= 1'b0;
      exp_base_full <= 64'd0;
    end else if (en) begin
      // detect burst start = first mem_fire after a tile request accepted
      // (we'll set exp_base_full/beat_idx from TB right after send_tile_req)
      if (mem_fire) begin
        if (!in_burst) begin
          // first beat
          beat_idx <= 0;
          in_burst <= 1'b1;
        end
      end

      if (mem_fire) begin
        // reconstruct full word address from (bank, addr)
        reg [63:0] got_fw;
        got_fw = full_word_from_bank_addr(mem_bank, mem_addr);

        if (got_fw !== (exp_base_full + 64'(beat_idx))) begin
          $display("[%0t] addr check fail: base=%0d idx=%0d got_full=%0d exp_full=%0d",
                   $time, exp_base_full, beat_idx, got_fw, (exp_base_full + 64'(beat_idx)));
          fatal_msg("mem full-word address mismatch");
        end

        // enqueue for return (SRAM accepts read)
        q_push(got_fw);

        beat_idx <= beat_idx + 1;

        if (beat_idx + 1 >= BEATS) begin
          // issued all beats; still may be returning data later
          in_burst <= 1'b0;
        end
      end
    end
  end

  // SRAM return generator
  always @(posedge clk) begin
    if (rst) begin
      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      mem_rtag   <= '0;
      q_wptr     <= 0;
      q_rptr     <= 0;
      q_count    <= 0;
    end else if (en) begin
      mem_rvalid <= 1'b0;

      if (q_count > 0) begin
        if (bubble_en && (($random % 3) == 0)) begin
          mem_rvalid <= 1'b0; // bubble: delay only
        end else begin
          q_pop(pop_fw);
          mem_rvalid <= 1'b1;
          mem_rdata  <= data_fn(pop_fw);
          mem_rtag   <= mem_tag; // not used when USE_RTAG=0
        end
      end
    end
  end

  // ----------------------------
  // Main tests
  // ----------------------------
  reg [FEAT_W-1:0] exp_feat;
  reg [TAG_W-1:0]  exp_tag;

  reg [FEAT_W-1:0] hold_feat;
  reg [TAG_W-1:0]  hold_tag;

  initial begin
    // init
    en = 1'b0;
    rst = 1'b1;

    rd_valid  = 1'b0;
    rd_tile_i = '0;
    rd_tile_j = '0;
    rd_tag    = '0;

    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    feat_ready   = 1'b1;

    repeat (5) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    $display("=== TB start: feature_sram_tile_burst_fetch_top ===");

    // ----------------------------
    // Test1: basic burst
    // ----------------------------
    $display("[%0t] Test1: basic burst", $time);
    exp_tag      = 16'hA100;
    exp_base_full = base_word_full_fn(1, 3);
    build_expected_feat(exp_base_full, exp_feat);

    // IMPORTANT: hold output while checking
    @(negedge clk);
    feat_ready = 1'b0;

    // reset checker beat_idx state for this test
    @(negedge clk);
    beat_idx = 0;
    in_burst = 1'b0;

    send_tile_req(1, 3, exp_tag);

    wait_feat_valid();
    expect_eq_feat(feat_vec, exp_feat, "basic: feat_vec mismatch");
    expect_eq_tag(feat_tag, exp_tag, "basic: feat_tag mismatch");

    @(negedge clk);
    feat_ready = 1'b1;
    wait_feat_fire("basic");
    @(negedge clk);
    $display("PASS: Test1");

    // ----------------------------
    // Test2: mem_rd_ready stall mid-burst
    // ----------------------------
    $display("[%0t] Test2: mem_rd_ready stall", $time);
    exp_tag       = 16'hBEEF;
    exp_base_full = base_word_full_fn(2, 5);
    build_expected_feat(exp_base_full, exp_feat);

    @(negedge clk);
    feat_ready = 1'b0;

    @(negedge clk);
    beat_idx = 0;
    in_burst = 1'b0;

    send_tile_req(2, 5, exp_tag);

    // wait first mem_fire then stall
    begin : wait_first_fire
      integer k;
      k = 0;
      while (!mem_fire) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg("Test2: timeout waiting first mem_fire");
      end
    end

    // stall on negedge to avoid posedge races
    @(negedge clk);
    mem_rd_ready = 1'b0;
    repeat (6) @(posedge clk);
    @(negedge clk);
    mem_rd_ready = 1'b1;

    wait_feat_valid();
    expect_eq_feat(feat_vec, exp_feat, "stall: feat_vec mismatch");
    expect_eq_tag(feat_tag, exp_tag, "stall: feat_tag mismatch");

    @(negedge clk);
    feat_ready = 1'b1;
    wait_feat_fire("stall");
    @(negedge clk);
    $display("PASS: Test2");

    // ----------------------------
    // Test3: mem_rvalid bubbles (delay only)
    // ----------------------------
    $display("[%0t] Test3: mem_rvalid bubbles", $time);
    exp_tag       = 16'hCAFE;
    exp_base_full = base_word_full_fn(0, 7);
    build_expected_feat(exp_base_full, exp_feat);

    @(negedge clk);
    bubble_en  = 1'b1;
    feat_ready = 1'b0;

    @(negedge clk);
    beat_idx = 0;
    in_burst = 1'b0;

    send_tile_req(0, 7, exp_tag);

    wait_feat_valid();
    expect_eq_feat(feat_vec, exp_feat, "bubble: feat_vec mismatch");
    expect_eq_tag(feat_tag, exp_tag, "bubble: feat_tag mismatch");

    @(negedge clk);
    feat_ready = 1'b1;
    wait_feat_fire("bubble");
    @(negedge clk);

    bubble_en = 1'b0;
    $display("PASS: Test3");

    // ----------------------------
    // Test4: output backpressure (hold stable)
    // ----------------------------
    $display("[%0t] Test4: backpressure", $time);
    exp_tag       = 16'h1234;
    exp_base_full = base_word_full_fn(3, 1);
    build_expected_feat(exp_base_full, exp_feat);

    @(negedge clk);
    feat_ready = 1'b0;

    @(negedge clk);
    beat_idx = 0;
    in_burst = 1'b0;

    send_tile_req(3, 1, exp_tag);

    wait_feat_valid();
    expect_eq_feat(feat_vec, exp_feat, "backpressure: feat_vec mismatch");
    expect_eq_tag(feat_tag, exp_tag, "backpressure: feat_tag mismatch");

    hold_feat = feat_vec;
    hold_tag  = feat_tag;

    repeat (10) begin
      @(posedge clk);
      if (feat_valid !== 1'b1) fatal_msg("backpressure: feat_valid dropped");
      if (feat_vec   !== hold_feat) fatal_msg("backpressure: feat_vec changed");
      if (feat_tag   !== hold_tag)  fatal_msg("backpressure: feat_tag changed");
    end

    @(negedge clk);
    feat_ready = 1'b1;
    wait_feat_fire("backpressure");
    @(negedge clk);
    $display("PASS: Test4");

    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
