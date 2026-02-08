// ============================================================
// tb_feature_sram_burst_reader.sv  (IVERILOG-SAFE)
// - Tests feature_sram_burst_reader with a simple SRAM read model
//
// Checks:
//  1) Basic burst: pack beats into feat_vec, tag matches
//  2) mem_rd_ready stall mid-burst: address sequencing still correct
//  3) mem_rvalid bubbles (delay beats, do not drop): still packs correctly
//  4) feat_ready backpressure: feat_* holds stable until ready
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_feature_sram_burst_reader.vvp \
//     ./test/tb_feature_sram_burst_reader.sv
// Run:
//   vvp ./vvp/tb_feature_sram_burst_reader.vvp
// ============================================================

`timescale 1ns/1ps
`default_nettype none

`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_burst_reader.sv"

module tb_feature_sram_burst_reader;

  // ----------------------------
  // Parameters
  // ----------------------------
  localparam int unsigned FEAT_W       = 256;
  localparam int unsigned MEM_W        = 64;
  localparam int unsigned TAG_W        = 16;
  localparam int unsigned ADDR_W       = 32;
  localparam int unsigned READ_LATENCY = 1;
  localparam bit          USE_RTAG     = 0;

  localparam int unsigned TIMEOUT_CYC  = 5000;
  localparam int unsigned BEATS        = (FEAT_W + MEM_W - 1) / MEM_W;

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  // ----------------------------
  // DUT signals
  // ----------------------------
  reg                  cmd_valid;
  wire                 cmd_ready;
  reg  [ADDR_W-1:0]     cmd_addr;
  reg  [15:0]           cmd_beats;
  reg  [TAG_W-1:0]      cmd_tag;

  wire                 mem_rd_valid;
  reg                  mem_rd_ready;
  wire [ADDR_W-1:0]     mem_addr;
  wire [TAG_W-1:0]      mem_tag;

  reg                  mem_rvalid;
  reg  [MEM_W-1:0]      mem_rdata;
  reg  [TAG_W-1:0]      mem_rtag;

  wire                 feat_valid;
  reg                  feat_ready;
  wire [FEAT_W-1:0]     feat_vec;
  wire [TAG_W-1:0]      feat_tag;

  feature_sram_burst_reader #(
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .TAG_W(TAG_W),
    .ADDR_W(ADDR_W),
    .READ_LATENCY(READ_LATENCY),
    .USE_RTAG(USE_RTAG)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .cmd_valid(cmd_valid),
    .cmd_ready(cmd_ready),
    .cmd_addr(cmd_addr),
    .cmd_beats(cmd_beats),
    .cmd_tag(cmd_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr(mem_addr),
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
    $dumpfile("tb_feature_sram_burst_reader.vcd");
    $dumpvars(0, tb_feature_sram_burst_reader);
  end

  // ----------------------------
  // Helpers
  // ----------------------------
  task fatal_msg(input [200*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  cmd_v=%0d cmd_r=%0d mem_rd_v=%0d mem_rd_r=%0d mem_addr=%0d mem_rv=%0d feat_v=%0d feat_r=%0d",
               cmd_valid, cmd_ready, mem_rd_valid, mem_rd_ready, mem_addr, mem_rvalid, feat_valid, feat_ready);
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

  // IVERILOG-safe cmd sender (drive on negedge, wait ready on posedge)
  task send_cmd_safe(input [ADDR_W-1:0] a, input [15:0] b, input [TAG_W-1:0] t);
    integer k;
    reg done;
    begin
      @(negedge clk);
      cmd_addr  = a;
      cmd_beats = b;
      cmd_tag   = t;
      cmd_valid = 1'b1;

      done = 1'b0;
      k    = 0;
      while (!done) begin
        @(posedge clk);
        if (cmd_ready) done = 1'b1;
        else begin
          k = k + 1;
          if (k >= TIMEOUT_CYC) fatal_msg("timeout waiting cmd_ready");
        end
      end

      @(negedge clk);
      cmd_valid = 1'b0;
    end
  endtask

  // ----------------------------
  // SRAM model (guaranteed return; bubble = delay, NEVER drop)
  // ----------------------------
  reg bubble_en;

  reg [ADDR_W-1:0] addr_pipe [0:READ_LATENCY];
  reg              v_pipe    [0:READ_LATENCY];
  integer i;

  function [MEM_W-1:0] data_fn(input [ADDR_W-1:0] a);
    reg [63:0] x;
    begin
      x = 64'(a) ^ 64'hA5A5_0000_0000_5A5A;
      data_fn = x[MEM_W-1:0];
    end
  endfunction

  localparam int QDEP = 32;
  reg [ADDR_W-1:0] q_addr [0:QDEP-1];
  integer q_wptr, q_rptr, q_count;

  task q_push(input [ADDR_W-1:0] a);
    begin
      if (q_count >= QDEP) fatal_msg("SRAM model FIFO overflow");
      q_addr[q_wptr] = a;
      q_wptr = (q_wptr + 1) % QDEP;
      q_count = q_count + 1;
    end
  endtask

  task q_pop(output [ADDR_W-1:0] a);
    begin
      if (q_count <= 0) fatal_msg("SRAM model FIFO underflow");
      a = q_addr[q_rptr];
      q_rptr = (q_rptr + 1) % QDEP;
      q_count = q_count - 1;
    end
  endtask

  reg [ADDR_W-1:0] pop_addr;

  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i <= READ_LATENCY; i = i + 1) begin
        addr_pipe[i] <= '0;
        v_pipe[i]    <= 1'b0;
      end

      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      mem_rtag   <= '0;

      q_wptr  <= 0;
      q_rptr  <= 0;
      q_count <= 0;

    end else if (en) begin
      // shift latency
      for (i = READ_LATENCY; i > 0; i = i - 1) begin
        addr_pipe[i] <= addr_pipe[i-1];
        v_pipe[i]    <= v_pipe[i-1];
      end
      addr_pipe[0] <= mem_addr;
      v_pipe[0]    <= (mem_rd_valid && mem_rd_ready);

      // enqueue reads that completed latency
      if (v_pipe[READ_LATENCY]) begin
        q_push(addr_pipe[READ_LATENCY]);
      end

      // default no return
      mem_rvalid <= 1'b0;

      // return bandwidth = 1 beat/cycle when not bubbling
      if (q_count > 0) begin
        if (bubble_en && (($random % 3) == 0)) begin
          mem_rvalid <= 1'b0; // delay only
        end else begin
          q_pop(pop_addr);
          mem_rvalid <= 1'b1;
          mem_rdata  <= data_fn(pop_addr);
          mem_rtag   <= mem_tag;
        end
      end
    end
  end

  // ----------------------------
  // Expected vector builder
  // ----------------------------
  reg [FEAT_W-1:0] exp_feat;
  reg [TAG_W-1:0]  exp_tag;

  task build_expected(input [ADDR_W-1:0] a0, input integer beats);
    integer b;
    begin
      exp_feat = '0;
      for (b = 0; b < beats; b = b + 1) begin
        exp_feat[b*MEM_W +: MEM_W] = data_fn(a0 + b);
      end
    end
  endtask

  // ----------------------------
  // Address checker (stall-safe): base + beat_idx on each mem_fire
  // ----------------------------
  integer beat_idx;
  reg in_burst;
  reg [ADDR_W-1:0] burst_base;

  always @(posedge clk) begin
    if (rst) begin
      beat_idx   <= 0;
      in_burst   <= 1'b0;
      burst_base <= '0;
    end else if (en) begin
      if (cmd_valid && cmd_ready) begin
        in_burst   <= 1'b1;
        burst_base <= cmd_addr;
        beat_idx   <= 0;
      end

      if (mem_rd_valid && mem_rd_ready) begin
        if (!in_burst) fatal_msg("mem_fire seen but not in_burst");
        if (mem_addr !== (burst_base + beat_idx)) begin
          $display("[%0t] addr check fail: base=%0d idx=%0d got=%0d exp=%0d",
                   $time, burst_base, beat_idx, mem_addr, (burst_base + beat_idx));
          fatal_msg("mem_addr not matching base+beat_idx");
        end
        beat_idx <= beat_idx + 1;
      end

      if (feat_valid && feat_ready) begin
        in_burst <= 1'b0;
      end
    end
  end

  // ----------------------------
  // Main tests
  // ----------------------------
  reg [FEAT_W-1:0] hold_feat;
  reg [TAG_W-1:0]  hold_tag;

  initial begin
    en = 1'b0;
    rst = 1'b1;

    cmd_valid = 1'b0;
    cmd_addr  = '0;
    cmd_beats = '0;
    cmd_tag   = '0;

    mem_rd_ready = 1'b1;
    bubble_en    = 1'b0;

    feat_ready   = 1'b1;

    repeat (5) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    $display("=== TB start: feature_sram_burst_reader ===");

    // ----------------------------
    // Test1: basic burst
    // ----------------------------
    $display("[%0t] Test1: basic burst", $time);
    exp_tag = 16'hA100;
    build_expected(32'd1000, BEATS);

    @(negedge clk);
    feat_ready = 1'b0;

    send_cmd_safe(32'd1000, 16'(BEATS), exp_tag);

    wait_feat_valid();
    expect_eq_feat(feat_vec, exp_feat, "basic: feat_vec mismatch");
    expect_eq_tag(feat_tag, exp_tag, "basic: feat_tag mismatch");

    @(negedge clk);
    feat_ready = 1'b1;
    wait_feat_fire("basic");
    @(negedge clk);
    $display("PASS: Test1");

    // ----------------------------
    // Test2: mem_rd_ready stall mid-burst (RACE-SAFE)
    // ----------------------------
    $display("[%0t] Test2: mem_rd_ready stall", $time);
    exp_tag = 16'hBEEF;
    build_expected(32'd2000, BEATS);

    @(negedge clk);
    feat_ready = 1'b0;

    send_cmd_safe(32'd2000, 16'(BEATS), exp_tag);

    // wait first mem_fire
    begin : wait_first_fire
      integer k;
      k = 0;
      while (!(mem_rd_valid && mem_rd_ready)) begin
        @(posedge clk);
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg("Test2: timeout waiting first mem_fire");
      end
    end

    // stall ready (change on negedge to avoid races)
    @(negedge clk);
    mem_rd_ready = 1'b0;
    repeat (5) @(posedge clk);
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
    // Test3: mem_rvalid bubbles (DELAY ONLY, NEVER DROP)
    // ----------------------------
    $display("[%0t] Test3: mem_rvalid bubbles", $time);

    @(negedge clk);
    bubble_en  = 1'b1;
    feat_ready = 1'b0;

    exp_tag = 16'hCAFE;
    build_expected(32'd3000, BEATS);

    send_cmd_safe(32'd3000, 16'(BEATS), exp_tag);

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

    exp_tag = 16'h1234;
    build_expected(32'd4000, BEATS);

    @(negedge clk);
    feat_ready = 1'b0;

    send_cmd_safe(32'd4000, 16'(BEATS), exp_tag);

    wait_feat_valid();
    hold_feat = feat_vec;
    hold_tag  = feat_tag;

    repeat (8) begin
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
