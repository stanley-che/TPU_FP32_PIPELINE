`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_rd_addr_encode.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_feature_sram_rd_addr_encode;

  localparam int unsigned TILES_X        = 8;
  localparam int unsigned TILES_Y        = 4;
  localparam int unsigned FEAT_W         = 256;
  localparam int unsigned MEM_W          = 64;
  localparam int unsigned BASE_ADDR_WORD = 100;
  localparam int unsigned BANKS          = 4;
  localparam int unsigned TAG_W          = 16;

  localparam int unsigned IW = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW = (TILES_X <= 1) ? 1 : $clog2(TILES_X);
  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  localparam int unsigned BEATS = (FEAT_W + MEM_W - 1) / MEM_W;
  localparam int unsigned WORDS_PER_TILE = (BEATS < 1) ? 1 : BEATS;

  // timeout cycles for any wait
  localparam int unsigned TIMEOUT_CYC = 200;

  // clock/reset
  reg clk, rst, en;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  // DUT I/O
  reg                   rd_valid;
  wire                  rd_ready_in;
  reg  [IW-1:0]          rd_tile_i;
  reg  [JW-1:0]          rd_tile_j;
  reg  [TAG_W-1:0]       rd_tag;

  wire                  mem_rd_valid;
  reg                   mem_rd_ready;
  wire [31:0]            mem_addr;
  wire [BANK_W-1:0]      mem_bank;
  wire [TAG_W-1:0]       mem_tag;
  wire [15:0]            mem_beats;

  feature_sram_rd_addr_encode #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS(BANKS),
    .TAG_W(TAG_W)
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
    .mem_beats(mem_beats)
  );

  // dump
  initial begin
    $dumpfile("tb_feature_sram_rd_addr_encode.vcd");
    $dumpvars(0, tb_feature_sram_rd_addr_encode);
  end

  // helpers
  task fatal_msg(input [200*8-1:0] msg);
    begin
      $display("[%0t] FATAL: %s", $time, msg);
      $display("  en=%0d rst=%0d rd_valid=%0d rd_ready_in=%0d mem_rd_valid=%0d mem_rd_ready=%0d mem_addr=%0d mem_bank=%0d mem_tag=%h",
               en, rst, rd_valid, rd_ready_in, mem_rd_valid, mem_rd_ready, mem_addr, mem_bank, mem_tag);
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

  task expect_eq_hexN(input [TAG_W-1:0] got, input [TAG_W-1:0] exp, input [200*8-1:0] msg);
    begin
      if (got !== exp) begin
        $display("[%0t] FAIL: %s got=%h exp=%h", $time, msg, got, exp);
        $fatal(1);
      end
    end
  endtask

  task calc_expected(
    input  integer ti,
    input  integer tj,
    output integer exp_bank,
    output integer exp_addr
  );
    integer linear, base_word, bank_mask;
    begin
      linear    = ti * TILES_X + tj;
      base_word = BASE_ADDR_WORD + linear * WORDS_PER_TILE;
      if (BANKS <= 1) begin
        exp_bank = 0;
        exp_addr = base_word;
      end else begin
        bank_mask = (1 << BANK_W) - 1;
        exp_bank  = base_word & bank_mask;
        exp_addr  = base_word >>> BANK_W;
      end
    end
  endtask

  // wait with timeout
  task wait_cond_with_timeout(input [200*8-1:0] what);
    int unsigned k;
    begin
      k = 0;
      while (k < TIMEOUT_CYC) begin
        @(posedge clk);
        k++;
        // user will check condition outside (so just used as template) - not used
      end
      fatal_msg({what, " TIMEOUT"});
    end
  endtask

  // send one req; must see rd_ready_in within TIMEOUT_CYC
  task send_req(input integer ti, input integer tj, input [TAG_W-1:0] tagv);
  int unsigned k;
  reg done;
  begin
    // setup before posedge
    @(negedge clk);
    rd_tile_i = IW'(ti);
    rd_tile_j = JW'(tj);
    rd_tag    = tagv;
    rd_valid  = 1'b1;

    // wait until a POSEDGE where rd_ready_in=1 (handshake at posedge)
    done = 1'b0;
    k    = 0;
    while (!done) begin
      @(posedge clk);
      if (rd_ready_in) begin
        done = 1'b1;
      end else begin
        k = k + 1;
        if (k >= TIMEOUT_CYC) fatal_msg("send_req: wait rd_ready_in(posedge) TIMEOUT");
      end
    end

    // drop valid after handshake (next negedge is safe)
    @(negedge clk);
    rd_valid = 1'b0;
  end
endtask



  // wait mem_rd_valid with timeout
  task wait_valid;
    int unsigned k;
    begin
      k = 0;
      while (mem_rd_valid !== 1'b1) begin
        @(posedge clk);
        k++;
        if (k >= TIMEOUT_CYC) fatal_msg("wait_valid: mem_rd_valid TIMEOUT");
      end
    end
  endtask

  // wait pop handshake with timeout
  task wait_pop;
  int unsigned k;
  reg done;
  begin
    // 先等 mem_rd_valid 真的變 1
    k = 0;
    while (mem_rd_valid !== 1'b1) begin
      @(posedge clk);
      k++;
      if (k >= TIMEOUT_CYC) fatal_msg("wait_pop: mem_rd_valid TIMEOUT");
    end

    // ★重點：一定要等到「某一個 posedge」上 handshake 為真
    done = 1'b0;
    k    = 0;
    while (!done) begin
      @(posedge clk);
      if (mem_rd_valid && mem_rd_ready) begin
        done = 1'b1;   // pop edge 確認完成
      end else begin
        k++;
        if (k >= TIMEOUT_CYC) fatal_msg("wait_pop: handshake TIMEOUT");
      end
    end
  end
endtask


  // hold check
  reg [31:0]       hold_addr;
  reg [BANK_W-1:0] hold_bank;
  reg [TAG_W-1:0]  hold_tag;

  task snap_out;
    begin
      hold_addr = mem_addr;
      hold_bank = mem_bank;
      hold_tag  = mem_tag;
    end
  endtask

  task expect_out_held(input [200*8-1:0] msg);
    begin
      if (mem_addr !== hold_addr) fatal_msg({msg, " mem_addr changed while stalled"});
      if (mem_bank !== hold_bank) fatal_msg({msg, " mem_bank changed while stalled"});
      if (mem_tag  !== hold_tag)  fatal_msg({msg, " mem_tag changed while stalled"});
    end
  endtask

  // main
  integer exp_bank, exp_addr;
  integer rr;

  initial begin
    en = 1'b0;
    rst = 1'b1;
    rd_valid  = 1'b0;
    rd_tile_i = '0;
    rd_tile_j = '0;
    rd_tag    = '0;
    mem_rd_ready = 1'b0;

    repeat (5) @(posedge clk);
    en  = 1'b1;
    rst = 1'b0;

    $display("=== TB start: feature_sram_rd_addr_encode ===");

    // Sanity: go must be true now
    @(posedge clk);
    if (!(en && !rst)) fatal_msg("go not asserted (en && !rst should be 1)");

    // Test0: mem_beats
    $display("[%0t] Test0: check mem_beats", $time);
    expect_eq_int(mem_beats, BEATS, "mem_beats mismatch");

    // Test1
    $display("[%0t] Test1: basic mapping", $time);
    mem_rd_ready = 1'b1;

    send_req(0,0,16'hA100);
    wait_valid();

    calc_expected(0,0,exp_bank,exp_addr);
    expect_eq_int(mem_bank, exp_bank, "bank mismatch (0,0)");
    expect_eq_int(mem_addr, exp_addr, "addr mismatch (0,0)");
    expect_eq_hexN(mem_tag, 16'hA100, "tag mismatch (0,0)");

    wait_pop();
    @(negedge clk);
    if (mem_rd_valid) fatal_msg("expected buffer empty after pop");
    $display("PASS: basic mapping");

    // Test2: stall hold
    $display("[%0t] Test2: stall hold", $time);
    mem_rd_ready = 1'b0;
    send_req(2,5,16'hBEEF);
    wait_valid();
    snap_out();

    repeat (10) begin
      @(posedge clk);
      rr = $random;
      rd_tile_i <= rr[IW-1:0];
      rd_tile_j <= rr[JW-1:0];
      rd_tag    <= rr[TAG_W-1:0];
      expect_out_held("stall hold");
    end

    mem_rd_ready = 1'b1;
    wait_pop();


    @(negedge clk);
    if (mem_rd_valid) fatal_msg("expected empty after stall pop");
    $display("PASS: stall hold");

    // Test3: overwrite-on-pop
    $display("[%0t] Test3: overwrite-on-pop", $time);
    mem_rd_ready = 1'b0;
    send_req(1,2,16'h1111);
    wait_valid();#1;

    // prepare simultaneous pop+push
    @(negedge clk);
    mem_rd_ready = 1'b1;
    rd_tile_i    = IW'(3);
    rd_tile_j    = JW'(6);
    rd_tag       = 16'h2222;
    rd_valid     = 1'b1;

    @(posedge clk);
    if (!rd_ready_in)  fatal_msg("overwrite-on-pop: rd_ready_in should be 1");
    if (!mem_rd_valid) fatal_msg("overwrite-on-pop: should remain full");

    @(negedge clk);
    rd_valid = 1'b0;

    calc_expected(3,6,exp_bank,exp_addr);
    expect_eq_int(mem_bank, exp_bank, "overwrite bank mismatch (3,6)");
    expect_eq_int(mem_addr, exp_addr, "overwrite addr mismatch (3,6)");
    expect_eq_hexN(mem_tag, 16'h2222, "overwrite tag mismatch");

    wait_pop();
    @(negedge clk);
    if (mem_rd_valid) fatal_msg("expected empty after overwrite pop");
    $display("PASS: overwrite-on-pop");

    $display("=== TB PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
