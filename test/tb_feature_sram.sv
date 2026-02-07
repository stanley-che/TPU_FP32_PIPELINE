// ============================================================
// tb_feature_sram.sv
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_feature_sram.vvp ./test/tb_feature_sram.sv
//
// Run:
// vvp ./vvp/tb_feature_sram.vvp
// gtkwave ./vvp/tb_feature_sram.vcd
// ============================================================

`include "./src/AMOLED/feature_sram/feature_sram.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_feature_sram;

  // -----------------------------
  // Small TB config
  // -----------------------------
  localparam int unsigned TILE_X   = 32;
  localparam int unsigned TILE_Y   = 32;

  localparam int unsigned SRAM_BUS = 32;
  localparam int unsigned FEAT_DIM = 8;   // 8*32=256 => WORDS_PER_FEAT=8
  localparam int unsigned ELEN_W   = 32;
  localparam int unsigned TAG_W    = 16;

  localparam int unsigned FEAT_W         = FEAT_DIM * ELEN_W;                 // 256
  localparam int unsigned WORDS_PER_FEAT = (FEAT_W + SRAM_BUS - 1)/SRAM_BUS;   // 8
  localparam int unsigned TOTAL_WORDS    = TILE_X*TILE_Y*WORDS_PER_FEAT;
  localparam int unsigned MEM_AW         = (TOTAL_WORDS <= 1) ? 1 : $clog2(TOTAL_WORDS);

  localparam int unsigned RD_LAT    = 2;
  localparam int unsigned MEM_WORDS = (1 << MEM_AW);

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst;

  // -----------------------------
  // tile write/read ports
  // -----------------------------
  logic                      valid_wr, ready_wr;
  logic [$clog2(TILE_Y)-1:0] tile_i_wr;
  logic [$clog2(TILE_X)-1:0] tile_j_wr;
  logic [FEAT_W-1:0]         feat_in;

  logic                      valid_rd, ready_rd;
  logic [$clog2(TILE_Y)-1:0] tile_i_rd;
  logic [$clog2(TILE_X)-1:0] tile_j_rd;
  logic [TAG_W-1:0]          tag_rd;

  // -----------------------------
  // feature out
  // -----------------------------
  logic                      feat_out_valid;
  logic                      feat_out_ready;
  logic [TAG_W-1:0]          feat_out_tag;
  logic [FEAT_W-1:0]         feat_out_data;

  // -----------------------------
  // DUT
  // -----------------------------
  feature_sram #(
    .tile_x(TILE_X),
    .tile_y(TILE_Y),
    .sram_bus(SRAM_BUS),
    .feat_dim(FEAT_DIM),
    .elen_W(ELEN_W),
    .tag_w(TAG_W),
    .isclamp(1'b0),

    .REQ_DEPTH(2),
    .META_DEPTH(16),
    .BEAT_DEPTH(16),
    .RD_FIFO_DEPTH(2),

    .RD_LAT(RD_LAT),
    .MEM_WORDS(MEM_WORDS)
  ) dut (
    .clk(clk),
    .rst(rst),

    .valid_wr(valid_wr),
    .ready_wr(ready_wr),
    .tile_i_wr(tile_i_wr),
    .tile_j_wr(tile_j_wr),
    .feat_in(feat_in),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data)
  );

  // -----------------------------
  // waveform
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_feature_sram.vcd");
    $dumpvars(0, tb_feature_sram);
  end

  // ============================================================
  // random-ish backpressure on feat_out_ready
  // ============================================================
  int unsigned rng = 32'hc0ffee;
  function automatic int unsigned urand();
    begin
      rng ^= (rng << 13);
      rng ^= (rng >> 17);
      rng ^= (rng << 5);
      urand = rng;
    end
  endfunction

  always @(posedge clk) begin
    if (rst) feat_out_ready <= 1'b0;
    else begin
      // ~70% ready
      feat_out_ready <= ((urand() % 10) < 7);
    end
  end

  // ============================================================
  // tasks
  // ============================================================
  task automatic drive_idle();
    begin
      valid_wr  <= 1'b0;
      valid_rd  <= 1'b0;
      tile_i_wr <= '0;
      tile_j_wr <= '0;
      tile_i_rd <= '0;
      tile_j_rd <= '0;
      feat_in   <= '0;
      tag_rd    <= '0;
    end
  endtask

  task automatic tile_wr_once(
    input int unsigned ti,
    input int unsigned tj,
    input logic [FEAT_W-1:0] data
  );
    int unsigned t;
    begin
      tile_i_wr <= ti[$clog2(TILE_Y)-1:0];
      tile_j_wr <= tj[$clog2(TILE_X)-1:0];
      feat_in   <= data;
      valid_wr  <= 1'b1;

      t = 0;
      while (!(valid_wr && ready_wr)) begin
        @(posedge clk);
        t++;
        if (t > 5000) begin
          $display("ERROR: tile_wr_once timeout");
          $finish(1);
        end
      end

      @(posedge clk);
      valid_wr <= 1'b0;
    end
  endtask

  task automatic tile_rd_once(
    input int unsigned ti,
    input int unsigned tj,
    input logic [TAG_W-1:0] tag
  );
    int unsigned t;
    begin
      tile_i_rd <= ti[$clog2(TILE_Y)-1:0];
      tile_j_rd <= tj[$clog2(TILE_X)-1:0];
      tag_rd    <= tag;
      valid_rd  <= 1'b1;

      t = 0;
      while (!(valid_rd && ready_rd)) begin
        @(posedge clk);
        t++;
        if (t > 5000) begin
          $display("ERROR: tile_rd_once timeout");
          $finish(1);
        end
      end

      @(posedge clk);
      valid_rd <= 1'b0;
    end
  endtask

  task automatic wait_feat_out_hs(input int unsigned timeout_cycles);
    int unsigned t;
    begin
      t = 0;
      while (!(feat_out_valid && feat_out_ready)) begin
        @(posedge clk);
        t++;
        if (t > timeout_cycles) begin
          $display("ERROR: wait_feat_out_hs timeout");
          $finish(1);
        end
      end
    end
  endtask

  // ============================================================
  // Debug print (印出整個 feat_out_data 每個 word)
  // 放在 TB 裡的 module scope，跟 other always 同層即可
  // ============================================================
  integer k;
  always @(posedge clk) begin
    if (!rst && feat_out_valid && feat_out_ready) begin
      $display("[FEAT] tag=0x%h t=%0t", feat_out_tag, $time);
      for (k = 0; k < WORDS_PER_FEAT; k = k + 1) begin
        $display("  word[%0d]=0x%08x",
                 k,
                 feat_out_data[k*SRAM_BUS +: SRAM_BUS]);
      end
    end
  end

  // ============================================================
  // reference: pack word -> vector
  // word0 is [31:0], word7 is [255:224]
  // ============================================================
  function automatic [FEAT_W-1:0] pack_words(
    input logic [31:0] w0,
    input logic [31:0] w1,
    input logic [31:0] w2,
    input logic [31:0] w3,
    input logic [31:0] w4,
    input logic [31:0] w5,
    input logic [31:0] w6,
    input logic [31:0] w7
  );
    pack_words = {w7,w6,w5,w4,w3,w2,w1,w0};
  endfunction

  // ============================================================
  // main test
  // ============================================================
  logic [FEAT_W-1:0] exp_feat;
  logic [TAG_W-1:0]  exp_tag;

  initial begin
    rst = 1'b1;
    drive_idle();

    exp_tag  = 16'hBEEF;
    exp_feat = pack_words(
      32'h0000_0000,
      32'h1111_1111,
      32'h2222_2222,
      32'h3333_3333,
      32'h4444_4444,
      32'h5555_5555,
      32'h6666_6666,
      32'h7777_7777
    );

    repeat (10) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);

    // 1) write tile (2,3)
    $display("== TILE WR (2,3) ==");
    tile_wr_once(2, 3, exp_feat);

    // let engine finish writes
    repeat (300) @(posedge clk);

    // 2) read tile (2,3)
    $display("== TILE RD (2,3) ==");
    tile_rd_once(2, 3, exp_tag);

    // wait for output
    wait_feat_out_hs(20000);

    // check
    if (feat_out_tag !== exp_tag) begin
      $display("ERROR: feat_out_tag mismatch exp=0x%h got=0x%h", exp_tag, feat_out_tag);
      $finish(1);
    end
    if (feat_out_data !== exp_feat) begin
      $display("ERROR: feat_out_data mismatch");
      $display("  exp=0x%h", exp_feat);
      $display("  got=0x%h", feat_out_data);
      $finish(1);
    end

    $display("PASS ✅ tb_tile_feat_sram_burst_system_with_phy");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
