// ============================================================
// tb_tile_feat_sram_burst_system.sv  (FULL E2E)
// tile wr/rd -> burst -> SRAM model -> assemble -> feat_out
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_tile_feat_sram_burst_system.vvp ./test/tb_tile_feat_sram_burst_system.sv
//
// Run:
// vvp ./vvp/tb_tile_feat_sram_burst_system.vvp
// gtkwave ./vvp/tb_tile_feat_sram_burst_system.vcd
// ============================================================
`include "./src/AMOLED/feature_sram/tile_feat_sram_burst_system.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_tile_feat_sram_burst_system;

  // -----------------------------
  // Use SMALL tile dims in TB
  // -----------------------------
  localparam int unsigned TILE_X   = 32;
  localparam int unsigned TILE_Y   = 32;

  localparam int unsigned SRAM_BUS = 32;
  localparam int unsigned FEAT_DIM = 16;
  localparam int unsigned ELEN_W   = 32;
  localparam int unsigned TAG_W    = 16;

  localparam int unsigned FEAT_W         = FEAT_DIM * ELEN_W; // 256
  localparam int unsigned WORDS_PER_FEAT = (FEAT_W + SRAM_BUS - 1)/SRAM_BUS; // 8

  localparam int unsigned TOTAL_WORDS = TILE_X*TILE_Y*WORDS_PER_FEAT;
  localparam int unsigned MEM_AW      = (TOTAL_WORDS <= 1) ? 1 : $clog2(TOTAL_WORDS);

  localparam int unsigned MEM_WORDS = (1 << MEM_AW);
  localparam int unsigned RD_LAT    = 2;

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
  // SRAM interface
  // -----------------------------
  logic                    mem_req_valid;
  logic                    mem_req_ready;
  logic                    mem_req_is_wr;
  logic [MEM_AW-1:0]       mem_req_addr;
  logic [SRAM_BUS-1:0]     mem_req_wdata;
  logic [SRAM_BUS/8-1:0]   mem_req_wmask;

  logic                    mem_rvalid;
  logic [SRAM_BUS-1:0]     mem_rdata;

  // -----------------------------
  // feature out
  // -----------------------------
  logic                    feat_out_valid;
  logic                    feat_out_ready;
  logic [TAG_W-1:0]        feat_out_tag;
  logic [FEAT_W-1:0]       feat_out_data;

  // -----------------------------
  // DUT
  // -----------------------------
  tile_feat_sram_burst_system #(
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
    .RD_FIFO_DEPTH(2)
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

    .mem_req_valid(mem_req_valid),
    .mem_req_ready(mem_req_ready),
    .mem_req_is_wr(mem_req_is_wr),
    .mem_req_addr(mem_req_addr),
    .mem_req_wdata(mem_req_wdata),
    .mem_req_wmask(mem_req_wmask),

    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data)
  );

  // -----------------------------
  // waveform
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_tile_feat_sram_burst_system.vcd");
    $dumpvars(0, tb_tile_feat_sram_burst_system);
  end

  // ============================================================
  // SRAM model
  // ============================================================
  logic [SRAM_BUS-1:0] mem [0:MEM_WORDS-1];

  logic [MEM_AW-1:0] rdq_addr [0:RD_LAT-1];
  logic              rdq_valid[0:RD_LAT-1];

  integer i;
  int unsigned rng = 32'hc0ffee;

  // make mem_req_ready fairly high to avoid TB timeouts
  always @(posedge clk) begin
    if (rst) mem_req_ready <= 1'b0;
    else begin
      rng <= rng * 32'h343fd + 32'h269ec3;
      mem_req_ready <= (rng[7:0] < 8'd230); // ~90%
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      for (i=0;i<RD_LAT;i++) begin
        rdq_valid[i] <= 1'b0;
        rdq_addr[i]  <= '0;
      end
    end else begin
      mem_rvalid <= 1'b0;

      for (i=RD_LAT-1;i>0;i--) begin
        rdq_valid[i] <= rdq_valid[i-1];
        rdq_addr[i]  <= rdq_addr[i-1];
      end
      rdq_valid[0] <= 1'b0;

      if (mem_req_valid && mem_req_ready) begin
        if (mem_req_is_wr) begin
          mem[mem_req_addr] <= mem_req_wdata; // ignore mask
        end else begin
          rdq_valid[0] <= 1'b1;
          rdq_addr[0]  <= mem_req_addr;
        end
      end

      if (rdq_valid[RD_LAT-1]) begin
        mem_rvalid <= 1'b1;
        mem_rdata  <= mem[rdq_addr[RD_LAT-1]];
      end
    end
  end

  // ============================================================
  // downstream backpressure on feat_out
  // ============================================================
  always @(posedge clk) begin
    if (rst) feat_out_ready <= 1'b0;
    else begin
      rng <= rng * 32'h343fd + 32'h269ec3;
      feat_out_ready <= (rng[7:0] < 8'd180); // ~70%
    end
  end

  // ============================================================
  // tasks
  // ============================================================
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

  task automatic wait_feat_out(input int unsigned timeout_cycles);
    int unsigned t;
    begin
      t = 0;
      while (!(feat_out_valid && feat_out_ready)) begin
        @(posedge clk);
        t++;
        if (t > timeout_cycles) begin
          $display("ERROR: wait_feat_out timeout");
          $finish(1);
        end
      end
    end
  endtask

  // ============================================================
  // debug prints
  // ============================================================
  integer k;
  always @(posedge clk) begin
    if (!rst && mem_req_valid && mem_req_ready) begin
      if (mem_req_is_wr)
        $display("[MEM] WR addr=%0d data=0x%08x t=%0t", mem_req_addr, mem_req_wdata, $time);
      else
        $display("[MEM] RD addr=%0d t=%0t", mem_req_addr, $time);
    end
    if (!rst && feat_out_valid && feat_out_ready) begin
    $display("[FEAT] tag=0x%h t=%0t", feat_out_tag, $time);
    for (k = 0; k < WORDS_PER_FEAT; k = k + 1) begin
      $display("  word[%0d]=0x%08x",
               k,
               feat_out_data[k*SRAM_BUS+: SRAM_BUS]);
    end
  end
  end

  // ============================================================
  // main test
  // ============================================================
  logic [FEAT_W-1:0] exp_feat;
  logic [TAG_W-1:0]  exp_tag;

  initial begin
    rst = 1'b1;

    valid_wr  = 1'b0;
    valid_rd  = 1'b0;
    tile_i_wr = '0;
    tile_j_wr = '0;
    tile_i_rd = '0;
    tile_j_rd = '0;
    feat_in   = '0;
    tag_rd    = '0;

    exp_tag  = 16'hBEEF;
    exp_feat = {
      32'h7777_7777,32'h6666_6666,32'h5555_5555,32'h4444_4444,
      32'h3333_3333,32'h2222_2222,32'h1111_1111,32'h1234_5678,
      32'hABCD_EF01,32'hDEAD_BEEF,32'hC0DE_CAFE,32'hFACE_B00C,
      32'h0123_4567,32'h89AB_CDEF,32'hFEDC_BA98,32'h7654_3210
    };

    repeat (10) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);

    // 1) write tile (2,3)
    $display("== TILE WR (2,3) ==");
    tile_wr_once(2, 3, exp_feat);

    // let burst engine finish writes
    repeat (400) @(posedge clk);

    // 2) read tile (2,3)
    $display("== TILE RD (2,3) ==");
    tile_rd_once(2, 3, exp_tag);

    // wait feature out
    wait_feat_out(20000);

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

    $display("PASS ✅ tile_feat_sram_burst_system E2E");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
