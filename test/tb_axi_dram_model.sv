/*
iverilog -g2012 -Wall -o ./vvp/tb_axi_dram.vvp ./test/tb_axi_dram_model.sv 


vvp ./vvp/tb_axi_dram.vvp
*/
`include "./src/axi_dram_model.sv"
`timescale 1ns/1ps

`default_nettype none

module tb_axi_dram_model;

  // -----------------------------
  // Parameters
  // -----------------------------
  localparam int ADDR_W = 32;
  localparam int DATA_W = 32;
  localparam int STRB_W = DATA_W/8;
  localparam int DEPTH_WORD_AW = 12; // words = 2^12 = 4096 (16KB at 32b)

  // AXI burst constants
  localparam logic [1:0] BURST_INCR = 2'b01;
  localparam logic [2:0] SIZE_4B    = 3'd2;   // 2^2 = 4 bytes (for 32-bit)

  // -----------------------------
  // Clock / Reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // -----------------------------
  // DUT AXI signals (simplified)
  // -----------------------------
  // AR
  logic [ADDR_W-1:0] s_axi_araddr;
  logic [7:0]        s_axi_arlen;
  logic [2:0]        s_axi_arsize;
  logic [1:0]        s_axi_arburst;
  logic              s_axi_arvalid;
  wire               s_axi_arready;

  // R
  wire [DATA_W-1:0]  s_axi_rdata;
  wire               s_axi_rvalid;
  wire               s_axi_rlast;
  logic              s_axi_rready;

  // AW
  logic [ADDR_W-1:0] s_axi_awaddr;
  logic [7:0]        s_axi_awlen;
  logic [2:0]        s_axi_awsize;
  logic [1:0]        s_axi_awburst;
  logic              s_axi_awvalid;
  wire               s_axi_awready;

  // W
  logic [DATA_W-1:0] s_axi_wdata;
  logic [STRB_W-1:0] s_axi_wstrb;
  logic              s_axi_wlast;
  logic              s_axi_wvalid;
  wire               s_axi_wready;

  // B
  wire               s_axi_bvalid;
  logic              s_axi_bready;

  // -----------------------------
  // Instantiate DUT
  // -----------------------------
  axi_dram_model #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .STRB_W(STRB_W),
    .DEPTH_WORD_AW(DEPTH_WORD_AW)
  ) dut (
    .clk(clk),
    .rst(rst),

    .s_axi_araddr (s_axi_araddr),
    .s_axi_arlen  (s_axi_arlen),
    .s_axi_arsize (s_axi_arsize),
    .s_axi_arburst(s_axi_arburst),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),

    .s_axi_rdata  (s_axi_rdata),
    .s_axi_rvalid (s_axi_rvalid),
    .s_axi_rlast  (s_axi_rlast),
    .s_axi_rready (s_axi_rready),

    .s_axi_awaddr (s_axi_awaddr),
    .s_axi_awlen  (s_axi_awlen),
    .s_axi_awsize (s_axi_awsize),
    .s_axi_awburst(s_axi_awburst),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),

    .s_axi_wdata  (s_axi_wdata),
    .s_axi_wstrb  (s_axi_wstrb),
    .s_axi_wlast  (s_axi_wlast),
    .s_axi_wvalid (s_axi_wvalid),
    .s_axi_wready (s_axi_wready),

    .s_axi_bvalid (s_axi_bvalid),
    .s_axi_bready (s_axi_bready)
  );

  // -----------------------------
  // Simple scoreboard model (byte-addressed within DEPTH)
  // -----------------------------
  localparam int MEM_WORDS = (1 << DEPTH_WORD_AW);
  logic [DATA_W-1:0] sb_mem [0:MEM_WORDS-1];
   // -----------------------------
  // Simple scoreboard model (byte-addressed within DEPTH)
  // -----------------------------

  // ✅ 跟 axi_dram_model 裡完全一致的 RAM_ADDR_W
  localparam int RAM_ADDR_W = DEPTH_WORD_AW + $clog2(STRB_W);

  // ✅ Icarus-friendly mask: all-ones then shift right
  localparam [ADDR_W-1:0] RAM_ADDR_MASK = {ADDR_W{1'b1}} >> (ADDR_W - RAM_ADDR_W);


  function automatic int unsigned addr_to_word(input logic [ADDR_W-1:0] addr);
  logic [ADDR_W-1:0] masked;
  begin
    masked = addr & RAM_ADDR_MASK;             // 等價於截斷低 RAM_ADDR_W bits
    addr_to_word = masked >> $clog2(STRB_W);   // byte -> word
  end
endfunction



  // Apply write with strobe into scoreboard
  task automatic sb_write_word(
    input logic [ADDR_W-1:0] addr,
    input logic [DATA_W-1:0] data,
    input logic [STRB_W-1:0] strb
  );
    int unsigned w;
    logic [DATA_W-1:0] oldv, newv;
    begin
      w = addr_to_word(addr);
      oldv = sb_mem[w];
      newv = oldv;
      for (int b = 0; b < STRB_W; b++) begin
        if (strb[b]) newv[b*8 +: 8] = data[b*8 +: 8];
      end
      sb_mem[w] = newv;
    end
  endtask

  // -----------------------------
  // AXI-lite-ish helpers (but burst capable)
  // -----------------------------
  task automatic axi_reset();
    begin
      // drive defaults
      s_axi_araddr  = '0;
      s_axi_arlen   = '0;
      s_axi_arsize  = SIZE_4B;
      s_axi_arburst = BURST_INCR;
      s_axi_arvalid = 1'b0;

      s_axi_rready  = 1'b0;

      s_axi_awaddr  = '0;
      s_axi_awlen   = '0;
      s_axi_awsize  = SIZE_4B;
      s_axi_awburst = BURST_INCR;
      s_axi_awvalid = 1'b0;

      s_axi_wdata   = '0;
      s_axi_wstrb   = '0;
      s_axi_wlast   = 1'b0;
      s_axi_wvalid  = 1'b0;

      s_axi_bready  = 1'b0;

      // clear scoreboard
      for (int i = 0; i < MEM_WORDS; i++) sb_mem[i] = '0;

      // reset pulse
      rst = 1'b1;
      repeat (5) @(posedge clk);
      rst = 1'b0;
      repeat (2) @(posedge clk);
    end
  endtask

  // Write burst: len = number of beats (1..256); arlen/awlen = len-1
  task automatic axi_write_burst(
    input logic [ADDR_W-1:0] base_addr,
    input int unsigned       beats,
    input logic [DATA_W-1:0] data0,
    input logic [DATA_W-1:0] data_step,
    input logic [STRB_W-1:0] strb,
    input bit               random_w_stall = 0
  );
    int unsigned i;
    logic [ADDR_W-1:0] addr_i;
    logic [DATA_W-1:0] d;
    begin
      if (beats == 0 || beats > 256) $fatal(1, "beats must be 1..256");

      // AW
      @(posedge clk);
      s_axi_awaddr  <= base_addr;
      s_axi_awlen   <= beats-1;
      s_axi_awsize  <= SIZE_4B;
      s_axi_awburst <= BURST_INCR;
      s_axi_awvalid <= 1'b1;

      // wait for AW handshake
      while (!(s_axi_awvalid && s_axi_awready)) @(posedge clk);
      @(posedge clk);
      s_axi_awvalid <= 1'b0;

      // W beats
      d = data0;
      for (i = 0; i < beats; i++) begin
        addr_i = base_addr + i*STRB_W;

        // optional random stall of wvalid
        if (random_w_stall && ($urandom_range(0,3)==0)) begin
          s_axi_wvalid <= 1'b0;
          s_axi_wlast  <= 1'b0;
          @(posedge clk);
        end

        s_axi_wdata  <= d;
        s_axi_wstrb  <= strb;
        s_axi_wlast  <= (i == beats-1);
        s_axi_wvalid <= 1'b1;

        // wait for W handshake
        while (!(s_axi_wvalid && s_axi_wready)) @(posedge clk);

        // update scoreboard at accept time
        sb_write_word(addr_i, d, strb);

        @(posedge clk);
        s_axi_wvalid <= 1'b0;
        s_axi_wlast  <= 1'b0;

        d = d + data_step;
      end

      // B
      s_axi_bready <= 1'b1;
      while (!s_axi_bvalid) @(posedge clk);
      @(posedge clk);
      s_axi_bready <= 1'b0;
    end
  endtask

  // Read burst and check against scoreboard
  task automatic axi_read_burst_check(
    input logic [ADDR_W-1:0] base_addr,
    input int unsigned       beats,
    input bit               random_r_backpressure = 0
  );
    int unsigned i;
    logic [ADDR_W-1:0] addr_i;
    logic [DATA_W-1:0] exp;
    begin
      if (beats == 0 || beats > 256) $fatal(1, "beats must be 1..256");

      // AR
      @(posedge clk);
      s_axi_araddr  <= base_addr;
      s_axi_arlen   <= beats-1;
      s_axi_arsize  <= SIZE_4B;
      s_axi_arburst <= BURST_INCR;
      s_axi_arvalid <= 1'b1;

      while (!(s_axi_arvalid && s_axi_arready)) @(posedge clk);
      @(posedge clk);
      s_axi_arvalid <= 1'b0;

      // R beats
      i = 0;
      while (i < beats) begin
        // backpressure pattern
        if (random_r_backpressure) begin
          s_axi_rready <= ($urandom_range(0,2) != 0); // ~2/3 ready
        end else begin
          s_axi_rready <= 1'b1;
        end

        @(posedge clk);
        if (s_axi_rvalid && s_axi_rready) begin
          addr_i = base_addr + i*STRB_W;
          exp    = sb_mem[addr_to_word(addr_i)];

          if (s_axi_rdata !== exp) begin
            $display("[FAIL] READ MISMATCH beat=%0d addr=0x%08x got=0x%08x exp=0x%08x",
                     i, addr_i, s_axi_rdata, exp);
            $fatal(1);
          end

          if ((i == beats-1) && !s_axi_rlast) begin
            $display("[FAIL] RLAST not asserted on last beat");
            $fatal(1);
          end
          if ((i != beats-1) && s_axi_rlast) begin
            $display("[FAIL] RLAST asserted early at beat=%0d", i);
            $fatal(1);
          end

          i++;
        end
      end

      // finish
      s_axi_rready <= 1'b0;
      @(posedge clk);
    end
  endtask

  // -----------------------------
  // Test sequences
  // -----------------------------
  initial begin
    axi_reset();

    $display("\n[TEST1] single write/read, full strobe");
    axi_write_burst(32'h0000_0010, 1, 32'h11223344, 32'h0, 4'b1111);
    axi_read_burst_check(32'h0000_0010, 1);

    $display("\n[TEST2] burst write/read, 8 beats, full strobe");
    axi_write_burst(32'h0000_0100, 8, 32'h0000_1000, 32'h0000_0003, 4'b1111);
    axi_read_burst_check(32'h0000_0100, 8);

    $display("\n[TEST3] byte-lane strobe test (write only lower byte)");
    axi_write_burst(32'h0000_0200, 1, 32'hAABB_CCDD, 32'h0, 4'b0001);
    axi_read_burst_check(32'h0000_0200, 1);

    $display("\n[TEST4] mixed stall on W channel + read backpressure");
    axi_write_burst(32'h0000_0300, 16, 32'h0102_0304, 32'h1111_1111, 4'b1111, 1);
    axi_read_burst_check(32'h0000_0300, 16, 1);

    $display("\n[TEST5] random small tests");
    for (int t = 0; t < 50; t++) begin
      int unsigned beats;
      logic [ADDR_W-1:0] base;
      logic [DATA_W-1:0] d0, step;
      logic [STRB_W-1:0] st;

      beats = $urandom_range(1, 16);
      base  = ($urandom_range(0, (MEM_WORDS*STRB_W)-64)) & 32'hFFFF_FFFC; // align 4B
      d0    = $urandom();
      step  = $urandom();
      st    = $urandom_range(0, 15);

      // ensure at least one byte enabled sometimes
      if (st == 0) st = 4'b1111;

      axi_write_burst(base, beats, d0, step, st, 1);
      axi_read_burst_check(base, beats, 1);
    end

    $display("\n[PASS] All tests passed.");
    $finish;
  end

endmodule

`default_nettype wire
