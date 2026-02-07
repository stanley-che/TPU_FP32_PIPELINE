// ============================================================
// tb_burst_req_if_tileaddr_wrap.sv  (tasks + testcases, no hang)
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src \
//   -o ./vvp/tb_burst_req_if_tileaddr_wrap.vvp \
//   ./test/tb_burst_req_if_tileaddr_wrap.sv
//
// Run:
// vvp ./vvp/tb_burst_req_if_tileaddr_wrap.vvp
// gtkwave ./vvp/tb_burst_req_if_tileaddr_wrap.vcd
// ============================================================

`include "./src/AMOLED/feature_sram/burst_req_if_tileaddr_wrap.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_burst_req_if_tileaddr_wrap;

  // ------------------------------------------------------------
  // parameters (match your real config)
  // ------------------------------------------------------------
  localparam int unsigned TILE_X   = 320;
  localparam int unsigned TILE_Y   = 180;
  localparam int unsigned SRAM_BUS = 32;
  localparam int unsigned FEAT_DIM = 8;
  localparam int unsigned ELEN_W   = 32;
  localparam int unsigned TAG_W    = 16;

  localparam int unsigned FEAT_BITS      = FEAT_DIM * ELEN_W; // 256
  localparam int unsigned WORDS_PER_FEAT = (FEAT_BITS + SRAM_BUS - 1) / SRAM_BUS; // 8
  localparam int unsigned TOTAL_WORDS    = TILE_X * TILE_Y * WORDS_PER_FEAT;
  localparam int unsigned MEM_AW         = (TOTAL_WORDS <= 1) ? 1 : $clog2(TOTAL_WORDS);

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst;

  // ------------------------------------------------------------
  // DUT I/O
  // ------------------------------------------------------------
  logic valid_wr, ready_wr;
  logic [$clog2(TILE_Y)-1:0] tile_i_wr;
  logic [$clog2(TILE_X)-1:0] tile_j_wr;
  logic [FEAT_BITS-1:0]      feat_in;

  logic valid_rd, ready_rd;
  logic [$clog2(TILE_Y)-1:0] tile_i_rd;
  logic [$clog2(TILE_X)-1:0] tile_j_rd;
  logic [TAG_W-1:0]          tag_rd;

  logic req_valid, req_ready;
  logic req_is_wr;
  logic [MEM_AW-1:0]         sram_addr;
  logic [FEAT_BITS-1:0]      sram_wdata;
  logic [TAG_W-1:0]          sram_tag;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  burst_req_if_tileaddr_wrap #(
    .tile_x(TILE_X),
    .tile_y(TILE_Y),
    .sram_bus(SRAM_BUS),
    .feat_dim(FEAT_DIM),
    .elen_W(ELEN_W),
    .tag_w(TAG_W),
    .isclamp(1'b0)
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

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .sram_addr(sram_addr),
    .sram_wdata(sram_wdata),
    .sram_tag(sram_tag)
  );

  // ------------------------------------------------------------
  // req_ready stub (always ready)
  // ------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) req_ready <= 1'b0;
    else     req_ready <= 1'b1;
  end

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_burst_req_if_tileaddr_wrap.vcd");
    $dumpvars(0, tb_burst_req_if_tileaddr_wrap);
  end

  // ------------------------------------------------------------
  // helper: expected base address (word address)
  // base = (tile_i * TILE_X + tile_j) * WORDS_PER_FEAT
  // ------------------------------------------------------------
  function automatic [MEM_AW-1:0] exp_addr(input int unsigned ti, input int unsigned tj);
    exp_addr = (ti * TILE_X + tj) * WORDS_PER_FEAT;
  endfunction

  // ------------------------------------------------------------
  // monitor (log output-side handshake)
  // ------------------------------------------------------------
  always @(posedge clk) begin
    if (req_valid && req_ready) begin
      if (req_is_wr) begin
        $display("[HS] WR base_addr=%0d time=%0t", sram_addr, $time);
      end else begin
        $display("[HS] RD base_addr=%0d tag=0x%h time=%0t", sram_addr, sram_tag, $time);
      end
    end
  end

  // ============================================================
  // Common tasks
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

  task automatic wait_out_hs(input int unsigned timeout_cycles);
    int unsigned t;
    begin
      t = 0;
      while (!(req_valid && req_ready)) begin
        @(posedge clk);
        t++;
        if (t > timeout_cycles) begin
          $display("ERROR: wait_out_hs timeout");
          $finish(1);
        end
      end
    end
  endtask

  // one-shot write: deassert right after INPUT fire
  task automatic do_wr_fire(
    input int unsigned ti,
    input int unsigned tj,
    input logic [31:0] low32
  );
    logic [FEAT_BITS-1:0] v;
    int unsigned t;
    begin
      v = '0;
      v[31:0] = low32;

      tile_i_wr <= ti[$clog2(TILE_Y)-1:0];
      tile_j_wr <= tj[$clog2(TILE_X)-1:0];
      feat_in   <= v;
      valid_wr  <= 1'b1;

      // wait INPUT fire: valid_wr && ready_wr
      t = 0;
      while (!(valid_wr && ready_wr)) begin
        @(posedge clk);
        t++;
        if (t > 2000) begin
          $display("ERROR: do_wr_fire timeout waiting ready_wr");
          $finish(1);
        end
      end

      // drop valid next cycle (prevent double-issue)
      @(posedge clk);
      valid_wr <= 1'b0;
    end
  endtask

  // one-shot read: deassert right after INPUT fire
  task automatic do_rd_fire(
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

      // wait INPUT fire: valid_rd && ready_rd
      t = 0;
      while (!(valid_rd && ready_rd)) begin
        @(posedge clk);
        t++;
        if (t > 2000) begin
          $display("ERROR: do_rd_fire timeout waiting ready_rd");
          $finish(1);
        end
      end

      @(posedge clk);
      valid_rd <= 1'b0;
    end
  endtask

  task automatic expect_wr_addr(input [MEM_AW-1:0] exp);
    begin
      if (!(req_valid && req_ready && req_is_wr)) begin
        // if caller didn't align with hs, just check signals at that moment:
        // caller should call this right after wait_out_hs
      end
      if (req_is_wr !== 1'b1) begin
        $display("ERROR: expect WR but got RD");
        $finish(1);
      end
      if (sram_addr !== exp) begin
        $display("ERROR: WR addr mismatch exp=%0d got=%0d", exp, sram_addr);
        $finish(1);
      end
    end
  endtask

  task automatic expect_rd_addr_tag(input [MEM_AW-1:0] exp, input logic [TAG_W-1:0] tag);
    begin
      if (req_is_wr !== 1'b0) begin
        $display("ERROR: expect RD but got WR");
        $finish(1);
      end
      if (sram_addr !== exp) begin
        $display("ERROR: RD addr mismatch exp=%0d got=%0d", exp, sram_addr);
        $finish(1);
      end
      if (sram_tag !== tag) begin
        $display("ERROR: RD tag mismatch exp=0x%h got=0x%h", tag, sram_tag);
        $finish(1);
      end
    end
  endtask

  // ============================================================
  // Testcases
  // ============================================================

  task automatic testcase_write_addr();
    begin
      $display("== testcase_write_addr ==");
      do_wr_fire(2, 3, 32'hDEADBEEF);

      wait_out_hs(2000);
      expect_wr_addr(exp_addr(2,3));
    end
  endtask

  task automatic testcase_read_addr();
    begin
      $display("== testcase_read_addr ==");
      do_rd_fire(1, 6, 16'h00A5);

      wait_out_hs(2000);
      expect_rd_addr_tag(exp_addr(1,6), 16'h00A5);
    end
  endtask

  task automatic testcase_write_priority();
    begin
      $display("== testcase_write_priority ==");

      // assert both valids; wrapper should accept WR first (write priority)
      feat_in   <= '0;
      feat_in[31:0] <= 32'h12345678;

      tile_i_wr <= 0;
      tile_j_wr <= 1;
      valid_wr  <= 1'b1;

      tile_i_rd <= 3;
      tile_j_rd <= 2;
      tag_rd    <= 16'h0055;
      valid_rd  <= 1'b1;

      // wait for input fire of WR (ready_wr must be 1)
      // (drop both valids right after one cycle is OK too, but this is safer)
      while (!(valid_wr && ready_wr)) @(posedge clk);

      @(posedge clk);
      valid_wr <= 1'b0;
      valid_rd <= 1'b0;

      // now wait output handshake and confirm it's WR + correct addr
      wait_out_hs(2000);
      if (req_is_wr !== 1'b1) begin
        $display("ERROR: WRITE priority violated (got RD)");
        $finish(1);
      end
      if (sram_addr !== exp_addr(0,1)) begin
        $display("ERROR: priority WR addr wrong exp=%0d got=%0d", exp_addr(0,1), sram_addr);
        $finish(1);
      end
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    rst = 1'b1;
    drive_idle();

    repeat (5) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);

    testcase_write_addr();
    testcase_read_addr();
    testcase_write_priority();

    $display("PASS ✅ burst_req_if_tileaddr_wrap (all testcases)");
    #20;
    $finish;
  end

endmodule

`default_nettype wire
