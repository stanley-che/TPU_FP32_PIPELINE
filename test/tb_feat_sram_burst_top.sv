// ============================================================
// tb_feat_sram_burst_top.sv  (burst_engine + rd_assemble_fifo E2E)
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_feat_sram_burst_top.vvp ./test/tb_feat_sram_burst_top.sv
//
// Run:
// vvp ./vvp/tb_feat_sram_burst_top.vvp
// gtkwave ./vvp/tb_feat_sram_burst_top.vcd
// ============================================================
`include "./src/AMOLED/feature_sram/feat_sram_burst_top.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_feat_sram_burst_top;

  // ------------------------------------------------------------
  // params
  // ------------------------------------------------------------
  localparam int unsigned SRAM_BUS_W      = 32;
  localparam int unsigned FEAT_W          = 256;
  localparam int unsigned WORDS_PER_FEAT  = 8;
  localparam int unsigned TAG_W           = 16;

  // Keep MEM_AW reasonably small for TB SRAM model
  localparam int unsigned MEM_AW          = 12;  // 4096 words addressable
  localparam int unsigned MEM_WORDS       = (1 << MEM_AW);

  localparam int unsigned RD_LAT          = 2;   // SRAM read latency (cycles)

  // FIFO depths
  localparam int unsigned REQ_DEPTH       = 2;
  localparam int unsigned META_DEPTH      = 16;
  localparam int unsigned BEAT_DEPTH      = 16;
  localparam int unsigned RD_FIFO_DEPTH   = 2;

  localparam int unsigned BEAT_IDX_W =
    (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);

  // ------------------------------------------------------------
  // clock/reset
  // ------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // ------------------------------------------------------------
  // DUT request in
  // ------------------------------------------------------------
  logic               req_valid;
  logic               req_ready;
  logic               req_is_wr;
  logic [FEAT_W-1:0]  req_wdata;
  logic [TAG_W-1:0]   req_tag;
  logic [MEM_AW-1:0]  req_addr;

  // ------------------------------------------------------------
  // SRAM interface
  // ------------------------------------------------------------
  logic                  mem_req_valid;
  logic                  mem_req_ready;
  logic                  mem_req_is_wr;
  logic [MEM_AW-1:0]     mem_req_addr;
  logic [SRAM_BUS_W-1:0] mem_req_wdata;
  logic [SRAM_BUS_W/8-1:0] mem_req_wmask;

  logic                  mem_rvalid;
  logic [SRAM_BUS_W-1:0] mem_rdata;

  // ------------------------------------------------------------
  // Feature out
  // ------------------------------------------------------------
  logic               feat_out_valid;
  logic               feat_out_ready;
  logic [TAG_W-1:0]   feat_out_tag;
  logic [FEAT_W-1:0]  feat_out_data;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  feat_sram_burst_top #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .MEM_AW(MEM_AW),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(TAG_W),
    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH)
  ) dut (
    .clk(clk),
    .rst(rst),

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .req_wdata(req_wdata),
    .req_tag(req_tag),
    .req_addr(req_addr),

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

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_feat_sram_burst_top.vcd");
    $dumpvars(0, tb_feat_sram_burst_top);
  end

  // ============================================================
  // Simple SRAM model (word-addressed, RD_LAT cycles)
  // ============================================================
  logic [SRAM_BUS_W-1:0] mem [0:MEM_WORDS-1];

  logic [MEM_AW-1:0] rdq_addr [0:RD_LAT-1];
  logic              rdq_valid[0:RD_LAT-1];

  integer i;
  int unsigned rng = 32'hc0ffee;

  // random-ready for mem_req (like real SRAM ctrl)
  always @(posedge clk) begin
    if (rst) mem_req_ready <= 1'b0;
    else begin
      // ~80% ready
      rng <= rng * 32'h343fd + 32'h269ec3;
      mem_req_ready <= (rng[7:0] < 8'd205);
    end
  end

  // SRAM behavior
  always @(posedge clk) begin
    if (rst) begin
      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      for (i=0; i<RD_LAT; i++) begin
        rdq_valid[i] <= 1'b0;
        rdq_addr[i]  <= '0;
      end
    end else begin
      mem_rvalid <= 1'b0;

      // shift read pipeline
      for (i=RD_LAT-1; i>0; i--) begin
        rdq_valid[i] <= rdq_valid[i-1];
        rdq_addr[i]  <= rdq_addr[i-1];
      end
      rdq_valid[0] <= 1'b0;

      if (mem_req_valid && mem_req_ready) begin
        if (mem_req_is_wr) begin
          // ignore mask for now (assume full write)
          mem[mem_req_addr] <= mem_req_wdata;
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
  // Helpers
  // ============================================================
  function automatic [SRAM_BUS_W-1:0] get_word(input [FEAT_W-1:0] feat, input int unsigned idx);
    get_word = feat >> (idx * SRAM_BUS_W);
  endfunction

  // random backpressure on feature output
  always @(posedge clk) begin
    if (rst) feat_out_ready <= 1'b0;
    else begin
      // ~70% ready
      rng <= rng * 32'h343fd + 32'h269ec3;
      feat_out_ready <= (rng[7:0] < 8'd180);
    end
  end

  // request one-shot task (IMPORTANT)
  task automatic send_req(
    input logic is_wr,
    input logic [MEM_AW-1:0] base_addr,
    input logic [TAG_W-1:0]  tag,
    input logic [FEAT_W-1:0] wdata
  );
    int unsigned t;
    begin
      req_is_wr <= is_wr;
      req_addr  <= base_addr;
      req_tag   <= tag;
      req_wdata <= wdata;
      req_valid <= 1'b1;

      t = 0;
      while (!(req_valid && req_ready)) begin
        @(posedge clk);
        t++;
        if (t > 5000) begin
          $display("ERROR: send_req timeout waiting req_ready");
          $finish(1);
        end
      end

      @(posedge clk);
      req_valid <= 1'b0;
    end
  endtask

  // wait for one feature output handshake
  task automatic wait_feat_hs(input int unsigned timeout_cycles);
    int unsigned t;
    begin
      t = 0;
      while (!(feat_out_valid && feat_out_ready)) begin
        @(posedge clk);
        t++;
        if (t > timeout_cycles) begin
          $display("ERROR: wait_feat_hs timeout");
          $finish(1);
        end
      end
    end
  endtask

  // ============================================================
  // Monitor logs
  // ============================================================
  always @(posedge clk) begin
    if (!rst && mem_req_valid && mem_req_ready) begin
      if (mem_req_is_wr)
        $display("[MEM] WR addr=%0d data=0x%08x t=%0t", mem_req_addr, mem_req_wdata, $time);
      else
        $display("[MEM] RD addr=%0d t=%0t", mem_req_addr, $time);
    end
    if (!rst && feat_out_valid && feat_out_ready) begin
      $display("[FEAT] tag=0x%h data[31:0]=0x%08x t=%0t",
               feat_out_tag, feat_out_data[31:0], $time);
    end
  end

  // ============================================================
  // Test
  // ============================================================
  logic [FEAT_W-1:0] exp_feat;
  logic [TAG_W-1:0]  exp_tag;
  logic [MEM_AW-1:0] base;

  initial begin
    // init
    rst      = 1'b1;
    req_valid= 1'b0;
    req_is_wr= 1'b0;
    req_wdata= '0;
    req_tag  = '0;
    req_addr = '0;

    base = 12'h123;
    exp_tag = 16'hBEEF;

    // build expected feature: word[idx]=0x1111_1111 * idx pattern-ish
    // (LSW = idx0 at [31:0], idx7 at [255:224])
    exp_feat = {
      32'h7777_7777,32'h6666_6666,32'h5555_5555,32'h4444_4444,
      32'h3333_3333,32'h2222_2222,32'h1111_1111,32'h0000_0000
    };

    repeat (10) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);

    // -------------------------
    // 1) WRITE feature to SRAM
    // -------------------------
    $display("== WRITE ==");
    send_req(1'b1, base, exp_tag, exp_feat);

    // wait enough cycles for all beats to be issued
    repeat (300) @(posedge clk);

    // (optional) sanity check memory contents
    // $display("MEM[%0d]=0x%08x", base+0, mem[base+0]);

    // -------------------------
    // 2) READ it back
    // -------------------------
    $display("== READ ==");
    send_req(1'b0, base, exp_tag, '0);

    // wait for feature output
    wait_feat_hs(8000);

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

    $display("PASS ✅ feat_sram_burst_top E2E");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
