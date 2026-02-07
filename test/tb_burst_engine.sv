// tb_burst_engine.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_burst_engine.vvp \
  ./test/tb_burst_engine.sv

vvp ./vvp/tb_burst_engine.vvp
gtkwave ./vvp/tb_burst_engine.vcd
*/
`include "./src/AMOLED/feature_sram/burst_engine.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_burst_engine;

  localparam int unsigned SRAM_BUS_W     = 32;
  localparam int unsigned MEM_AW         = 18;
  localparam int unsigned FEAT_W         = 256;
  localparam int unsigned WORDS_PER_FEAT = 8;
  localparam int unsigned TAG_W          = 16;

  localparam int unsigned BEAT_IDX_W =
    (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);

  localparam int unsigned MEM_WORDS = 1 << 12;
  localparam int unsigned RD_LAT    = 2;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst;

  // ------------------------------------------------------------
  // DUT signals
  // ------------------------------------------------------------
  logic req_valid, req_ready, req_is_wr;
  logic [FEAT_W-1:0] req_wdata;
  logic [TAG_W-1:0]  req_tag;
  logic [MEM_AW-1:0] req_addr;

  logic mem_req_valid, mem_req_ready, mem_req_is_wr;
  logic [MEM_AW-1:0] mem_req_addr;
  logic [SRAM_BUS_W-1:0] mem_req_wdata;
  logic [SRAM_BUS_W/8-1:0] mem_req_wmask;

  logic mem_rvalid;
  logic [SRAM_BUS_W-1:0] mem_rdata;

  logic beat_out_valid, beat_out_ready;
  logic [SRAM_BUS_W-1:0] beat_out_data;
  logic [TAG_W-1:0] beat_out_tag;
  logic [BEAT_IDX_W-1:0] beat_out_idx;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  burst_engine #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .MEM_AW(MEM_AW),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(TAG_W),
    .REQ_DEPTH(2),
    .META_DEPTH(16),
    .BEAT_DEPTH(16)
  ) dut (
    .* 
  );

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_burst_engine.vcd");
    $dumpvars(0, tb_burst_engine);
  end

  // ============================================================
  // Simple SRAM model (iverilog-safe)
  // ============================================================
  logic [SRAM_BUS_W-1:0] mem [0:MEM_WORDS-1];

  logic [MEM_AW-1:0] rdq_addr [0:RD_LAT-1];
  logic rdq_valid [0:RD_LAT-1];

  integer i;
  int unsigned seed = 32'hc0ffee;

  // ready signals
  always_ff @(posedge clk) begin
    if (rst) mem_req_ready <= 0;
    else     mem_req_ready <= ($urandom(seed)%10) < 8;
  end

  always_ff @(posedge clk) begin
    if (rst) beat_out_ready <= 0;
    else     beat_out_ready <= ($urandom(seed)%10) < 7;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      mem_rvalid <= 0;
      mem_rdata  <= 0;
      for (i=0;i<RD_LAT;i++) begin
        rdq_valid[i] <= 0;
        rdq_addr[i]  <= 0;
      end
    end else begin
      mem_rvalid <= 0;

      // shift pipeline
      for (i=RD_LAT-1;i>0;i--) begin
        rdq_valid[i] <= rdq_valid[i-1];
        rdq_addr[i]  <= rdq_addr[i-1];
      end
      rdq_valid[0] <= 0;

      if (mem_req_valid && mem_req_ready) begin
        if (mem_req_is_wr) begin
          mem[mem_req_addr % MEM_WORDS] <= mem_req_wdata;
        end else begin
          rdq_valid[0] <= 1;
          rdq_addr[0]  <= mem_req_addr;
        end
      end

      if (rdq_valid[RD_LAT-1]) begin
        mem_rvalid <= 1;
        mem_rdata  <= mem[rdq_addr[RD_LAT-1] % MEM_WORDS];
      end
    end
  end

  // ============================================================
  // helpers
  // ============================================================
  function automatic [SRAM_BUS_W-1:0] feat_get_word;
    input [FEAT_W-1:0] feat;
    input int unsigned idx;
    begin
      feat_get_word = feat >> (idx * SRAM_BUS_W);
    end
  endfunction

  // ============================================================
  // request task
  // ============================================================
  task automatic send_req(
    input logic is_wr,
    input logic [MEM_AW-1:0] base,
    input logic [TAG_W-1:0]  tag,
    input logic [FEAT_W-1:0] data
  );
    begin
      req_valid <= 1;
      req_is_wr <= is_wr;
      req_addr  <= base;
      req_tag   <= tag;
      req_wdata <= data;
      do @(posedge clk); while (!req_ready);
      @(posedge clk);
      req_valid <= 0;
    end
  endtask

  // ============================================================
  // scoreboard
  // ============================================================
  logic [FEAT_W-1:0] exp_feat;
  logic [TAG_W-1:0]  exp_tag;
  int unsigned got;

  always_ff @(posedge clk) begin
    if (rst) got <= 0;
    else if (beat_out_valid && beat_out_ready) begin
      if (beat_out_tag !== exp_tag) $fatal;
      if (beat_out_idx !== got[BEAT_IDX_W-1:0]) $fatal;
      if (beat_out_data !== feat_get_word(exp_feat, got)) $fatal;
      got <= got + 1;
    end
  end

  // ============================================================
  // test
  // ============================================================
  initial begin
    rst = 1;
    req_valid = 0;
    got = 0;
    repeat(10) @(posedge clk);
    rst = 0;

    exp_feat = {
      32'h7777_7777,32'h6666_6666,32'h5555_5555,32'h4444_4444,
      32'h3333_3333,32'h2222_2222,32'h1111_1111,32'h0000_0000
    };
    exp_tag = 16'hBEEF;

    send_req(1, 18'h123, exp_tag, exp_feat);
    repeat(200) @(posedge clk);

    got = 0;
    send_req(0, 18'h123, exp_tag, '0);

    while (got < WORDS_PER_FEAT) @(posedge clk);
    $display("PASS ✅");
    $finish;
  end

endmodule
`default_nettype wire
