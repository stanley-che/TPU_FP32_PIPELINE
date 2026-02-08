// ============================================================
// feature_sram_rd_addr_encode.sv  (IVERILOG-SAFE, 1-deep buffer)
// 功能:
// - 接收上游單筆 tile read request (rd_tile_i, rd_tile_j, rd_tag)
// - 轉成 SRAM word address (mem_addr) + (optional) bank (mem_bank)
// - 輸出 mem_beats = ceil(FEAT_W / MEM_W) 作為 burst 長度
// - 內建 1-deep buffer：當 mem_rd_ready=0 時，輸出會 hold 不變
//
// Address mapping (word addressing):
//   linear_idx = rd_tile_i * TILES_X + rd_tile_j
//   base_word  = BASE_ADDR_WORD + linear_idx * WORDS_PER_TILE
//   WORDS_PER_TILE = BEATS (one tile feature consumes BEATS words)
//
// Bank mapping (if BANKS>1, must be power-of-2):
//   mem_bank = base_word[BANK_W-1:0]
//   mem_addr = base_word >> BANK_W
//
// Ready/Valid:
// - Input:  rd_valid/rd_ready_in  (from scheduler)
// - Output: mem_rd_valid/mem_rd_ready (to SRAM/burst reader)
//
// ============================================================

`ifndef FEATURE_SRAM_RD_ADDR_ENCODE_SV
`define FEATURE_SRAM_RD_ADDR_ENCODE_SV

`timescale 1ns/1ps
`default_nettype none

module feature_sram_rd_addr_encode #(
  parameter int unsigned TILES_X        = 320,
  parameter int unsigned TILES_Y        = 180,

  // feature vector width (bits) and SRAM read port width (bits)
  parameter int unsigned FEAT_W         = 256,   // e.g. dims=8 FP32 => 256
  parameter int unsigned MEM_W          = 64,    // SRAM read data width

  // SRAM word address base (in MEM_W words, not bytes)
  parameter int unsigned BASE_ADDR_WORD = 0,

  // Optional banking (interleaving by low bits of base_word)
  parameter int unsigned BANKS          = 1,     // 1 = no bank
  parameter int unsigned TAG_W          = 16
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Upstream request (single tile)
  // ----------------------------
  input  logic                         rd_valid,
  output logic                         rd_ready_in,
  input  logic [$clog2(TILES_Y)-1:0]    rd_tile_i,
  input  logic [$clog2(TILES_X)-1:0]    rd_tile_j,
  input  logic [TAG_W-1:0]             rd_tag,

  // ----------------------------
  // Downstream SRAM read command
  // ----------------------------
  output logic                         mem_rd_valid,
  input  logic                         mem_rd_ready,
  output logic [31:0]                  mem_addr,   // word address (after bank split if BANKS>1)
  output logic [$clog2(BANKS)-1:0]     mem_bank,   // valid only if BANKS>1, else 0
  output logic [TAG_W-1:0]             mem_tag,
  output logic [15:0]                  mem_beats   // burst length in words (fixed)
);

  // ----------------------------
  // guard / widths
  // ----------------------------
  localparam int unsigned IW = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW = (TILES_X <= 1) ? 1 : $clog2(TILES_X);

  // ceil_div(FEAT_W, MEM_W)
  localparam int unsigned BEATS =
    (MEM_W == 0) ? 1 :
    ((FEAT_W + MEM_W - 1) / MEM_W);

  localparam int unsigned WORDS_PER_TILE = (BEATS < 1) ? 1 : BEATS;

  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  wire go = en && !rst;

  // synthesis-time / sim-time sanity checks (IVERILOG ok)
  initial begin
    if (MEM_W == 0) begin
      $display("FATAL: MEM_W must be > 0");
      $fatal(1);
    end
    if (FEAT_W == 0) begin
      $display("FATAL: FEAT_W must be > 0");
      $fatal(1);
    end
    if (BANKS > 1) begin
      // require power-of-2 banks for low-bit interleave
      if ((BANKS & (BANKS-1)) != 0) begin
        $display("FATAL: BANKS must be power-of-2 when BANKS>1 (got %0d)", BANKS);
        $fatal(1);
      end
    end
  end

  // ----------------------------
  // 1-deep buffer (reg slice)
  // ----------------------------
  logic             full;
  logic [IW-1:0]     bi;
  logic [JW-1:0]     bj;
  logic [TAG_W-1:0]  btag;

  // accept new input if buffer not full OR we are popping this cycle
  // (overwrite-on-pop style)
  wire pop  = go && full && mem_rd_valid && mem_rd_ready;
  always @* begin
    if (!go) rd_ready_in = 1'b0;
    else     rd_ready_in = (!full) || pop;
  end

  wire push = go && rd_valid && rd_ready_in;

  // update buffer
  always_ff @(posedge clk) begin
    if (rst) begin
      full <= 1'b0;
      bi   <= '0;
      bj   <= '0;
      btag <= '0;
    end else if (en) begin
      // default: keep state
      if (pop && !push) begin
        full <= 1'b0;
      end

      if (push) begin
        full <= 1'b1;
        bi   <= rd_tile_i;
        bj   <= rd_tile_j;
        btag <= rd_tag;
      end
    end
  end

  // ----------------------------
  // Address encode from buffered request
  // ----------------------------
  // linear_idx = bi*TILES_X + bj
  // base_word  = BASE_ADDR_WORD + linear_idx*WORDS_PER_TILE
  logic [63:0] linear_idx;
  logic [63:0] base_word;

  always @* begin
    linear_idx = 64'(bi) * 64'(TILES_X) + 64'(bj);
    base_word  = 64'(BASE_ADDR_WORD) + linear_idx * 64'(WORDS_PER_TILE);
  end

  // bank split (low-bit interleave)
  logic [63:0] base_word_shifted;

  always @* begin
    if (BANKS <= 1) begin
      mem_bank = '0;
      mem_addr = base_word[31:0];
    end else begin
      mem_bank          = base_word[BANK_W-1:0];
      base_word_shifted = (base_word >> BANK_W);
      mem_addr          = base_word_shifted[31:0];
    end
  end


  // ----------------------------
  // Outputs
  // ----------------------------
  always @* begin
    if (!go) begin
      mem_rd_valid = 1'b0;
      mem_tag      = '0;
      mem_beats    = 16'(0);
    end else begin
      mem_rd_valid = full;
      mem_tag      = btag;
      mem_beats    = 16'(BEATS);
    end
  end

endmodule

`default_nettype wire
`endif
