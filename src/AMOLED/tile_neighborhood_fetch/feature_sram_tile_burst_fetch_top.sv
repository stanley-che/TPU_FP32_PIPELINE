// ============================================================
// feature_sram_tile_burst_fetch_top.sv  (INSTANCE INTEGRATION)
// - Instantiates:
//   * feature_sram_rd_addr_encode  (tile -> base full-word address + beats + tag, 1-deep buffer)
//   * feature_sram_burst_reader    (issues per-beat reads, packs feature vector)
//
// Banking strategy:
// - encoder is instantiated with BANKS=1 to output "full word address" (no split)
// - burst_reader increments full word addr by +1 each beat
// - top splits full word addr into mem_bank + mem_addr when BANKS>1
// ============================================================

`ifndef FEATURE_SRAM_TILE_BURST_FETCH_TOP_SV
`define FEATURE_SRAM_TILE_BURST_FETCH_TOP_SV
`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_rd_addr_encode.sv"
`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_burst_reader.sv"
`timescale 1ns/1ps
`default_nettype none

module feature_sram_tile_burst_fetch_top #(
  parameter int unsigned TILES_X        = 320,
  parameter int unsigned TILES_Y        = 180,
  parameter int unsigned FEAT_W         = 256,
  parameter int unsigned MEM_W          = 64,
  parameter int unsigned BASE_ADDR_WORD = 0,

  // final output banking to SRAM
  parameter int unsigned BANKS          = 1,     // 1=no bank, >1 power-of-2
  parameter int unsigned TAG_W          = 16,
  parameter int unsigned ADDR_W         = 32,
  parameter int unsigned READ_LATENCY   = 1,
  parameter bit          USE_RTAG       = 0
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Upstream tile request
  // ----------------------------
  input  logic                         rd_valid,
  output logic                         rd_ready_in,
  input  logic [$clog2(TILES_Y)-1:0]    rd_tile_i,
  input  logic [$clog2(TILES_X)-1:0]    rd_tile_j,
  input  logic [TAG_W-1:0]             rd_tag,

  // ----------------------------
  // Downstream SRAM read command (per beat)
  // ----------------------------
  output logic                         mem_rd_valid,
  input  logic                         mem_rd_ready,
  output logic [ADDR_W-1:0]            mem_addr,   // bank-split addr (if BANKS>1)
  output logic [((BANKS<=1)?1:$clog2(BANKS))-1:0] mem_bank,
  output logic [TAG_W-1:0]             mem_tag,

  // ----------------------------
  // SRAM read return
  // ----------------------------
  input  logic                         mem_rvalid,
  input  logic [MEM_W-1:0]             mem_rdata,
  input  logic [TAG_W-1:0]             mem_rtag,

  // ----------------------------
  // Packed feature output
  // ----------------------------
  output logic                         feat_valid,
  input  logic                         feat_ready,
  output logic [FEAT_W-1:0]            feat_vec,
  output logic [TAG_W-1:0]             feat_tag
);

  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  // ============================================================
  // (1) encoder outputs BASE FULL WORD address (no bank split)
  // ============================================================
  wire              enc_cmd_valid;
  wire              enc_cmd_ready;
  wire [31:0]       enc_base_full_word; // full word addr (includes bank bits logically)
  wire [TAG_W-1:0]  enc_cmd_tag;
  wire [15:0]       enc_cmd_beats;

  // We force BANKS=1 here so encoder does NOT split bank/address.
  // NOTE: encoder module must have mem_bank width safe when BANKS=1.
  feature_sram_rd_addr_encode #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS(1),                // <--- IMPORTANT: no split here
    .TAG_W(TAG_W)
  ) u_enc (
    .clk(clk),
    .rst(rst),
    .en(en),

    .rd_valid(rd_valid),
    .rd_ready_in(rd_ready_in),
    .rd_tile_i(rd_tile_i),
    .rd_tile_j(rd_tile_j),
    .rd_tag(rd_tag),

    // treat encoder "mem_rd_*" as burst_reader "cmd_*"
    .mem_rd_valid(enc_cmd_valid),
    .mem_rd_ready(enc_cmd_ready),
    .mem_addr(enc_base_full_word),
    .mem_bank(),              // unused (BANKS=1 anyway)
    .mem_tag(enc_cmd_tag),
    .mem_beats(enc_cmd_beats)
  );

  // ============================================================
  // (2) burst_reader issues per-beat FULL WORD address
  // ============================================================
  wire              br_mem_rd_valid_full;
  wire [ADDR_W-1:0] br_full_word_addr;   // we treat as full word address
  wire [TAG_W-1:0]  br_mem_tag_full;

  feature_sram_burst_reader #(
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .TAG_W(TAG_W),
    .ADDR_W(ADDR_W),
    .READ_LATENCY(READ_LATENCY),
    .USE_RTAG(USE_RTAG)
  ) u_br (
    .clk(clk),
    .rst(rst),
    .en(en),

    // cmd from encoder
    .cmd_valid(enc_cmd_valid),
    .cmd_ready(enc_cmd_ready),
    .cmd_addr(enc_base_full_word[ADDR_W-1:0]),
    .cmd_beats(enc_cmd_beats),
    .cmd_tag(enc_cmd_tag),

    // per-beat read out (full word address)
    .mem_rd_valid(br_mem_rd_valid_full),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr(br_full_word_addr),
    .mem_tag(br_mem_tag_full),

    // SRAM return
    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),
    .mem_rtag(mem_rtag),

    // packed out
    .feat_valid(feat_valid),
    .feat_ready(feat_ready),
    .feat_vec(feat_vec),
    .feat_tag(feat_tag)
  );

  // ============================================================
  // (3) Top-level bank split for SRAM interface
  // ============================================================
  always @* begin
    mem_rd_valid = br_mem_rd_valid_full;
    mem_tag      = br_mem_tag_full;

    if (BANKS <= 1) begin
      mem_bank = '0;
      mem_addr = br_full_word_addr;
    end else begin
      mem_bank = br_full_word_addr[BANK_W-1:0];
      mem_addr = (br_full_word_addr >> BANK_W);
    end
  end

  // sanity checks
  initial begin
    if (BANKS > 1) begin
      if ((BANKS & (BANKS-1)) != 0) begin
        $display("FATAL: BANKS must be power-of-2 when BANKS>1 (got %0d)", BANKS);
        $fatal(1);
      end
    end
  end

endmodule

`default_nettype wire
`endif
