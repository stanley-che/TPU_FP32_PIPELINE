// ============================================================
// tile_nb_feature_fetch_3x3_flat_top.sv  (SINGLE TOP, ALL INSTANCE)
// - Single integrated top that instantiates (ONLY via instances):
//   * tile_nb_fetch_3x3_to_9req_inst
//   * feature_sram_rd_addr_encode      (BANKS forced to 1 => outputs FULL-WORD base addr)
//   * feature_sram_burst_reader        (issues per-beat FULL-WORD addr, packs feat_vec)
//
// - This replaces the extra wrapper layer (feature_sram_tile_burst_fetch_top),
//   while preserving the same external behavior as tile_nb_feature_fetch_3x3_top_inst.
//
// Banking strategy:
// - Encoder outputs "full word address" (no split) by forcing BANKS=1
// - Burst reader increments full-word address by +1 each beat
// - Top splits full-word address into {mem_bank, mem_addr} when BANKS>1
// ============================================================

`ifndef TILE_NB_FEATURE_FETCH_3X3_TOP_INST_SV
`define TILE_NB_FEATURE_FETCH_3X3_TOP_INST_SV

`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_fetch_3x3_to_9req_inst.sv"
`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_rd_addr_encode.sv"
`include "./src/AMOLED/tile_neighborhood_fetch/feature_sram_burst_reader.sv"

`timescale 1ns/1ps
`default_nettype none

module tile_nb_feature_fetch_3x3_top_inst #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,

  parameter int unsigned TAG_W   = 16,
  parameter int unsigned IDX_W   = 4,

  parameter int unsigned FEAT_W         = 256,
  parameter int unsigned MEM_W          = 64,
  parameter int unsigned BASE_ADDR_WORD = 0,

  // final output banking to SRAM
  parameter int unsigned BANKS          = 1,     // 1=no bank, >1 power-of-2
  parameter int unsigned ADDR_W         = 32,
  parameter int unsigned READ_LATENCY   = 1,
  parameter bit          USE_RTAG       = 0
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Center tile request
  // ----------------------------
  input  logic                         center_valid,
  output logic                         center_ready,
  input  logic [$clog2(TILES_Y)-1:0]    center_i,
  input  logic [$clog2(TILES_X)-1:0]    center_j,
  input  logic [TAG_W-1:0]             center_tag,

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
  // Packed feature output (1 vector per tile request)
  // ----------------------------
  output logic                         feat_valid,
  input  logic                         feat_ready,
  output logic [FEAT_W-1:0]            feat_vec,
  output logic [TAG_W-1:0]             feat_tag,

  // ----------------------------
  // Scheduler status
  // ----------------------------
  output logic                         sched_busy,
  output logic                         sched_done_pulse,   // pulse when 9th FEATURE is CONSUMED
  output logic [8:0]                   nb_is_center
);

  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);

  // ------------------------------------------------------------
  // A) Scheduler -> emits 9 tile requests (row-major, clamp)
  // ------------------------------------------------------------
  logic                       rd_valid_w;
  logic                       rd_ready_w;
  logic [$clog2(TILES_Y)-1:0]  rd_tile_i_w;
  logic [$clog2(TILES_X)-1:0]  rd_tile_j_w;
  logic [TAG_W-1:0]            rd_tag_w;

  wire sched_done_pulse_rd; // original "9 req issued" pulse (internal only)

  tile_nb_fetch_3x3_to_9req_inst #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W  (TAG_W),
    .IDX_W  (IDX_W)
  ) u_nb_3x3_to_9req (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i    (center_i),
    .center_j    (center_j),
    .center_tag  (center_tag),

    .rd_valid (rd_valid_w),
    .rd_ready (rd_ready_w),
    .rd_tile_i(rd_tile_i_w),
    .rd_tile_j(rd_tile_j_w),
    .rd_tag   (rd_tag_w),

    .sched_busy      (sched_busy),
    .sched_done_pulse(sched_done_pulse_rd),
    .nb_is_center    (nb_is_center)
  );

  // ------------------------------------------------------------
  // B) Encoder (force BANKS=1) -> outputs FULL-WORD base addr + beats + tag
  // ------------------------------------------------------------
  wire             enc_cmd_valid;
  wire             enc_cmd_ready;
  wire [31:0]      enc_base_full_word;
  wire [TAG_W-1:0] enc_cmd_tag;
  wire [15:0]      enc_cmd_beats;

  feature_sram_rd_addr_encode #(
    .TILES_X       (TILES_X),
    .TILES_Y       (TILES_Y),
    .FEAT_W        (FEAT_W),
    .MEM_W         (MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS         (1),      // IMPORTANT: no split here, outputs FULL-WORD addr
    .TAG_W         (TAG_W)
  ) u_enc (
    .clk(clk),
    .rst(rst),
    .en (en),

    .rd_valid   (rd_valid_w),
    .rd_ready_in(rd_ready_w),
    .rd_tile_i  (rd_tile_i_w),
    .rd_tile_j  (rd_tile_j_w),
    .rd_tag     (rd_tag_w),

    .mem_rd_valid(enc_cmd_valid),
    .mem_rd_ready(enc_cmd_ready),
    .mem_addr    (enc_base_full_word),
    .mem_bank    (),               // unused (BANKS=1)
    .mem_tag     (enc_cmd_tag),
    .mem_beats   (enc_cmd_beats)
  );

  // ------------------------------------------------------------
  // C) Burst reader -> issues per-beat FULL-WORD addr, packs feature vector
  // ------------------------------------------------------------
  wire              br_mem_rd_valid_full;
  wire [ADDR_W-1:0] br_full_word_addr;
  wire [TAG_W-1:0]  br_mem_tag_full;

  feature_sram_burst_reader #(
    .FEAT_W       (FEAT_W),
    .MEM_W        (MEM_W),
    .TAG_W        (TAG_W),
    .ADDR_W       (ADDR_W),
    .READ_LATENCY (READ_LATENCY),
    .USE_RTAG     (USE_RTAG)
  ) u_br (
    .clk(clk),
    .rst(rst),
    .en (en),

    .cmd_valid (enc_cmd_valid),
    .cmd_ready (enc_cmd_ready),
    .cmd_addr  (enc_base_full_word[ADDR_W-1:0]),
    .cmd_beats (enc_cmd_beats),
    .cmd_tag   (enc_cmd_tag),

    .mem_rd_valid(br_mem_rd_valid_full),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr   (br_full_word_addr),
    .mem_tag    (br_mem_tag_full),

    .mem_rvalid(mem_rvalid),
    .mem_rdata (mem_rdata),
    .mem_rtag  (mem_rtag),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),
    .feat_vec  (feat_vec),
    .feat_tag  (feat_tag)
  );

  // ------------------------------------------------------------
  // D) Bank split at top-level SRAM interface
  // ------------------------------------------------------------
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

    // ------------------------------------------------------------
  // E) End-to-end done pulse: 9th FEATURE is CONSUMED (feat_fire)
  // - IMPORTANT: sched_busy may drop right after 9 reqs issued,
  //   but feature returns can happen later. So we must NOT gate
  //   done pulse with sched_busy.
  // - Use e2e_busy latched from center acceptance until 9th consume.
  // ------------------------------------------------------------
  logic       e2e_busy;
  logic [3:0] feat_cnt;

  wire go          = en && !rst;
  wire center_fire = go && center_valid && center_ready;
  wire feat_fire   = go && feat_valid && feat_ready;

  always_ff @(posedge clk) begin
    if (rst) begin
      e2e_busy <= 1'b0;
      feat_cnt <= 4'd0;
    end else if (en) begin
      // start a transaction when center is accepted
      if (center_fire) begin
        e2e_busy <= 1'b1;
        feat_cnt <= 4'd0;
      end else if (e2e_busy && feat_fire) begin
        // count consumed features 0..8
        if (feat_cnt == 4'd8) begin
          feat_cnt <= 4'd0;
          e2e_busy <= 1'b0; // transaction complete
        end else begin
          feat_cnt <= feat_cnt + 4'd1;
        end
      end
    end
  end

  always @* begin
    if (!go) sched_done_pulse = 1'b0;
    else     sched_done_pulse = (e2e_busy && feat_fire && (feat_cnt == 4'd8));
  end


  // ------------------------------------------------------------
  // F) Sanity: BANKS power-of-2 when >1
  // ------------------------------------------------------------
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
