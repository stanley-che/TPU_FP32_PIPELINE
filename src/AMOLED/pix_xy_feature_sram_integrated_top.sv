`ifndef PIX_XY_FEATURE_SRAM_INTEGRATED_TOP_SV
`define PIX_XY_FEATURE_SRAM_INTEGRATED_TOP_SV

`include "./src/AMOLED/tile feature_extracter/pix_xy_feature_top.sv"
`include "./src/AMOLED/feature_sram/feature_sram.sv"

`timescale 1ns/1ps
`default_nettype none

// ============================================================
// 1-deep skid buffer (NO DROP, ready/valid compliant)
// - upstream must hold in_data stable while in_valid=1 and in_ready=0
// ============================================================
module rv_skid_1_nodrop #(
  parameter int unsigned DW = 32
)(
  input  logic           clk,
  input  logic           rst,
  input  logic           en,

  input  logic           in_valid,
  output logic           in_ready,
  input  logic [DW-1:0]  in_data,

  output logic           out_valid,
  input  logic           out_ready,
  output logic [DW-1:0]  out_data
);
  logic          full;
  logic [DW-1:0] buf_r;

  always_ff @(posedge clk) begin
    if (rst) begin
      full  <= 1'b0;
      buf_r <= '0;
    end else if (en) begin
      // pop
      if (full && out_ready) begin
        full <= 1'b0;
      end

      // push (capture one beat when downstream stalls)
      if (!full) begin
        if (in_valid && !out_ready) begin
          full  <= 1'b1;
          buf_r <= in_data;
        end
      end
    end
  end

  always_comb begin
    if (!en) begin
      in_ready  = 1'b0;
      out_valid = 1'b0;
      out_data  = '0;
    end else if (full) begin
      out_valid = 1'b1;
      out_data  = buf_r;
      in_ready  = 1'b0;      // MUST backpressure upstream
    end else begin
      out_valid = in_valid;
      out_data  = in_data;
      in_ready  = out_ready; // pass-through when not buffering
    end
  end
endmodule


// ------------------------------------------------------------
// pix_xy_feature_sram_integrated_top
// - Adds: event pending alignment + no-drop skid for (sof/eol/eof/y)
// ------------------------------------------------------------
module pix_xy_feature_sram_integrated_top #(
  // -----------------------------
  // video / coordinate
  // -----------------------------
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  parameter int unsigned TILE_SHIFT = 2,
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  parameter int unsigned YPIX_W = 8,

  parameter bit USE_SOF = 1'b1,
  parameter bit USE_EOL = 1'b0,
  parameter bit USE_EOF = 1'b0,
  parameter bit SAT_AT_MAX = 1'b0,
  parameter bit ADVANCE_ON_VALID_ONLY = 1'b1,

  parameter bit          ROI_EN_DEFAULT  = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT  = 0,
  parameter int unsigned ROI_Y0_DEFAULT  = 0,
  parameter int unsigned ROI_X1_DEFAULT  = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT  = ACTIVE_H-1,

  parameter bit DBG_COUNTERS_EN = 1'b1,

  // -----------------------------
  // tile_stats_edge knobs
  // -----------------------------
  parameter int unsigned TILE_W = 4,
  parameter int unsigned TILE_H = 4,

  parameter bit SUPPORT_PIX_VALID_STATS = 1'b1,
  parameter bit DO_SUMSQ_STATS          = 1'b0,
  parameter bit ASSERT_ON_STATS         = 1'b1,

  parameter bit SUPPORT_PIX_VALID_EDGE  = 1'b1,
  parameter bit SUPPORT_TILE_START_EDGE = 1'b1,
  parameter bit ASSERT_ON_EDGE          = 1'b0,

  parameter int unsigned EDGE_MODE = 0,
  parameter int unsigned EDGE_THR  = 8,
  parameter int unsigned EDGE_W    = 32,
  parameter int unsigned CNT_W     = 8,

  // -----------------------------
  // feature pack knobs
  // -----------------------------
  parameter int unsigned FEAT_W   = 16,
  parameter int unsigned FEAT_DIM = 8,

  parameter bit EDGE_USE_MEAN = 1'b1,

  parameter int unsigned TILE_PIX_SHIFT  = 4,
  parameter int unsigned TILE_PIX_N      = 16,
  parameter bit          MEAN_MODE_SHIFT = 1'b1,

  parameter bit MATCH_MODE_BUFFERED = 1'b0,
  parameter bit ZERO_WHEN_INVALID   = 1'b0,

  parameter int unsigned FEAT0_SHIFT = 0,
  parameter int unsigned FEAT1_SHIFT = 0,
  parameter int unsigned FEAT2_SHIFT = 0,
  parameter int unsigned FEAT3_SHIFT = 0,
  parameter int signed   FEAT0_BIAS  = 0,
  parameter int signed   FEAT1_BIAS  = 0,
  parameter int signed   FEAT2_BIAS  = 0,
  parameter int signed   FEAT3_BIAS  = 0,
  parameter int unsigned FEAT_CLAMP_MAX = (1<<FEAT_W)-1,

  // meta widths
  parameter int unsigned TILE_I_W  = 16,
  parameter int unsigned TILE_J_W  = 16,
  parameter int unsigned TILE_ID_W = (TILES_X*TILES_Y <= 1) ? 1 : $clog2(TILES_X*TILES_Y),

  parameter bit ENABLE_CNTS = 1'b1,

  // -----------------------------
  // feature_sram
  // -----------------------------
  parameter int unsigned sram_bus = 32,
  parameter int unsigned elen_W   = 32,
  parameter int unsigned tag_w    = 16,
  parameter bit          isclamp  = 1'b0,

  parameter int unsigned REQ_DEPTH     = 2,
  parameter int unsigned META_DEPTH    = 16,
  parameter int unsigned BEAT_DEPTH    = 16,
  parameter int unsigned RD_FIFO_DEPTH = 2,

  parameter int unsigned RD_LAT    = 2
)(
  input  logic clk,
  input  logic rst,

  // pixel stream in
  input  logic              en,
  input  logic              pix_valid,
  output logic              pix_ready,
  input  logic              sof,
  input  logic              eol,
  input  logic              eof,
  input  logic [YPIX_W-1:0] y_in,

  // external READ port
  input  logic                       valid_rd,
  output logic                       ready_rd,
  input  logic [$clog2(TILES_Y)-1:0]  tile_i_rd,
  input  logic [$clog2(TILES_X)-1:0]  tile_j_rd,
  input  logic [tag_w-1:0]           tag_rd,

  // feature OUT from SRAM
  output logic                       feat_out_valid,
  input  logic                       feat_out_ready,
  output logic [tag_w-1:0]           feat_out_tag,
  output logic [FEAT_DIM*elen_W-1:0] feat_out_data,

  // write-side activity
  output logic                       wr_fire,
  output logic [TILE_I_W-1:0]        wr_tile_i,
  output logic [TILE_J_W-1:0]        wr_tile_j,

  // packer debug
  output logic                       err_mismatch_pulse,
  output logic [31:0]                cnt_join_ok,
  output logic [31:0]                cnt_mismatch,
  output logic [31:0]                cnt_drop
);

  // ============================================================
  // (NEW) event pending + skid on input pixel stream
  // ============================================================
  logic pend_sof, pend_eol, pend_eof;

  // in_ready of skid (this is what we expose as pix_ready)
  logic in_ready_s;

  // fire at module input boundary
  wire in_fire = pix_valid && in_ready_s;

  always_ff @(posedge clk) begin
    if (rst) begin
      pend_sof <= 1'b0;
      pend_eol <= 1'b0;
      pend_eof <= 1'b0;
    end else if (en) begin
      // accumulate pulses even if pix_valid=0
      pend_sof <= pend_sof | sof;
      pend_eol <= pend_eol | eol;
      pend_eof <= pend_eof | eof;

      // consume pending on the next accepted pixel
      if (in_fire) begin
        pend_sof <= 1'b0;
        pend_eol <= 1'b0;
        pend_eof <= 1'b0;
      end
    end
  end

  // align events into the beat that actually enters the pipeline
  wire sof_a = sof | pend_sof;
  wire eol_a = eol | pend_eol;
  wire eof_a = eof | pend_eof;

  localparam int unsigned IN_DW = (3 + YPIX_W); // {sof,eol,eof,y}
  wire [IN_DW-1:0] in_pack = {sof_a, eol_a, eof_a, y_in};

  wire              pix_v_s;
  wire              pix_r_s;
  wire [IN_DW-1:0]  in_pack_s;

  rv_skid_1_nodrop #(.DW(IN_DW)) u_in_skid (
    .clk(clk),
    .rst(rst),
    .en(en),

    .in_valid(pix_valid),
    .in_ready(in_ready_s),
    .in_data(in_pack),

    .out_valid(pix_v_s),
    .out_ready(pix_r_s),
    .out_data(in_pack_s)
  );

  // expose ready to upstream
  assign pix_ready = in_ready_s;

  // unpack to feature_top side
  wire sof_s;
  wire eol_s;
  wire eof_s;
  wire [YPIX_W-1:0] y_s;
  assign {sof_s, eol_s, eof_s, y_s} = in_pack_s;

  // ============================================================
  // A) pix_xy_feature_top
  // ============================================================
  logic                 feat_valid;
  logic                 feat_ready;

  logic [TILE_I_W-1:0]  tile_i_o;
  logic [TILE_J_W-1:0]  tile_j_o;
  logic [TILE_ID_W-1:0] tile_id_o;

  logic [8*FEAT_W-1:0]  feat_vec_8;

  pix_xy_feature_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .YPIX_W(YPIX_W),

    .USE_SOF(USE_SOF),
    .USE_EOL(USE_EOL),
    .USE_EOF(USE_EOF),
    .SAT_AT_MAX(SAT_AT_MAX),
    .ADVANCE_ON_VALID_ONLY(ADVANCE_ON_VALID_ONLY),

    .ROI_EN_DEFAULT(ROI_EN_DEFAULT),
    .ROI_X0_DEFAULT(ROI_X0_DEFAULT),
    .ROI_Y0_DEFAULT(ROI_Y0_DEFAULT),
    .ROI_X1_DEFAULT(ROI_X1_DEFAULT),
    .ROI_Y1_DEFAULT(ROI_Y1_DEFAULT),

    .DBG_COUNTERS_EN(DBG_COUNTERS_EN),

    .TILE_W(TILE_W), .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID_STATS(SUPPORT_PIX_VALID_STATS),
    .DO_SUMSQ_STATS(DO_SUMSQ_STATS),
    .ASSERT_ON_STATS(ASSERT_ON_STATS),

    .SUPPORT_PIX_VALID_EDGE(SUPPORT_PIX_VALID_EDGE),
    .SUPPORT_TILE_START_EDGE(SUPPORT_TILE_START_EDGE),
    .ASSERT_ON_EDGE(ASSERT_ON_EDGE),

    .EDGE_MODE(EDGE_MODE),
    .EDGE_THR(EDGE_THR),
    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W),

    .FEAT_W(FEAT_W),
    .FEAT_DIM(FEAT_DIM),

    .EDGE_USE_MEAN(EDGE_USE_MEAN),
    .TILE_PIX_SHIFT(TILE_PIX_SHIFT),
    .TILE_PIX_N(TILE_PIX_N),
    .MEAN_MODE_SHIFT(MEAN_MODE_SHIFT),

    .MATCH_MODE_BUFFERED(MATCH_MODE_BUFFERED),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),

    .FEAT0_SHIFT(FEAT0_SHIFT),
    .FEAT1_SHIFT(FEAT1_SHIFT),
    .FEAT2_SHIFT(FEAT2_SHIFT),
    .FEAT3_SHIFT(FEAT3_SHIFT),
    .FEAT0_BIAS(FEAT0_BIAS),
    .FEAT1_BIAS(FEAT1_BIAS),
    .FEAT2_BIAS(FEAT2_BIAS),
    .FEAT3_BIAS(FEAT3_BIAS),
    .FEAT_CLAMP_MAX(FEAT_CLAMP_MAX),

    .TILE_I_W(TILE_I_W),
    .TILE_J_W(TILE_J_W),
    .TILE_ID_W(TILE_ID_W),

    .ENABLE_CNTS(ENABLE_CNTS)
  ) u_pix2feat (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_v_s),
    .pix_ready(pix_r_s),
    .sof(sof_s),
    .eol(eol_s),
    .eof(eof_s),
    .y_in(y_s),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),

    .tile_i_o(tile_i_o),
    .tile_j_o(tile_j_o),
    .tile_id_o(tile_id_o),

    .feat0(), .feat1(), .feat2(), .feat3(),
    .feat4(), .feat5(), .feat6(), .feat7(),
    .feat_vec(feat_vec_8),

    .stats_tile_err(),
    .stats_err_code(),
    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop),

    .x(), .y(),
    .tile_j_dbg(), .tile_i_dbg(),
    .x_mod(), .y_mod(),
    .tile_first(), .tile_last(),
    .in_roi(),
    .err_sof_midframe(),
    .err_eol_mismatch(),
    .frame_cnt(),
    .line_cnt()
  );

  // ============================================================
  // Convert feat_vec -> SRAM write data (zero-extend FEAT_W -> elen_W)
  // ============================================================
  localparam int unsigned FEAT_SRAM_W = FEAT_DIM * elen_W;
  logic [FEAT_SRAM_W-1:0] feat_wr_data;

  genvar d;
  generate
    for (d = 0; d < FEAT_DIM; d++) begin : G_EXT
      wire [FEAT_W-1:0] lane = feat_vec_8[d*FEAT_W +: FEAT_W];
      assign feat_wr_data[d*elen_W +: elen_W] = {{(elen_W-FEAT_W){1'b0}}, lane};
    end
  endgenerate

  // ============================================================
  // B) feature_sram
  // ============================================================
  logic                       valid_wr_s, ready_wr_s;
  logic [$clog2(TILES_Y)-1:0] tile_i_wr_s;
  logic [$clog2(TILES_X)-1:0] tile_j_wr_s;

  assign valid_wr_s = feat_valid;
  assign feat_ready = ready_wr_s;

  assign tile_i_wr_s = tile_i_o[$clog2(TILES_Y)-1:0];
  assign tile_j_wr_s = tile_j_o[$clog2(TILES_X)-1:0];

  feature_sram #(
    .tile_x(TILES_X),
    .tile_y(TILES_Y),
    .sram_bus(sram_bus),
    .feat_dim(FEAT_DIM),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp),

    .REQ_DEPTH(REQ_DEPTH),
    .META_DEPTH(META_DEPTH),
    .BEAT_DEPTH(BEAT_DEPTH),
    .RD_FIFO_DEPTH(RD_FIFO_DEPTH),

    .RD_LAT(RD_LAT)
  ) u_feat_sram (
    .clk(clk),
    .rst(rst),

    .valid_wr(valid_wr_s),
    .ready_wr(ready_wr_s),
    .tile_i_wr(tile_i_wr_s),
    .tile_j_wr(tile_j_wr_s),
    .feat_in(feat_wr_data),

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

  assign wr_fire   = valid_wr_s & ready_wr_s;
  assign wr_tile_i = tile_i_o;
  assign wr_tile_j = tile_j_o;

endmodule

`default_nettype wire
`endif
