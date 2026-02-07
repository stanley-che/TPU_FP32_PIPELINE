// pix_xy_feature_top.sv
// ------------------------------------------------------------
// Integrates:
//   A) pix_xy_linebuf_stats_edge_top : pixel->tile stats+edge bundle (out_valid/out_ready)
//   B) feature_pack_and_valid        : stats+edge -> feature vector (feat_valid/feat_ready)
//
// Fix:
// - Insert 1-deep skid FIFO between A and B to break timing/backpressure coupling
// - Packer consumes FIFO-Q payloads (NOT raw bundle wires)
// ------------------------------------------------------------

`ifndef PIX_XY_FEATURE_TOP_SV
`define PIX_XY_FEATURE_TOP_SV

`include "./src/AMOLED/tile feature_extracter/pix_xy_linebuf_stats_edge_top.sv"
`include "./src/AMOLED/tile feature_extracter/feature_pack_and_valid.sv"

`timescale 1ns/1ps
`default_nettype none

module pix_xy_feature_top #(
  // -----------------------------
  // video / coordinate
  // -----------------------------
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  // Tile config (4x4 default)
  parameter int unsigned TILE_SHIFT = 2,
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  // pixel value width
  parameter int unsigned YPIX_W = 8,

  // pixel_xy_counter behavior
  parameter bit USE_SOF = 1'b1,
  parameter bit USE_EOL = 1'b0,
  parameter bit USE_EOF = 1'b0,
  parameter bit SAT_AT_MAX = 1'b0,
  parameter bit ADVANCE_ON_VALID_ONLY = 1'b1,

  // ROI defaults
  parameter bit          ROI_EN_DEFAULT  = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT  = 0,
  parameter int unsigned ROI_Y0_DEFAULT  = 0,
  parameter int unsigned ROI_X1_DEFAULT  = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT  = ACTIVE_H-1,

  parameter bit DBG_COUNTERS_EN = 1'b1,

  // -----------------------------
  // tile_stats_edge_4x4 knobs
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

  // ---- derived widths for stats output ports ----
  parameter int unsigned TILE_PIXELS = TILE_W*TILE_H,
  parameter int unsigned SUM_MAX     = TILE_PIXELS * ((1<<YPIX_W) - 1),
  parameter int unsigned SUM_W       = (SUM_MAX <= 1) ? 1 : $clog2(SUM_MAX + 1),

  parameter int unsigned SUMSQ_MAX   = TILE_PIXELS * (((1<<YPIX_W)-1) * ((1<<YPIX_W)-1)),
  parameter int unsigned SUMSQ_W     = (SUMSQ_MAX <= 1) ? 1 : $clog2(SUMSQ_MAX + 1),

  // -----------------------------
  // feature pack knobs
  // -----------------------------
  parameter int unsigned FEAT_W   = 16,
  parameter int unsigned FEAT_DIM = 4,

  parameter bit EDGE_USE_MEAN = 1'b1,

  // mean config (tile=4x4 => shift=4)
  parameter int unsigned TILE_PIX_SHIFT  = 4,
  parameter int unsigned TILE_PIX_N      = 16,
  parameter bit          MEAN_MODE_SHIFT = 1'b1,

  // join/matching mode inside packer
  parameter bit MATCH_MODE_BUFFERED = 1'b0,

  parameter bit ZERO_WHEN_INVALID = 1'b0,

  // post-process
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

  parameter bit ENABLE_CNTS = 1'b1
)(
  input  logic                   clk,
  input  logic                   rst,
  input  logic                   en,

  // -----------------------------
  // upstream pixel/control stream
  // -----------------------------
  input  logic                   pix_valid,
  output logic                   pix_ready,
  input  logic                   sof,
  input  logic                   eol,
  input  logic                   eof,
  input  logic [YPIX_W-1:0]      y_in,

  // -----------------------------
  // downstream feature stream
  // -----------------------------
  output logic                   feat_valid,
  input  logic                   feat_ready,

  output logic [TILE_I_W-1:0]    tile_i_o,
  output logic [TILE_J_W-1:0]    tile_j_o,
  output logic [TILE_ID_W-1:0]   tile_id_o,

  output logic [FEAT_W-1:0]      feat0,
  output logic [FEAT_W-1:0]      feat1,
  output logic [FEAT_W-1:0]      feat2,
  output logic [FEAT_W-1:0]      feat3,
  output logic [FEAT_W-1:0]      feat4,
  output logic [FEAT_W-1:0]      feat5,
  output logic [FEAT_W-1:0]      feat6,
  output logic [FEAT_W-1:0]      feat7,
  output logic [8*FEAT_W-1:0]    feat_vec,

  // -----------------------------
  // optional debug/errors (export)
  // -----------------------------
  output logic                   stats_tile_err,
  output logic [7:0]             stats_err_code,
  output logic                   err_mismatch_pulse,
  output logic [31:0]            cnt_join_ok,
  output logic [31:0]            cnt_mismatch,
  output logic [31:0]            cnt_drop,

  // coordinates/meta passthrough if you want
  output logic [X_W-1:0]         x,
  output logic [Y_W-1:0]         y,
  output logic [X_W-1:0]         tile_j_dbg,
  output logic [Y_W-1:0]         tile_i_dbg,
  output logic [1:0]             x_mod,
  output logic [1:0]             y_mod,
  output logic                   tile_first,
  output logic                   tile_last,
  output logic                   in_roi,
  output logic                   err_sof_midframe,
  output logic                   err_eol_mismatch,
  output logic [31:0]            frame_cnt,
  output logic [31:0]            line_cnt
);

  // ============================================================
  // Stage A outputs (raw bundle)
  // ============================================================
  logic                 v_bundle_valid;
  logic                 v_bundle_ready;

  logic [SUM_W-1:0]     sumY;
  logic [YPIX_W-1:0]    minY, maxY, meanY, rangeY;

  logic [EDGE_W-1:0]    edge_sum, edge_max, edge_mean;
  logic [CNT_W-1:0]     edge_cnt;

  logic [15:0]          y_meta_o_s;
  logic [15:0]          tile_x_o_s, tile_y_o_s;
  logic [15:0]          frame_id_o_s;

  logic [$clog2(TILE_W*TILE_H+1)-1:0] stats_pix_cnt_o;
 
    // ============================================================
  // 2-entry FIFO (burst=2) for paired bundle (atomic consume)
  //  - Stage A already pairs stats+edge in the same out_valid beat
  //  - Present v3/v4 together; pop only when BOTH accepted same cycle
  // ============================================================

  localparam int unsigned FIFO_DEPTH = 2;

  logic [$clog2(FIFO_DEPTH):0] fifo_cnt; // can hold 0..2
  logic [$clog2(FIFO_DEPTH)-1:0] rd_ptr, wr_ptr;

  wire fifo_full  = (fifo_cnt == FIFO_DEPTH);
  wire fifo_empty = (fifo_cnt == 0);

  // Upstream (Stage A) handshake
  assign v_bundle_ready = ~fifo_full;
  wire push = v_bundle_valid & v_bundle_ready;

  // ------------------------------------------------------------
  // FIFO storage
  // ------------------------------------------------------------
  logic [SUM_W-1:0]  sumY_mem     [FIFO_DEPTH];
  logic [YPIX_W-1:0] minY_mem     [FIFO_DEPTH];
  logic [YPIX_W-1:0] maxY_mem     [FIFO_DEPTH];
  logic [EDGE_W-1:0] edge_sum_mem [FIFO_DEPTH];
  logic [15:0]       tile_x_mem   [FIFO_DEPTH];
  logic [15:0]       tile_y_mem   [FIFO_DEPTH];

  // Read current entry
  wire [SUM_W-1:0]  sumY_q     = sumY_mem[rd_ptr];
  wire [YPIX_W-1:0] minY_q     = minY_mem[rd_ptr];
  wire [YPIX_W-1:0] maxY_q     = maxY_mem[rd_ptr];
  wire [EDGE_W-1:0] edge_sum_q = edge_sum_mem[rd_ptr];
  wire [15:0]       tile_x_q   = tile_x_mem[rd_ptr];
  wire [15:0]       tile_y_q   = tile_y_mem[rd_ptr];

  // ------------------------------------------------------------
  // Downstream (packer) handshake: atomic pair
  // ------------------------------------------------------------
  wire v_pair_valid = ~fifo_empty;

  // v3/v4 BOTH valid together (atomic)
  wire v3_valid = v_pair_valid;
  wire v4_valid = v_pair_valid;

  // Pop only if BOTH channels accepted in same cycle
  wire pop = v_pair_valid & v3_ready & v4_ready;

  // ------------------------------------------------------------
  // FIFO update
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      fifo_cnt <= '0;
      rd_ptr   <= '0;
      wr_ptr   <= '0;
    end else begin
      case ({push, pop})
        2'b10: begin // push only
          sumY_mem[wr_ptr]     <= sumY;
          minY_mem[wr_ptr]     <= minY;
          maxY_mem[wr_ptr]     <= maxY;
          edge_sum_mem[wr_ptr] <= edge_sum;
          tile_x_mem[wr_ptr]   <= tile_x_o_s;
          tile_y_mem[wr_ptr]   <= tile_y_o_s;

          wr_ptr   <= wr_ptr + 1'b1;
          fifo_cnt <= fifo_cnt + 1'b1;
        end

        2'b01: begin // pop only
          rd_ptr   <= rd_ptr + 1'b1;
          fifo_cnt <= fifo_cnt - 1'b1;
        end

        2'b11: begin // pop + push same cycle (count unchanged)
          // pop (advance rd)
          rd_ptr <= rd_ptr + 1'b1;

          // push into wr slot
          sumY_mem[wr_ptr]     <= sumY;
          minY_mem[wr_ptr]     <= minY;
          maxY_mem[wr_ptr]     <= maxY;
          edge_sum_mem[wr_ptr] <= edge_sum;
          tile_x_mem[wr_ptr]   <= tile_x_o_s;
          tile_y_mem[wr_ptr]   <= tile_y_o_s;

          wr_ptr <= wr_ptr + 1'b1;
        end

        default: begin
          // 00: nothing
        end
      endcase
    end
  end


  // ============================================================
  // Stage A instance
  // ============================================================
  pix_xy_linebuf_stats_edge_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .YPIX_W(YPIX_W),

    .USE_SOF(1'b0),
    .USE_EOL(1'b0),
    .USE_EOF(1'b0),
    .ADVANCE_ON_VALID_ONLY(1'b1),

    .SAT_AT_MAX(SAT_AT_MAX),


    .ROI_EN_DEFAULT(ROI_EN_DEFAULT),
    .ROI_X0_DEFAULT(ROI_X0_DEFAULT),
    .ROI_Y0_DEFAULT(ROI_Y0_DEFAULT),
    .ROI_X1_DEFAULT(ROI_X1_DEFAULT),
    .ROI_Y1_DEFAULT(ROI_Y1_DEFAULT),

    .DBG_COUNTERS_EN(DBG_COUNTERS_EN),

    .TILE_W(TILE_W),
    .TILE_H(TILE_H),

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

    .TILE_PIXELS(TILE_PIXELS),
    .SUM_MAX(SUM_MAX),
    .SUM_W(SUM_W),
    .SUMSQ_MAX(SUMSQ_MAX),
    .SUMSQ_W(SUMSQ_W)
  ) u_stats_edge (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .pix_ready(pix_ready),
    .sof(sof),
    .eol(eol),
    .eof(eof),
    .y_in(y_in),

    .out_valid(v_bundle_valid),
    .out_ready(v_bundle_ready),

    .sumY(sumY),
    .minY(minY),
    .maxY(maxY),
    .meanY(meanY),
    .rangeY(rangeY),

    .stats_tile_err(stats_tile_err),
    .stats_err_code(stats_err_code),
    .stats_pix_cnt_o(stats_pix_cnt_o),

    .y_meta_o(y_meta_o_s),
    .tile_x_o(tile_x_o_s),
    .tile_y_o(tile_y_o_s),
    .frame_id_o(frame_id_o_s),

    .edge_sum(edge_sum),
    .edge_max(edge_max),
    .edge_cnt(edge_cnt),
    .edge_mean(edge_mean),

    .x(x),
    .y(y),

    .tile_j(tile_j_dbg),
    .tile_i(tile_i_dbg),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .tile_first(tile_first),
    .tile_last(tile_last),
    .in_roi(in_roi),

    .err_sof_midframe(err_sof_midframe),
    .err_eol_mismatch(err_eol_mismatch),
    .frame_cnt(frame_cnt),
    .line_cnt(line_cnt)
  );


  // ============================================================
  // FIFO payload regs + packer handshake wires (MUST declare)
  // ============================================================


  logic v3_ready, v4_ready;


  wire [TILE_I_W-1:0] ti_now = tile_y_q[TILE_I_W-1:0];
  wire [TILE_J_W-1:0] tj_now = tile_x_q[TILE_J_W-1:0];

  feature_pack_and_valid #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .SUM_W(SUM_W),
    .Y_W(YPIX_W),
    .EDGE_W(EDGE_W),

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
  ) u_pack (
    .clk(clk),
    .rst(rst),

    // v3: stats
    .v3_valid(v3_valid),
    .v3_ready(v3_ready),
    .sumY(sumY_q),
    .minY(minY_q),
    .maxY(maxY_q),
    .tile_i(ti_now),
    .tile_j(tj_now),

    // v4: edge
    .v4_valid(v4_valid),
    .v4_ready(v4_ready),
    .edge_sum(edge_sum_q),
    .v4_tile_i(ti_now),
    .v4_tile_j(tj_now),

    // output
    .feat_valid(feat_valid),
    .feat_ready(feat_ready),

    .tile_i_o(tile_i_o),
    .tile_j_o(tile_j_o),
    .tile_id_o(tile_id_o),

    .feat0(feat0),
    .feat1(feat1),
    .feat2(feat2),
    .feat3(feat3),
    .feat4(feat4),
    .feat5(feat5),
    .feat6(feat6),
    .feat7(feat7),
    .feat_vec(feat_vec),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

endmodule

`default_nettype wire
`endif
