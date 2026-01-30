`timescale 1ns/1ps
`default_nettype none

module tile_stats_4x4 #(
  parameter int unsigned Y_W      = 8,
  parameter int unsigned TILE_W   = 4,
  parameter int unsigned TILE_H   = 4,

  // ---- feature switches ----
  parameter bit          SUPPORT_PIX_VALID = 1'b1,  // pix_in_tile_valid
  parameter bit          DO_SUMSQ           = 1'b0, // optional (not used in TB now)
  parameter bit          SATURATE_SUM       = 1'b0, // optional
  parameter bit          SATURATE_SUMSQ     = 1'b0, // optional
  parameter bit          ASSERT_ON          = 1'b1
)(
  input  logic                  clk,
  input  logic                  rst,

  // inputs (v2)
  input  logic                  v2_valid,
  input  logic [Y_W-1:0]         y_in,
  input  logic [$clog2(TILE_W)-1:0] x_mod,
  input  logic [$clog2(TILE_H)-1:0] y_mod,
  input  logic                  tile_last,
  input  logic [15:0]            y_cur,

  input  logic                  pix_in_tile_valid,

  input  logic [15:0]            tile_x_idx,
  input  logic [15:0]            tile_y_idx,
  input  logic [15:0]            frame_id,

  // outputs (v3) with ready/valid
  output logic                  v3_valid,
  input  logic                  v3_ready,

  output logic [SUM_W-1:0]       sumY,
  output logic [Y_W-1:0]         minY,
  output logic [Y_W-1:0]         maxY,

  output logic [Y_W-1:0]         meanY,
  output logic [Y_W-1:0]         rangeY,

  output logic [SUMSQ_W-1:0]     sumSq,

  output logic [15:0]            y_cur_o,
  output logic [15:0]            tile_x_o,
  output logic [15:0]            tile_y_o,
  output logic [15:0]            frame_id_o,

  output logic                  tile_err,
  output logic [7:0]            err_code,
  output logic [$clog2(TILE_W*TILE_H+1)-1:0] pix_cnt_o,
  output logic                  overflow_sum,
  output logic                  overflow_sumsq
);

  // ----------------------------------------------------------------
  // derived widths
  // ----------------------------------------------------------------
  localparam int unsigned TILE_PIXELS = TILE_W * TILE_H;

  localparam int unsigned SUM_MAX      = TILE_PIXELS * ((1<<Y_W) - 1);
  localparam int unsigned SUM_W_LOCAL  = (SUM_MAX <= 1) ? 1 : $clog2(SUM_MAX + 1);
  localparam int unsigned SUM_W        = SUM_W_LOCAL;

  localparam int unsigned SUMSQ_MAX     = TILE_PIXELS * (((1<<Y_W)-1) * ((1<<Y_W)-1));
  localparam int unsigned SUMSQ_W_LOCAL = (SUMSQ_MAX <= 1) ? 1 : $clog2(SUMSQ_MAX + 1);
  localparam int unsigned SUMSQ_W       = SUMSQ_W_LOCAL;

  // ----------------------------------------------------------------
  // helpers
  // ----------------------------------------------------------------
  wire is_tile_first = (x_mod == '0) && (y_mod == '0);
  wire pix_ok        = v2_valid && (SUPPORT_PIX_VALID ? pix_in_tile_valid : 1'b1);
  wire out_hold      = v3_valid && !v3_ready;

  // extend
  logic [SUM_W-1:0]   y_ext_sum;
  logic [SUMSQ_W-1:0] y_sq;

  always_comb begin
    y_ext_sum = {{(SUM_W-Y_W){1'b0}}, y_in};
    y_sq      = '0;
    if (DO_SUMSQ) y_sq = y_in * y_in;
  end

  // ----------------------------------------------------------------
  // running accumulators
  // ----------------------------------------------------------------
  logic [SUM_W-1:0]   sum_r;
  logic [Y_W-1:0]     min_r, max_r;
  logic [SUMSQ_W-1:0] sumsq_r;

  logic in_tile;
  logic [$clog2(TILE_PIXELS+1)-1:0] pix_cnt; // index of current pixel (0..15)

  logic ov_sum_r, ov_sumsq_r;

  // ----------------------------------------------------------------
  // final-computed values for "this pixel" (include current pixel)
  // (computed combinationally; used for latch on tile_last)
  // ----------------------------------------------------------------
  logic [SUM_W-1:0]   sum_final;
  logic [Y_W-1:0]     min_final, max_final;
  logic [SUMSQ_W-1:0] sumsq_final;
  logic [$clog2(TILE_PIXELS+1)-1:0] pix_cnt_eff;

  always_comb begin
    // "this pixel index"
    pix_cnt_eff = is_tile_first ? '0 : (pix_cnt + 1'b1);

    if (is_tile_first) begin
      sum_final  = y_ext_sum;
      min_final  = y_in;
      max_final  = y_in;
      sumsq_final= (DO_SUMSQ) ? y_sq : '0;
    end else begin
      sum_final  = sum_r + y_ext_sum;
      min_final  = (y_in < min_r) ? y_in : min_r;
      max_final  = (y_in > max_r) ? y_in : max_r;
      sumsq_final= (DO_SUMSQ) ? (sumsq_r + y_sq) : sumsq_r;
    end
  end

  // ----------------------------------------------------------------
  // output regs (latched at tile_last, held by v3_ready)
  // ----------------------------------------------------------------
  logic [SUM_W-1:0]   sum_out_r;
  logic [Y_W-1:0]     min_out_r, max_out_r;
  logic [SUMSQ_W-1:0] sumsq_out_r;

  logic [15:0] ycur_out_r, tx_out_r, ty_out_r, fid_out_r;
  logic        v_out_r;

  logic        tile_err_r;
  logic [7:0]  err_code_r;
  logic [$clog2(TILE_PIXELS+1)-1:0] pix_cnt_out_r;

  logic ov_sum_out_r, ov_sumsq_out_r;

  // ----------------------------------------------------------------
  // sequential
  // ----------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      sum_r   <= '0;
      min_r   <= {Y_W{1'b1}};
      max_r   <= '0;
      sumsq_r <= '0;

      in_tile <= 1'b0;
      pix_cnt <= '0;

      ov_sum_r   <= 1'b0;
      ov_sumsq_r <= 1'b0;

      sum_out_r   <= '0;
      min_out_r   <= '0;
      max_out_r   <= '0;
      sumsq_out_r <= '0;

      ycur_out_r  <= '0;
      tx_out_r    <= '0;
      ty_out_r    <= '0;
      fid_out_r   <= '0;

      v_out_r       <= 1'b0;
      tile_err_r    <= 1'b0;
      err_code_r    <= 8'h00;
      pix_cnt_out_r <= '0;

      ov_sum_out_r   <= 1'b0;
      ov_sumsq_out_r <= 1'b0;

    end else begin
      // hold v_out_r/data until accepted
      if (!out_hold) v_out_r <= 1'b0;

      // clear error at tile_first
      if (pix_ok && is_tile_first) begin
        tile_err_r <= 1'b0;
        err_code_r <= 8'h00;
      end

      if (pix_ok) begin

        // ---- update running accumulators ----
        if (is_tile_first) begin
          in_tile <= 1'b1;
          pix_cnt <= '0;

          sum_r <= sum_final;
          min_r <= min_final;
          max_r <= max_final;
          if (DO_SUMSQ) sumsq_r <= sumsq_final;
          else          sumsq_r <= '0;

          ov_sum_r   <= 1'b0;
          ov_sumsq_r <= 1'b0;

          // latch meta at tile_first
          tx_out_r  <= tile_x_idx;
          ty_out_r  <= tile_y_idx;
          fid_out_r <= frame_id;

        end else begin
          if (!in_tile) begin
            tile_err_r <= 1'b1;
            err_code_r <= 8'h01; // non-first pixel while not in_tile
          end else begin
            pix_cnt <= pix_cnt_eff; // index of current pixel
            sum_r   <= sum_final;
            min_r   <= min_final;
            max_r   <= max_final;
            if (DO_SUMSQ) sumsq_r <= sumsq_final;
          end
        end

        // ---- tile_last: integrity check + latch outputs using FINAL ----
        if (tile_last) begin
          if (!in_tile && !is_tile_first) begin
            tile_err_r <= 1'b1;
            err_code_r <= 8'h02; // tile_last while not in_tile
          end else begin
            // fixed 4x4 expects last index = 15
            if (pix_cnt_eff != (TILE_PIXELS-1)) begin
              tile_err_r <= 1'b1;
              err_code_r <= 8'h03; // incomplete/wrong length
            end

            if (!out_hold) begin
              sum_out_r     <= sum_final;
              min_out_r     <= min_final;
              max_out_r     <= max_final;
              sumsq_out_r   <= sumsq_final;
              ycur_out_r    <= y_cur;
              pix_cnt_out_r <= pix_cnt_eff;

              ov_sum_out_r   <= ov_sum_r;
              ov_sumsq_out_r <= ov_sumsq_r;

              v_out_r <= 1'b1;
            end
          end

          // end tile
          in_tile <= 1'b0;
          pix_cnt <= '0;
        end

      end // pix_ok
    end
  end

  // ----------------------------------------------------------------
  // derived outputs (from latched outputs)
  // ----------------------------------------------------------------
  localparam int unsigned MEAN_SHIFT = (TILE_PIXELS <= 1) ? 0 : $clog2(TILE_PIXELS);

  logic [SUM_W-1:0] mean_ext;
  logic [Y_W:0]     range_ext;

  always_comb begin
    mean_ext  = (MEAN_SHIFT == 0) ? sum_out_r : (sum_out_r >> MEAN_SHIFT);
    range_ext = {1'b0, max_out_r} - {1'b0, min_out_r};
  end

  // ----------------------------------------------------------------
  // outputs
  // ----------------------------------------------------------------
  assign v3_valid   = v_out_r;

  assign sumY       = sum_out_r;
  assign minY       = min_out_r;
  assign maxY       = max_out_r;

  assign meanY      = mean_ext[Y_W-1:0];
  assign rangeY     = range_ext[Y_W-1:0];

  assign sumSq      = sumsq_out_r;

  assign y_cur_o    = ycur_out_r;
  assign tile_x_o   = tx_out_r;
  assign tile_y_o   = ty_out_r;
  assign frame_id_o = fid_out_r;

  assign tile_err   = tile_err_r;
  assign err_code   = err_code_r;
  assign pix_cnt_o  = pix_cnt_out_r;

  assign overflow_sum   = ov_sum_out_r;
  assign overflow_sumsq = ov_sumsq_out_r;

  // optional assertions hook (kept minimal)
  generate if (ASSERT_ON) begin : G_ASSERT
    // You can add SVAs here if desired
  end endgenerate

endmodule

`default_nettype wire
