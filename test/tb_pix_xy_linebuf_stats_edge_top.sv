// tb_pix_xy_linebuf_stats_edge_top.sv
/*
Build:
  iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_pix_xy_linebuf_stats_edge_top.vvp \
    ./test/tb_pix_xy_linebuf_stats_edge_top.sv

Run:
  vvp ./vvp/tb_pix_xy_linebuf_stats_edge_top.vvp

Wave:
  gtkwave ./vvp/tb_pix_xy_linebuf_stats_edge_top.vcd
*/

`include "./src/AMOLED/tile feature_extracter/pix_xy_linebuf_stats_edge_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_pix_xy_linebuf_stats_edge_top;

  // ============================================================
  // FAST params
  // ============================================================
  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;

  localparam int unsigned ACTIVE_W = 16; // multiple of 4
  localparam int unsigned ACTIVE_H = 8;  // multiple of 4

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT); // 2

  localparam int unsigned YPIX_W = 8;

  localparam int unsigned TILE_W = 4;
  localparam int unsigned TILE_H = 4;
  localparam int unsigned TILE_PIX = TILE_W*TILE_H; // 16

  localparam int unsigned EDGE_W = 32;
  localparam int unsigned CNT_W  = 8;
  localparam int unsigned THR    = 8;

  // SUM width (match DUT param logic)
  localparam int unsigned SUM_MAX = TILE_PIX * ((1<<YPIX_W)-1);
  localparam int unsigned SUM_W   = (SUM_MAX <= 1) ? 1 : $clog2(SUM_MAX + 1);

  // ============================================================
  // clock/reset
  // ============================================================
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst, en;

  // ============================================================
  // DUT I/O
  // ============================================================
  logic                 pix_valid;
  logic                 pix_ready;
  logic                 sof, eol, eof;
  logic [YPIX_W-1:0]    y_in;

  logic                 out_valid;
  logic                 out_ready;

  logic [SUM_W-1:0]     sumY;
  logic [YPIX_W-1:0]    minY, maxY, meanY, rangeY;

  logic                 stats_tile_err;
  logic [7:0]           stats_err_code;
  logic [$clog2(TILE_W*TILE_H+1)-1:0] stats_pix_cnt_o;

  logic [15:0]          y_meta_o, tile_x_o, tile_y_o, frame_id_o;

  logic [EDGE_W-1:0]    edge_sum, edge_max, edge_mean;
  logic [CNT_W-1:0]     edge_cnt;

  logic [X_W-1:0]       x;
  logic [Y_W-1:0]       y;
  logic [X_W-1:0]       tile_j;
  logic [Y_W-1:0]       tile_i;
  logic [1:0]           x_mod, y_mod;
  logic                 tile_first, tile_last;
  logic                 in_roi;

  logic                 err_sof_midframe, err_eol_mismatch;
  logic [31:0]          frame_cnt, line_cnt;

  // ============================================================
  // DUT
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

    // simplify TB: do NOT rely on SOF
    .USE_SOF(1'b0),
    .USE_EOL(1'b0),
    .USE_EOF(1'b0),
    .SAT_AT_MAX(1'b0),
    .ADVANCE_ON_VALID_ONLY(1'b1),

    .ROI_EN_DEFAULT(1'b0),
    .DBG_COUNTERS_EN(1'b1),

    .TILE_W(TILE_W),
    .TILE_H(TILE_H),

    .EDGE_MODE(0),
    .EDGE_THR(THR),
    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .pix_ready(pix_ready),
    .sof(sof),
    .eol(eol),
    .eof(eof),
    .y_in(y_in),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .sumY(sumY),
    .minY(minY),
    .maxY(maxY),
    .meanY(meanY),
    .rangeY(rangeY),

    .stats_tile_err(stats_tile_err),
    .stats_err_code(stats_err_code),
    .stats_pix_cnt_o(stats_pix_cnt_o),

    .y_meta_o(y_meta_o),
    .tile_x_o(tile_x_o),
    .tile_y_o(tile_y_o),
    .frame_id_o(frame_id_o),

    .edge_sum(edge_sum),
    .edge_max(edge_max),
    .edge_cnt(edge_cnt),
    .edge_mean(edge_mean),

    .x(x),
    .y(y),
    .tile_j(tile_j),
    .tile_i(tile_i),
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
  // helpers
  // ============================================================
  task automatic step; begin @(posedge clk); #1; end endtask
  function automatic int iabs(input int a); if (a<0) iabs=-a; else iabs=a; endfunction

  wire fire_in  = pix_valid && pix_ready;   // true accept into DUT pipeline
  wire fire_out = out_valid && out_ready;   // true consume of tile bundle

  // ============================================================
  // downstream backpressure
  // ============================================================
  int cyc;
  bit bp_enable;

  always_comb begin
    if (!bp_enable) out_ready = 1'b1;
    else begin
      // stall every 5th cycle
      if ((cyc % 5) == 4) out_ready = 1'b0;
      else                out_ready = 1'b1;
    end
  end

  // ============================================================
  // HOLD check under stall (tile bundle outputs must be stable)
  // ============================================================
  logic hold_active;
  logic [SUM_W-1:0]   hold_sumY;
  logic [YPIX_W-1:0]  hold_minY, hold_maxY, hold_meanY, hold_rangeY;
  logic [EDGE_W-1:0]  hold_edge_sum, hold_edge_max, hold_edge_mean;
  logic [CNT_W-1:0]   hold_edge_cnt;

  always_ff @(posedge clk) begin
    if (rst || !en) begin
      hold_active <= 1'b0;
      hold_sumY <= '0;
      hold_minY <= '0;
      hold_maxY <= '0;
      hold_meanY <= '0;
      hold_rangeY <= '0;
      hold_edge_sum <= '0;
      hold_edge_max <= '0;
      hold_edge_cnt <= '0;
      hold_edge_mean <= '0;
    end else begin
      if (out_valid && !out_ready) begin
        if (!hold_active) begin
          hold_active   <= 1'b1;
          hold_sumY     <= sumY;
          hold_minY     <= minY;
          hold_maxY     <= maxY;
          hold_meanY    <= meanY;
          hold_rangeY   <= rangeY;
          hold_edge_sum <= edge_sum;
          hold_edge_max <= edge_max;
          hold_edge_cnt <= edge_cnt;
          hold_edge_mean<= edge_mean;
        end else begin
          if (sumY      !== hold_sumY)      $fatal(1, "[HOLD] sumY changed under stall");
          if (minY      !== hold_minY)      $fatal(1, "[HOLD] minY changed under stall");
          if (maxY      !== hold_maxY)      $fatal(1, "[HOLD] maxY changed under stall");
          if (meanY     !== hold_meanY)     $fatal(1, "[HOLD] meanY changed under stall");
          if (rangeY    !== hold_rangeY)    $fatal(1, "[HOLD] rangeY changed under stall");
          if (edge_sum  !== hold_edge_sum)  $fatal(1, "[HOLD] edge_sum changed under stall");
          if (edge_max  !== hold_edge_max)  $fatal(1, "[HOLD] edge_max changed under stall");
          if (edge_cnt  !== hold_edge_cnt)  $fatal(1, "[HOLD] edge_cnt changed under stall");
          if (edge_mean !== hold_edge_mean) $fatal(1, "[HOLD] edge_mean changed under stall");
        end
      end
      if (fire_out) hold_active <= 1'b0;
    end
  end

  // ============================================================
  // Input driver: hold pix_valid until accepted (fire_in)
  // ============================================================
  task automatic send_pix(input byte val);
    int guard;
    bit accepted;
    begin
      pix_valid = 1'b1;
      sof = 1'b0; eol = 1'b0; eof = 1'b0;
      y_in = val[YPIX_W-1:0];

      guard = 0;
      accepted = 0;

      while (!accepted) begin
        step();
        cyc++;
        accepted = fire_in;
        guard++;
        if (guard > 5000) begin
          $display("[TB] TIMEOUT waiting fire_in (pix_ready=%0b out_valid=%0b out_ready=%0b)",
                   pix_ready, out_valid, out_ready);
          $fatal(1, "FAIL");
        end
      end

      // drop after accept
      pix_valid = 1'b0;
      y_in = '0;

      // optional bubble
      step(); cyc++;
    end
  endtask

  // ============================================================
  // Expected queue for tile bundles
  // ============================================================
  localparam int QDEPTH = 256;

  typedef struct packed {
    logic [SUM_W-1:0]    sumY;
    logic [YPIX_W-1:0]   minY, maxY, meanY, rangeY;
    logic [EDGE_W-1:0]   edge_sum, edge_max, edge_mean;
    logic [CNT_W-1:0]    edge_cnt;
    logic [15:0]         y_meta_o, tile_x_o, tile_y_o, frame_id_o;
    logic [$clog2(TILE_W*TILE_H+1)-1:0] pix_cnt_o;
  } exp_t;

  exp_t q [0:QDEPTH-1];
  int q_wptr, q_rptr;

  task automatic q_reset; begin q_wptr=0; q_rptr=0; end endtask

  task automatic q_push(input exp_t e);
    begin
      if (q_wptr >= QDEPTH) $fatal(1, "[TB] expected queue overflow");
      q[q_wptr] = e;
      q_wptr++;
    end
  endtask

  task automatic q_pop(output exp_t e);
    begin
      if (q_rptr >= q_wptr) $fatal(1, "[TB] expected queue underflow");
      e = q[q_rptr];
      q_rptr++;
    end
  endtask

  // ============================================================
  // Tile accumulator (build expected per tile on fire_in)
  // ============================================================
  int tile_pix   [0:TILE_H-1][0:TILE_W-1]; // store cur values for edge calc

  int acc_sum;
  int acc_min;
  int acc_max;

  int acc_edge_sum;
  int acc_edge_max;
  int acc_edge_cnt;

  // meta snapshot for this tile
  int acc_tile_x, acc_tile_y;
  int acc_frame_id;
  int acc_y_meta_last;
  int acc_pix_last; // expected pix_cnt_o (0-based -> 15)

  task automatic tile_acc_reset;
    int yy, xx;
    begin
      acc_sum = 0;
      acc_min = 999999;
      acc_max = -1;
      acc_edge_sum = 0;
      acc_edge_max = 0;
      acc_edge_cnt = 0;
      acc_tile_x = 0;
      acc_tile_y = 0;
      acc_frame_id = 0;
      acc_y_meta_last = 0;
      acc_pix_last = 0;

      for (yy=0; yy<TILE_H; yy=yy+1)
        for (xx=0; xx<TILE_W; xx=xx+1)
          tile_pix[yy][xx] = 0;
    end
  endtask

  // On each accepted pixel: update stats/edge using DUT x_mod/y_mod/tile_*.
  // We assume MODE0 border policy: if x_mod==0 left=cur; if y_mod==0 up=cur.
  always_ff @(posedge clk) begin
    if (rst || !en) begin
      // do nothing
    end else begin
      if (fire_in) begin
        int xm, ym;
        int cur, left, up;
        int dx, dy, e;

        xm  = x_mod;
        ym  = y_mod;
        cur = y_in;

        // new tile start
        if (tile_first) begin
          tile_acc_reset();
          acc_tile_x = tile_j;
          acc_tile_y = tile_i;
          acc_frame_id = frame_cnt[15:0];
        end

        // store for later neighbor reads
        tile_pix[ym][xm] = cur;

        // stats (only count if in_roi==1)
        if (in_roi) begin
          acc_sum += cur;
          if (cur < acc_min) acc_min = cur;
          if (cur > acc_max) acc_max = cur;
        end

        // edge mode0
        if (xm == 0) left = cur;
        else        left = tile_pix[ym][xm-1];

        if (ym == 0) up = cur;
        else        up = tile_pix[ym-1][xm];

        dx = (xm == 0) ? 0 : iabs(cur - left);
        dy = (ym == 0) ? 0 : iabs(cur - up);
        e  = dx + dy;

        acc_edge_sum += e;
        if (e > acc_edge_max) acc_edge_max = e;
        if (e > THR) acc_edge_cnt += 1;

        // meta last (matches wrapper policy where y_meta varies each pixel)
        acc_y_meta_last = y[15:0];

        // expected pix_cnt_o behavior: last pixel index (0..15) => 15
        if (tile_last) acc_pix_last = TILE_PIX-1;

        // when tile_last accepted, finalize expected bundle and push
        if (tile_last) begin
          exp_t eout;
          int mean_i;
          int range_i;
          int edge_mean_i;

          mean_i      = acc_sum / TILE_PIX;
          range_i     = acc_max - acc_min;
          edge_mean_i = acc_edge_sum / TILE_PIX;

          eout.sumY       = acc_sum[SUM_W-1:0];
          eout.minY       = acc_min[YPIX_W-1:0];
          eout.maxY       = acc_max[YPIX_W-1:0];
          eout.meanY      = mean_i[YPIX_W-1:0];
          eout.rangeY     = range_i[YPIX_W-1:0];

          eout.edge_sum   = acc_edge_sum[EDGE_W-1:0];
          eout.edge_max   = acc_edge_max[EDGE_W-1:0];
          eout.edge_cnt   = acc_edge_cnt[CNT_W-1:0];
          eout.edge_mean  = edge_mean_i[EDGE_W-1:0];

          eout.tile_x_o   = acc_tile_x[15:0];
          eout.tile_y_o   = acc_tile_y[15:0];
          eout.frame_id_o = acc_frame_id[15:0];
          eout.y_meta_o   = acc_y_meta_last[15:0];
          eout.pix_cnt_o  = acc_pix_last[$bits(stats_pix_cnt_o)-1:0];

          q_push(eout);
        end
      end
    end
  end

  // ============================================================
  // Compare only when tile bundle is consumed (fire_out)
  // ============================================================
  task automatic expect_eq_int(input string name, input int got, input int exp);
    begin
      if (got !== exp) $fatal(1, "%s mismatch: got=%0d exp=%0d", name, got, exp);
    end
  endtask

  always_ff @(posedge clk) begin
    if (!rst && en) begin
      if (fire_out) begin
        exp_t ex;
        q_pop(ex);

        // stats
        if (sumY !== ex.sumY) begin
          $display("[TB] sumY mismatch t=%0t got=%0d exp=%0d tile=(%0d,%0d)",
                   $time, sumY, ex.sumY, tile_x_o, tile_y_o);
          $fatal(1, "FAIL");
        end
        if (minY !== ex.minY) $fatal(1, "[TB] minY mismatch got=%0d exp=%0d", minY, ex.minY);
        if (maxY !== ex.maxY) $fatal(1, "[TB] maxY mismatch got=%0d exp=%0d", maxY, ex.maxY);
        if (meanY!== ex.meanY)$fatal(1, "[TB] meanY mismatch got=%0d exp=%0d", meanY, ex.meanY);
        if (rangeY!==ex.rangeY)$fatal(1, "[TB] rangeY mismatch got=%0d exp=%0d", rangeY, ex.rangeY);

        // edge
        if (edge_sum  !== ex.edge_sum)  $fatal(1, "[TB] edge_sum mismatch got=%0d exp=%0d", edge_sum, ex.edge_sum);
        if (edge_max  !== ex.edge_max)  $fatal(1, "[TB] edge_max mismatch got=%0d exp=%0d", edge_max, ex.edge_max);
        if (edge_cnt  !== ex.edge_cnt)  $fatal(1, "[TB] edge_cnt mismatch got=%0d exp=%0d", edge_cnt, ex.edge_cnt);
        if (edge_mean !== ex.edge_mean) $fatal(1, "[TB] edge_mean mismatch got=%0d exp=%0d", edge_mean, ex.edge_mean);

        // meta
        if (tile_x_o   !== ex.tile_x_o)   $fatal(1, "[TB] tile_x_o mismatch got=%0d exp=%0d", tile_x_o, ex.tile_x_o);
        if (tile_y_o   !== ex.tile_y_o)   $fatal(1, "[TB] tile_y_o mismatch got=%0d exp=%0d", tile_y_o, ex.tile_y_o);
        if (frame_id_o !== ex.frame_id_o) $fatal(1, "[TB] frame_id_o mismatch got=%0d exp=%0d", frame_id_o, ex.frame_id_o);

        // y_meta_o: wrapper policy uses last pixel's y[15:0] in the tile
        if (y_meta_o !== ex.y_meta_o) $fatal(1, "[TB] y_meta_o mismatch got=%0d exp=%0d", y_meta_o, ex.y_meta_o);

        // pix cnt: many designs output last index=15 (0-based)
        if (stats_pix_cnt_o !== ex.pix_cnt_o) begin
          $display("[TB] stats_pix_cnt_o got=%0d exp=%0d (if your module counts 16, adjust here)",
                   stats_pix_cnt_o, ex.pix_cnt_o);
          $fatal(1, "FAIL");
        end

        // error flags should be 0 in normal run
        if (stats_tile_err !== 1'b0) $fatal(1, "[TB] stats_tile_err asserted");
      end
    end
  end

  // ============================================================
  // Main
  // ============================================================
  int yy, xx;

  initial begin
    $dumpfile("./vvp/tb_pix_xy_linebuf_stats_edge_top.vcd");
    $dumpvars(0, tb_pix_xy_linebuf_stats_edge_top);

    // init
    en = 1'b1;
    rst = 1'b1;
    cyc = 0;
    bp_enable = 1'b0;

    pix_valid = 1'b0;
    sof = 1'b0;
    eol = 1'b0;
    eof = 1'b0;
    y_in = '0;

    q_reset();
    tile_acc_reset();

    repeat (6) step();
    rst = 1'b0;
    repeat (2) step();

    $display("[RUN] drive %0d x %0d pixels, with backpressure", ACTIVE_W, ACTIVE_H);
    bp_enable = 1'b1;

    for (yy=0; yy<ACTIVE_H; yy=yy+1) begin
      for (xx=0; xx<ACTIVE_W; xx=xx+1) begin
        send_pix(byte'(yy*40 + xx));
      end
    end

    // drain expected queue
    begin : DRAIN
      int guard;
      guard = 0;
      while (q_rptr != q_wptr) begin
        step(); cyc++; guard++;
        if (guard > 20000) begin
          $fatal(1, "[TB] TIMEOUT draining q_rptr=%0d q_wptr=%0d out_valid=%0b out_ready=%0b",
                 q_rptr, q_wptr, out_valid, out_ready);
        end
      end
    end

    $display("[PASS] tb_pix_xy_linebuf_stats_edge_top ✅");
    repeat (10) step();
    $finish;
  end

endmodule

`default_nettype wire
