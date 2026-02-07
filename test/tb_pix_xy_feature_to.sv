// tb_pix_xy_feature_top.sv
/*
Build:
  iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_pix_xy_feature_top.vvp \
    ./test/tb_pix_xy_feature_top.sv

Run:
  vvp ./vvp/tb_pix_xy_feature_top.vvp

Wave:
  gtkwave ./vvp/tb_pix_xy_feature_top.vcd
*/

`include "./src/AMOLED/tile feature_extracter/pix_xy_feature_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_pix_xy_feature_top;

  // ============================================================
  // FAST params (small image)
  // ============================================================
  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;

  localparam int unsigned ACTIVE_W = 16; // multiple of 4
  localparam int unsigned ACTIVE_H = 8;  // multiple of 4

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILE_W     = 4;
  localparam int unsigned TILE_H     = 4;
  localparam int unsigned TILE_PIX   = TILE_W*TILE_H; // 16

  localparam int unsigned TILES_X = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y = (ACTIVE_H >> TILE_SHIFT); // 2

  localparam int unsigned YPIX_W = 8;

  localparam int unsigned EDGE_W   = 32;
  localparam int unsigned CNT_W    = 8;
  localparam int unsigned EDGE_THR = 8;
  localparam int unsigned EDGE_MODE= 0;

  localparam int unsigned FEAT_W   = 16;
  localparam int unsigned FEAT_DIM = 4;

  localparam bit EDGE_USE_MEAN = 1'b1;
  localparam int unsigned TILE_PIX_SHIFT  = 4;   // 16 pix
  localparam int unsigned TILE_PIX_N      = 16;
  localparam bit          MEAN_MODE_SHIFT = 1'b1;

  localparam bit MATCH_MODE_BUFFERED = 1'b0; // same-cycle join recommended

  localparam int unsigned TILE_I_W  = 16;
  localparam int unsigned TILE_J_W  = 16;
  localparam int unsigned TILE_ID_W = (TILES_X*TILES_Y <= 1) ? 1 : $clog2(TILES_X*TILES_Y);

  // ============================================================
  // clock/reset
  // ============================================================
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst, en;

  // ============================================================
  // DUT I/O
  // ============================================================
  logic                   pix_valid;
  logic                   pix_ready;
  logic                   sof, eol, eof;
  logic [YPIX_W-1:0]      y_in;

  logic                   feat_valid;
  logic                   feat_ready;

  logic [TILE_I_W-1:0]    tile_i_o;
  logic [TILE_J_W-1:0]    tile_j_o;
  logic [TILE_ID_W-1:0]   tile_id_o;

  logic [FEAT_W-1:0]      feat0, feat1, feat2, feat3;
  logic [FEAT_W-1:0]      feat4, feat5, feat6, feat7;
  logic [8*FEAT_W-1:0]    feat_vec;

  logic                   stats_tile_err;
  logic [7:0]             stats_err_code;

  logic                   err_mismatch_pulse;
  logic [31:0]            cnt_join_ok, cnt_mismatch, cnt_drop;

  // ============================================================
  // DUT
  // ============================================================
  pix_xy_feature_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .YPIX_W(YPIX_W),

    .TILE_W(TILE_W),
    .TILE_H(TILE_H),

    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W),
    .EDGE_THR(EDGE_THR),
    .EDGE_MODE(EDGE_MODE),

    .FEAT_W(FEAT_W),
    .FEAT_DIM(FEAT_DIM),

    .EDGE_USE_MEAN(EDGE_USE_MEAN),
    .TILE_PIX_SHIFT(TILE_PIX_SHIFT),
    .TILE_PIX_N(TILE_PIX_N),
    .MEAN_MODE_SHIFT(MEAN_MODE_SHIFT),

    .MATCH_MODE_BUFFERED(MATCH_MODE_BUFFERED),

    .TILE_I_W(TILE_I_W),
    .TILE_J_W(TILE_J_W),
    .TILE_ID_W(TILE_ID_W)
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

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),

    .tile_i_o(tile_i_o),
    .tile_j_o(tile_j_o),
    .tile_id_o(tile_id_o),

    .feat0(feat0), .feat1(feat1), .feat2(feat2), .feat3(feat3),
    .feat4(feat4), .feat5(feat5), .feat6(feat6), .feat7(feat7),
    .feat_vec(feat_vec),

    .stats_tile_err(stats_tile_err),
    .stats_err_code(stats_err_code),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

  // ============================================================
  // helpers
  // ============================================================
  task automatic step; begin @(posedge clk); #1; end endtask

  function automatic int tile_id_calc(input int ti, input int tj);
    tile_id_calc = ti*TILES_X + tj;
  endfunction

  function automatic int iabs(input int a);
    if (a < 0) iabs = -a; else iabs = a;
  endfunction

  wire fire_in  = pix_valid && pix_ready;
  wire fire_out = feat_valid && feat_ready;

  // ============================================================
  // Downstream backpressure pattern (feat_ready)
  // ============================================================
  int cyc;
  bit bp_enable;

  always_comb begin
    if (!bp_enable) feat_ready = 1'b1;
    else begin
      // stall 2 cycles every 7 cycles
      if ((cyc % 7) == 5 || (cyc % 7) == 6) feat_ready = 1'b0;
      else                                  feat_ready = 1'b1;
    end
  end

  // ============================================================
  // HOLD check under stall (output must be stable while feat_valid=1 & feat_ready=0)
  // ============================================================
  logic hold_active;
  logic [TILE_I_W-1:0] hold_ti;
  logic [TILE_J_W-1:0] hold_tj;
  logic [TILE_ID_W-1:0] hold_tid;
  logic [FEAT_W-1:0] hold_f0, hold_f1, hold_f2, hold_f3;

  always_ff @(posedge clk) begin
    if (rst || !en) begin
      hold_active <= 1'b0;
      hold_ti <= '0; hold_tj <= '0; hold_tid <= '0;
      hold_f0 <= '0; hold_f1 <= '0; hold_f2 <= '0; hold_f3 <= '0;
    end else begin
      if (feat_valid && !feat_ready) begin
        if (!hold_active) begin
          hold_active <= 1'b1;
          hold_ti  <= tile_i_o;
          hold_tj  <= tile_j_o;
          hold_tid <= tile_id_o;
          hold_f0  <= feat0;
          hold_f1  <= feat1;
          hold_f2  <= feat2;
          hold_f3  <= feat3;
        end else begin
          if (tile_i_o !== hold_ti)  $fatal(1, "[HOLD] tile_i_o changed under stall");
          if (tile_j_o !== hold_tj)  $fatal(1, "[HOLD] tile_j_o changed under stall");
          if (tile_id_o!== hold_tid) $fatal(1, "[HOLD] tile_id_o changed under stall");
          if (feat0 !== hold_f0) $fatal(1, "[HOLD] feat0 changed under stall");
          if (feat1 !== hold_f1) $fatal(1, "[HOLD] feat1 changed under stall");
          if (feat2 !== hold_f2) $fatal(1, "[HOLD] feat2 changed under stall");
          if (feat3 !== hold_f3) $fatal(1, "[HOLD] feat3 changed under stall");
        end
      end
      if (fire_out) hold_active <= 1'b0;
    end
  end

  // ============================================================
  // Drive pixel with proper sof/eol/eof; hold valid until accepted
  // ============================================================
  task automatic send_pix(input int x, input int y, input byte val);
    int guard;
    bit accepted;
    begin
      pix_valid = 1'b1;
      y_in = val[YPIX_W-1:0];

      sof = (x == 0 && y == 0);
      eol = (x == (ACTIVE_W-1));
      eof = (x == (ACTIVE_W-1) && y == (ACTIVE_H-1));

      guard = 0;
      accepted = 0;

      while (!accepted) begin
        step(); cyc++;
        accepted = fire_in;
        guard++;
        if (guard > 20000) begin
          $display("[TB] TIMEOUT waiting fire_in (pix_ready=%0b feat_valid=%0b feat_ready=%0b)",
                   pix_ready, feat_valid, feat_ready);
          $fatal(1, "FAIL");
        end
      end

      // drop after accept
      pix_valid = 1'b0;
      y_in = '0;
      sof = 1'b0; eol = 1'b0; eof = 1'b0;

      // optional bubble
      step(); cyc++;
    end
  endtask

  // ============================================================
  // Expected queue per tile
  // ============================================================
  localparam int QDEPTH = 256;

  typedef struct packed {
    logic [TILE_I_W-1:0]  ti;
    logic [TILE_J_W-1:0]  tj;
    logic [TILE_ID_W-1:0] tid;
    logic [FEAT_W-1:0]    f0, f1, f2, f3;
  } exp_feat_t;

  exp_feat_t q [0:QDEPTH-1];
  int q_wptr, q_rptr;

  task automatic q_reset; begin q_wptr=0; q_rptr=0; end endtask
  task automatic q_push(input exp_feat_t e);
    begin
      if (q_wptr >= QDEPTH) $fatal(1, "[TB] expected queue overflow");
      q[q_wptr] = e;
      q_wptr++;
    end
  endtask
  task automatic q_pop(output exp_feat_t e);
    begin
      if (q_rptr >= q_wptr) $fatal(1, "[TB] expected queue underflow");
      e = q[q_rptr];
      q_rptr++;
    end
  endtask

  // ============================================================
  // Tile model (MODE0 edge): left/up replicate at border
  // ============================================================
  int tile_pix [0:TILE_H-1][0:TILE_W-1];

  int acc_sum, acc_min, acc_max;
  int acc_edge_sum;
  int cur_ti, cur_tj;

  task automatic tile_model_reset;
    int yy, xx;
    begin
      acc_sum = 0;
      acc_min = 999999;
      acc_max = -1;
      acc_edge_sum = 0;

      for (yy=0; yy<TILE_H; yy=yy+1)
        for (xx=0; xx<TILE_W; xx=xx+1)
          tile_pix[yy][xx] = 0;
    end
  endtask

  // Update model on each accepted pixel (fire_in)
  // Uses x/y from TB counters (not DUT internal), but mapping matches standard tiling.
  task automatic model_on_fire_in(input int x, input int y, input int val);
    int xm, ym;
    int left, up;
    int dx, dy, e;
    begin
      xm = x & (TILE_W-1);
      ym = y & (TILE_H-1);

      // tile indices
      if (xm == 0 && ym == 0) begin
        tile_model_reset();
        cur_ti = (y >> TILE_SHIFT);
        cur_tj = (x >> TILE_SHIFT);
      end

      tile_pix[ym][xm] = val;

      // stats
      acc_sum += val;
      if (val < acc_min) acc_min = val;
      if (val > acc_max) acc_max = val;

      // edge (MODE0): left/up replicate
      left = (xm == 0) ? val : tile_pix[ym][xm-1];
      up   = (ym == 0) ? val : tile_pix[ym-1][xm];

      dx = (xm == 0) ? 0 : iabs(val - left);
      dy = (ym == 0) ? 0 : iabs(val - up);
      e  = dx + dy;

      acc_edge_sum += e;

      // tile end -> push expected feature
      if (xm == (TILE_W-1) && ym == (TILE_H-1)) begin
        exp_feat_t ex;
        int ymean;
        int edge_feat;

        // packer uses mean via shift (sum >> 4) if MEAN_MODE_SHIFT=1
        if (MEAN_MODE_SHIFT) ymean = (acc_sum >> TILE_PIX_SHIFT);
        else                ymean = (acc_sum / TILE_PIX_N);

        // EDGE_USE_MEAN means edge_feat = edge_sum >> 4
        edge_feat = (EDGE_USE_MEAN) ? (acc_edge_sum >> TILE_PIX_SHIFT) : acc_edge_sum;

        ex.ti  = cur_ti[TILE_I_W-1:0];
        ex.tj  = cur_tj[TILE_J_W-1:0];
        ex.tid = tile_id_calc(cur_ti, cur_tj)[TILE_ID_W-1:0];

        ex.f0 = ymean[FEAT_W-1:0];
        ex.f1 = acc_max[FEAT_W-1:0];
        ex.f2 = acc_min[FEAT_W-1:0];
        ex.f3 = edge_feat[FEAT_W-1:0];

        q_push(ex);
      end
    end
  endtask

  // ============================================================
  // Compare on output fire
  // ============================================================
  always_ff @(posedge clk) begin
    if (!rst && en) begin
      if (fire_out) begin
        exp_feat_t ex;
        q_pop(ex);

        if (stats_tile_err !== 1'b0) $fatal(1, "[TB] stats_tile_err asserted (code=%0d)", stats_err_code);
        if (err_mismatch_pulse !== 1'b0) $fatal(1, "[TB] err_mismatch_pulse asserted");

        if (tile_i_o !== ex.ti)  $fatal(1, "[TB] tile_i_o mismatch got=%0d exp=%0d", tile_i_o, ex.ti);
        if (tile_j_o !== ex.tj)  $fatal(1, "[TB] tile_j_o mismatch got=%0d exp=%0d", tile_j_o, ex.tj);
        if (tile_id_o!== ex.tid) $fatal(1, "[TB] tile_id_o mismatch got=%0d exp=%0d", tile_id_o, ex.tid);

        if (feat0 !== ex.f0) $fatal(1, "[TB] feat0 mismatch got=%0d exp=%0d", feat0, ex.f0);
        if (feat1 !== ex.f1) $fatal(1, "[TB] feat1 mismatch got=%0d exp=%0d", feat1, ex.f1);
        if (feat2 !== ex.f2) $fatal(1, "[TB] feat2 mismatch got=%0d exp=%0d", feat2, ex.f2);
        if (feat3 !== ex.f3) $fatal(1, "[TB] feat3 mismatch got=%0d exp=%0d", feat3, ex.f3);

        // packing check
        if (feat_vec !== {feat7, feat6, feat5, feat4, feat3, feat2, feat1, feat0})
          $fatal(1, "[TB] feat_vec packing mismatch");
      end
    end
  end

  // ============================================================
  // Main
  // ============================================================
  int x, y;

  initial begin
    $dumpfile("./vvp/tb_pix_xy_feature_top.vcd");
    $dumpvars(0, tb_pix_xy_feature_top);

    // init
    en = 1'b1;
    rst = 1'b1;
    cyc = 0;
    bp_enable = 1'b0;

    pix_valid = 1'b0;
    sof = 1'b0; eol = 1'b0; eof = 1'b0;
    y_in = '0;

    q_reset();
    tile_model_reset();

    repeat (6) step();
    rst = 1'b0;
    repeat (2) step();

    $display("[RUN] drive %0d x %0d pixels; downstream backpressure enabled", ACTIVE_W, ACTIVE_H);
    bp_enable = 1'b1;

    for (y=0; y<ACTIVE_H; y=y+1) begin
      for (x=0; x<ACTIVE_W; x=x+1) begin
        byte v;
        v = byte'(y*40 + x); // deterministic pattern
        send_pix(x, y, v);
        // update model on accepted pixel only
        // (send_pix guarantees we only proceed after fire_in)
        model_on_fire_in(x, y, v);
      end
    end

    // drain
    begin : DRAIN
      int guard;
      guard = 0;
      while (q_rptr != q_wptr) begin
        step(); cyc++; guard++;
        if (guard > 50000) begin
          $fatal(1, "[TB] TIMEOUT draining q_rptr=%0d q_wptr=%0d feat_valid=%0b feat_ready=%0b",
                 q_rptr, q_wptr, feat_valid, feat_ready);
        end
      end
    end

    $display("[PASS] tb_pix_xy_feature_top ✅  (tiles=%0d)", q_wptr);
    repeat (10) step();
    $finish;
  end

endmodule

`default_nettype wire
