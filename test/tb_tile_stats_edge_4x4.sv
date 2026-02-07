// tb_tile_stats_edge_4x4.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_tile_stats_edge_4x4.vvp \
  ./test/tb_tile_stats_edge_4x4.sv

vvp ./vvp/tb_tile_stats_edge_4x4.vvp
gtkwave tb_tile_stats_edge_4x4.vcd
*/
`include "./src/AMOLED/tile feature_extracter/tile_stats_edge_4x4.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_tile_stats_edge_4x4;

  localparam int Y_W    = 8;
  localparam int TILE_W = 4;
  localparam int TILE_H = 4;
  localparam int TILE_PIX = TILE_W*TILE_H;

  localparam int EDGE_W = 32;
  localparam int CNT_W  = 8;
  localparam int THR    = 8;

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // -----------------------------
  // DUT upstream (pixel stream)
  // -----------------------------
  logic                     v2_valid;
  logic                     v2_ready;

  logic [Y_W-1:0]            y_cur;
  logic [Y_W-1:0]            y_left;
  logic [Y_W-1:0]            y_up;
  logic [$clog2(TILE_W)-1:0] x_mod;
  logic [$clog2(TILE_H)-1:0] y_mod;

  logic                     tile_last;
  logic                     tile_start;
  logic                     pix_in_tile_valid;

  logic [15:0]              tile_x_idx;
  logic [15:0]              tile_y_idx;
  logic [15:0]              frame_id;
  logic [15:0]              y_meta;

  // -----------------------------
  // DUT downstream (tile bundle)
  // -----------------------------
  logic                     out_valid;
  logic                     out_ready;

  // stats
  logic [11:0]              sumY;
  logic [Y_W-1:0]           minY, maxY, meanY, rangeY;
  logic                     stats_tile_err;
  logic [7:0]               stats_err_code;
  logic [$clog2(TILE_PIX+1)-1:0] stats_pix_cnt_o;

  logic [15:0]              y_meta_o, tile_x_o, tile_y_o, frame_id_o;

  // edge
  logic [EDGE_W-1:0]        edge_sum, edge_max, edge_mean;
  logic [CNT_W-1:0]         edge_cnt;

  // -----------------------------
  // Instantiate DUT (MODE0)
  // -----------------------------
  tile_stats_edge_4x4 #(
    .Y_W(Y_W),
    .TILE_W(TILE_W),
    .TILE_H(TILE_H),

    .SUPPORT_PIX_VALID_STATS(1'b1),
    .DO_SUMSQ_STATS(1'b0),
    .ASSERT_ON_STATS(1'b1),

    .SUPPORT_PIX_VALID_EDGE(1'b1),
    .SUPPORT_TILE_START_EDGE(1'b1),
    .ASSERT_ON_EDGE(1'b0),

    .EDGE_MODE(0),
    .EDGE_THR(THR),

    .EDGE_W(EDGE_W),
    .CNT_W(CNT_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .v2_valid(v2_valid),
    .v2_ready(v2_ready),

    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .tile_last(tile_last),
    .tile_start(tile_start),
    .pix_in_tile_valid(pix_in_tile_valid),

    .tile_x_idx(tile_x_idx),
    .tile_y_idx(tile_y_idx),
    .frame_id(frame_id),
    .y_meta(y_meta),

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
    .edge_mean(edge_mean)
  );

  // ============================================================
  // Tile storage (1D)
  // idx = ym*4 + xm
  // ============================================================
  integer tileA [0:15];
  integer tileB [0:15];

  function integer idx(input integer xm, input integer ym);
    idx = ym*4 + xm;
  endfunction

  function integer iabs(input integer a);
    if (a < 0) iabs = -a; else iabs = a;
  endfunction

  // ============================================================
  // Utils / Tasks
  // ============================================================
  task automatic reset_dut();
    begin
      rst = 1'b1;

      v2_valid = 1'b0;
      y_cur  = '0;
      y_left = '0;
      y_up   = '0;
      x_mod  = '0;
      y_mod  = '0;

      tile_last  = 1'b0;
      tile_start = 1'b0;
      pix_in_tile_valid = 1'b1;

      tile_x_idx = 16'd0;
      tile_y_idx = 16'd0;
      frame_id   = 16'd0;
      y_meta     = 16'd0;

      out_ready  = 1'b1;

      repeat (5) @(posedge clk);
      rst = 1'b0;
      repeat (2) @(posedge clk);
    end
  endtask

  task automatic wait_out_valid_or_timeout(input int max_cycles);
    int cyc;
    begin
      cyc = 0;
      while (out_valid !== 1'b1 && cyc < max_cycles) begin
        @(posedge clk);
        cyc++;
      end
      if (out_valid !== 1'b1) $fatal(1, "TIMEOUT: out_valid not asserted within %0d cycles", max_cycles);
    end
  endtask

  task automatic expect_eq_int(input string name, input int got, input int exp);
    begin
      if (got !== exp) $fatal(1, "%s mismatch: got=%0d exp=%0d", name, got, exp);
    end
  endtask

  task automatic expect_eq_bit(input string name, input bit got, input bit exp);
    begin
      if (got !== exp) $fatal(1, "%s mismatch: got=%0b exp=%0b", name, got, exp);
    end
  endtask

  // Compute expected edge metrics for MODE0: sum(|dx|+|dy|), max(|dx|+|dy|),
  // cnt = valid && (|dx|+|dy| > THR)
  task automatic compute_expected_edge_mode0(
    input integer use_tileA, // 1: tileA, 0: tileB
    output integer exp_sum,
    output integer exp_max,
    output integer exp_cnt
  );
    integer ym, xm;
    integer cur, left, up;
    integer dx, dy, e;
    begin
      exp_sum = 0; exp_max = 0; exp_cnt = 0;

      for (ym=0; ym<4; ym=ym+1) begin
        for (xm=0; xm<4; xm=xm+1) begin
          cur  = use_tileA ? tileA[idx(xm,ym)] : tileB[idx(xm,ym)];

          if (xm==0) left = cur;
          else       left = use_tileA ? tileA[idx(xm-1,ym)] : tileB[idx(xm-1,ym)];

          if (ym==0) up = cur;
          else       up = use_tileA ? tileA[idx(xm,ym-1)] : tileB[idx(xm,ym-1)];

          dx = (xm==0) ? 0 : iabs(cur - left);
          dy = (ym==0) ? 0 : iabs(cur - up);
          e  = dx + dy;

          exp_sum += e;
          if (e > exp_max) exp_max = e;
          if (e > THR) exp_cnt += 1;
        end
      end
    end
  endtask

  // Drive one pixel, respecting v2_ready (so upstream never violates handshake)
  task automatic drive_pix(
    input int xm,
    input int ym,
    input int cur,
    input int left,
    input int up,
    input bit is_first,
    input bit is_last
  );
    begin
      // wait until DUT ready
      while (!v2_ready) @(posedge clk);

      @(negedge clk);
      v2_valid = 1'b1;

      x_mod = xm[$bits(x_mod)-1:0];
      y_mod = ym[$bits(y_mod)-1:0];

      y_cur  = cur[Y_W-1:0];
      y_left = left[Y_W-1:0];
      y_up   = up[Y_W-1:0];

      tile_start = is_first;
      tile_last  = is_last;
      pix_in_tile_valid = 1'b1;

      @(posedge clk);
      @(negedge clk);
      v2_valid    = 1'b0;
      tile_start  = 1'b0;
      tile_last   = 1'b0;
    end
  endtask

  // Drive full 4x4 tile
  task automatic drive_tile(
    input integer use_tileA,
    input int meta_y,
    input int tx,
    input int ty,
    input int fid
  );
    integer ym, xm;
    integer cur, left, up;
    bit first, last;
    begin
      tile_x_idx = tx[15:0];
      tile_y_idx = ty[15:0];
      frame_id   = fid[15:0];
      y_meta     = meta_y[15:0];

      for (ym=0; ym<4; ym=ym+1) begin
        for (xm=0; xm<4; xm=xm+1) begin
          cur  = use_tileA ? tileA[idx(xm,ym)] : tileB[idx(xm,ym)];

          if (xm==0) left = cur;
          else       left = use_tileA ? tileA[idx(xm-1,ym)] : tileB[idx(xm-1,ym)];

          if (ym==0) up = cur;
          else       up = use_tileA ? tileA[idx(xm,ym-1)] : tileB[idx(xm,ym-1)];

          first = (xm==0 && ym==0);
          last  = (xm==3 && ym==3);

          drive_pix(xm, ym, cur, left, up, first, last);
        end
      end
    end
  endtask

  // Check one tile bundle (MODE0, all-valid)
  task automatic check_tile_bundle_mode0(
    input integer use_tileA,
    input int exp_sumY,
    input int exp_minY,
    input int exp_maxY,
    input int exp_meanY,
    input int exp_rangeY,
    input int exp_edge_sum,
    input int exp_edge_max,
    input int exp_edge_cnt,
    input int exp_meta_y,
    input int exp_tx,
    input int exp_ty,
    input int exp_fid
  );
    int exp_edge_mean;
    begin
      exp_edge_mean = exp_edge_sum / 16;

      expect_eq_int("sumY",   sumY,   exp_sumY);
      expect_eq_int("minY",   minY,   exp_minY);
      expect_eq_int("maxY",   maxY,   exp_maxY);
      expect_eq_int("meanY",  meanY,  exp_meanY);
      expect_eq_int("rangeY", rangeY, exp_rangeY);

      expect_eq_bit("stats_tile_err", stats_tile_err, 1'b0);
      expect_eq_int("stats_pix_cnt_o", stats_pix_cnt_o, 15);

      expect_eq_int("y_meta_o",    y_meta_o,    exp_meta_y);
      expect_eq_int("tile_x_o",    tile_x_o,    exp_tx);
      expect_eq_int("tile_y_o",    tile_y_o,    exp_ty);
      expect_eq_int("frame_id_o",  frame_id_o,  exp_fid);

      expect_eq_int("edge_sum",  edge_sum,  exp_edge_sum);
      expect_eq_int("edge_max",  edge_max,  exp_edge_max);
      expect_eq_int("edge_cnt",  edge_cnt,  exp_edge_cnt);
      expect_eq_int("edge_mean", edge_mean, exp_edge_mean);
    end
  endtask

  // ============================================================
  // TESTCASES
  // ============================================================

  task automatic tc1_basic_one_tile();
    integer e_sum, e_max, e_cnt;
    begin
      $display("\n[TC1] basic one tile (tileA 1..16)");
      out_ready = 1'b1;

      compute_expected_edge_mode0(1, e_sum, e_max, e_cnt);
      drive_tile(1, /*meta_y*/111, /*tx*/3, /*ty*/5, /*fid*/7);

      wait_out_valid_or_timeout(200);

      // accept it
      @(negedge clk);
      out_ready = 1'b1;
      @(posedge clk);

      check_tile_bundle_mode0(
        1,
        /*stats*/136, 1, 16, (136>>4), (16-1),
        /*edge */e_sum, e_max, e_cnt,
        /*meta */111, 3, 5, 7
      );

      // one more cycle -> should drop valid (since ready)
      @(posedge clk);
      if (out_valid === 1'b1) $fatal(1, "out_valid should deassert after accept");
    end
  endtask

  task automatic tc2_out_backpressure_hold();
    integer e_sum, e_max, e_cnt;
    int snap_sumY, snap_minY, snap_maxY;
    int snap_edge_sum, snap_edge_max, snap_edge_cnt;
    begin
      $display("\n[TC2] out_ready backpressure: out_valid holds + v2_ready stalls");

      compute_expected_edge_mode0(1, e_sum, e_max, e_cnt);
      drive_tile(1, /*meta_y*/222, /*tx*/9, /*ty*/8, /*fid*/6);

      wait_out_valid_or_timeout(200);

      // snapshot outputs
      snap_sumY     = sumY;
      snap_minY     = minY;
      snap_maxY     = maxY;
      snap_edge_sum = edge_sum;
      snap_edge_max = edge_max;
      snap_edge_cnt = edge_cnt;

      // stall downstream
      out_ready = 1'b0;

      // while stalled: out_valid must remain high, outputs stable, and v2_ready must be 0
      repeat (8) @(posedge clk) begin
        if (out_valid !== 1'b1) $fatal(1, "out_valid should stay high while !out_ready");
        if (sumY !== snap_sumY || minY !== snap_minY || maxY !== snap_maxY) $fatal(1, "stats outputs changed during hold");
        if (edge_sum !== snap_edge_sum || edge_max !== snap_edge_max || edge_cnt !== snap_edge_cnt) $fatal(1, "edge outputs changed during hold");
        if (v2_ready !== 1'b0) $fatal(1, "v2_ready should be low while output is holding");
      end

      // release
      out_ready = 1'b1;
      @(posedge clk);

      // check values still correct
      check_tile_bundle_mode0(
        1,
        136, 1, 16, (136>>4), (16-1),
        e_sum, e_max, e_cnt,
        222, 9, 8, 6
      );

      // accept completes -> out_valid should drop next cycle
      @(posedge clk);
      if (out_valid === 1'b1) $fatal(1, "out_valid should deassert after accept");
    end
  endtask

  task automatic tc3_two_tiles_back_to_back();
    integer e_sumA, e_maxA, e_cntA;
    integer e_sumB, e_maxB, e_cntB;
    begin
      $display("\n[TC3] two tiles back-to-back (no contamination)");
      out_ready = 1'b1;

      // --- tileA ---
      compute_expected_edge_mode0(1, e_sumA, e_maxA, e_cntA);
      drive_tile(1, /*meta_y*/333, /*tx*/1, /*ty*/2, /*fid*/3);
      wait_out_valid_or_timeout(200);
      @(posedge clk); // accept same cycle since out_ready=1

      check_tile_bundle_mode0(
        1,
        136, 1, 16, (136>>4), (16-1),
        e_sumA, e_maxA, e_cntA,
        333, 1, 2, 3
      );

      // ensure it cleared
      @(posedge clk);
      if (out_valid === 1'b1) $fatal(1, "out_valid should be 0 after accept before next tile result");

      // --- tileB (16..1) ---
      compute_expected_edge_mode0(0, e_sumB, e_maxB, e_cntB);
      drive_tile(0, /*meta_y*/444, /*tx*/4, /*ty*/5, /*fid*/6);
      wait_out_valid_or_timeout(200);
      @(posedge clk); // accept

      check_tile_bundle_mode0(
        0,
        136, 1, 16, (136>>4), (16-1), // stats for values 16..1 is same sum/min/max/mean/range
        e_sumB, e_maxB, e_cntB,
        444, 4, 5, 6
      );

      @(posedge clk);
      if (out_valid === 1'b1) $fatal(1, "out_valid should deassert after accept");
    end
  endtask

  // ============================================================
  // MAIN
  // ============================================================
  integer i;

  initial begin
    $dumpfile("tb_tile_feat_join_4x4_top.vcd");
    $dumpvars(0, tb_tile_stats_edge_4x4);

    // init tileA=1..16, tileB=16..1
    for (i=0; i<16; i=i+1) begin
      tileA[i] = i+1;
      tileB[i] = 16 - i;
    end

    reset_dut();

    tc1_basic_one_tile();
    reset_dut();

    tc2_out_backpressure_hold();
    reset_dut();

    tc3_two_tiles_back_to_back();
    reset_dut();

    $display("\nALL TESTS PASS ✅");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
