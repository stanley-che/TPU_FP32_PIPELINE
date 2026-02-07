// tb_pix_xy_feature_top_v2.sv
/*
Build:
  iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_pix_xy_feature_top_v2.vvp \
    ./test/tb_pix_xy_feature_top_v2.sv

Run:
  vvp ./vvp/tb_pix_xy_feature_top_v2.vvp

Wave:
  gtkwave ./vvp/tb_pix_xy_feature_top_v2.vcd
*/

`include "./src/AMOLED/tile feature_extracter/pix_xy_feature_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_pix_xy_feature_top_v2;

  // -----------------------------
  // small demo params
  // -----------------------------
  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 8;

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILE_W     = 4;
  localparam int unsigned TILE_H     = 4;

  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT); // 2
  localparam int unsigned EXP_TILES  = TILES_X * TILES_Y;        // 8

  localparam int unsigned YPIX_W     = 8;
  localparam int unsigned FEAT_W     = 16;
  localparam int unsigned FEAT_DIM   = 4;

  localparam int unsigned TILE_ID_W  = (EXP_TILES <= 1) ? 1 : $clog2(EXP_TILES);

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;
  logic en;

  // -----------------------------
  // upstream pixel stream
  // -----------------------------
  logic               pix_valid;
  wire                pix_ready;
  logic               sof, eol, eof;
  logic [YPIX_W-1:0]   y_in;

  // -----------------------------
  // downstream feature stream
  // -----------------------------
  wire                 feat_valid;
  logic                feat_ready;

  wire [15:0]          tile_i_o;
  wire [15:0]          tile_j_o;
  wire [TILE_ID_W-1:0] tile_id_o;

  wire [FEAT_W-1:0]    feat0, feat1, feat2, feat3;
  wire [8*FEAT_W-1:0]  feat_vec;

  wire                 err_mismatch_pulse;
  wire [31:0]          cnt_join_ok, cnt_mismatch, cnt_drop;

  // -----------------------------
  // DUT
  // -----------------------------
  pix_xy_feature_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT),
    .TILE_W(TILE_W), .TILE_H(TILE_H),
    .YPIX_W(YPIX_W),
    .FEAT_W(FEAT_W), .FEAT_DIM(FEAT_DIM),
    .MATCH_MODE_BUFFERED(1'b1),.USE_EOL(1'b0),
  .USE_EOF(1'b0)
  ) dut (
    .clk(clk), .rst(rst), .en(en),

    .pix_valid(pix_valid),
    .pix_ready(pix_ready),
    .sof(sof), .eol(eol), .eof(eof),
    .y_in(y_in),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),

    .tile_i_o(tile_i_o),
    .tile_j_o(tile_j_o),
    .tile_id_o(tile_id_o),

    .feat0(feat0), .feat1(feat1), .feat2(feat2), .feat3(feat3),
    .feat4(), .feat5(), .feat6(), .feat7(),
    .feat_vec(feat_vec),

    .stats_tile_err(), .stats_err_code(),
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

  // -----------------------------
  // helpers
  // -----------------------------
  task automatic tick; @(posedge clk); endtask

  function automatic logic [YPIX_W-1:0] gen_pix(input int xx, input int yy);
    gen_pix = (xx + (yy<<4)) & 'hFF;
  endfunction

  // ============================================================
  // SINK: choose bp or not
  // ============================================================
  task automatic sink_task(input bit bp_enable);
    begin
      feat_ready <= 1'b1;
      forever begin
        tick();
        if (bp_enable) feat_ready <= ($urandom_range(0,3) != 0);
        else           feat_ready <= 1'b1;
      end
    end
  endtask

  // ============================================================
  // Tile receive counter
  // ============================================================
  int got_tiles;
  always_ff @(posedge clk) begin
    if (rst) got_tiles <= 0;
    else if (feat_valid && feat_ready) begin
      got_tiles <= got_tiles + 1;
      $display("[TB] GOT feat ti=%0d tj=%0d id=%0d f0=%0d f1=%0d f2=%0d f3=%0d",
               tile_i_o, tile_j_o, tile_id_o, feat0, feat1, feat2, feat3);
    end
  end

  // ============================================================
  // Monitor: detect long pix_ready low (root cause of "卡在 source")
  // ============================================================
  int prdy_low_run;
  always_ff @(posedge clk) begin
    if (rst) prdy_low_run <= 0;
    else begin
      if (pix_valid && !pix_ready) prdy_low_run <= prdy_low_run + 1;
      else                         prdy_low_run <= 0;

      // If stuck too long, print a snapshot (do not immediately fatal; you can change to $fatal)
      if (prdy_low_run == 2000) begin

        $display("[TB][WARN] pix_ready stuck LOW for 2000 cycles while pix_valid=1 (likely deadlock).");
        $display("[TB][WARN] got_tiles=%0d join_ok=%0d mismatch=%0d drop=%0d",
                 got_tiles, cnt_join_ok, cnt_mismatch, cnt_drop);
                
      end
    end
  end

  // ============================================================
  // SOURCE: handshake-safe drive, but with TIMEOUT so it never hangs forever
  // - holds signals stable until accepted
  // - sof asserted only on first accepted pixel
  // - eof asserted only on last accepted pixel
  // ============================================================
  task automatic drive_frame_with_timeout(input int max_cycles);
  int xx, yy, cyc;
  bit fire;
  begin
    // init
    pix_valid <= 1'b0;
    sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
    y_in <= '0;
    tick();

    xx = 0; yy = 0; cyc = 0;

    // drive first beat
    pix_valid <= 1'b1;
    y_in      <= gen_pix(xx, yy);
    sof       <= 1'b1;                 // (0,0)
    eol       <= (xx == ACTIVE_W-1);
    eof       <= (xx == ACTIVE_W-1) && (yy == ACTIVE_H-1);

    while (yy < ACTIVE_H) begin
      tick();
      cyc++;

      // sample handshake for the beat we just presented
      fire = pix_valid && pix_ready;

      if (fire) begin
        // advance to next pixel
        xx++;
        if (xx == ACTIVE_W) begin
          xx = 0;
          yy++;
        end

        if (yy < ACTIVE_H) begin
          y_in <= gen_pix(xx, yy);
          sof  <= (xx == 0) && (yy == 0);
          eol  <= (xx == ACTIVE_W-1);
          eof  <= (xx == ACTIVE_W-1) && (yy == ACTIVE_H-1);
        end else begin
          // done
          pix_valid <= 1'b0;
          sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
          y_in <= '0;
        end
      end
      // else: stalled -> keep same xx/yy and keep sideband stable (do nothing)

      if (cyc > max_cycles) begin
        $fatal(1, "[TB] SOURCE TIMEOUT: yy=%0d xx=%0d pix_ready stuck? got_tiles=%0d",
               yy, xx, got_tiles);
      end
    end
  end
endtask


  // ============================================================
  // DRAIN: after source finished, force ready=1 and wait for exp tiles
  // ============================================================
  task automatic drain_until_done(input int exp_tiles, input int max_cycles);
    int c;
    begin
      feat_ready <= 1'b1;
      for (c=0; c<max_cycles; c++) begin
        tick();
        if (got_tiles >= exp_tiles) disable drain_until_done;
      end
      $fatal(1, "[TB] DRAIN TIMEOUT got_tiles=%0d/%0d join_ok=%0d mismatch=%0d drop=%0d",
             got_tiles, exp_tiles, cnt_join_ok, cnt_mismatch, cnt_drop);
    end
  endtask

  // -----------------------------
  // main
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_pix_xy_feature_top_v2.vcd");
    $dumpvars(0, tb_pix_xy_feature_top_v2);

    rst = 1'b1;
    en  = 1'b0;

    pix_valid = 1'b0;
    sof = 0; eol = 0; eof = 0;
    y_in = '0;

    feat_ready = 1'b1;
    got_tiles  = 0;

    repeat (5) tick();
    rst = 1'b0;
    en  = 1'b1;

    $display("[RUN] drive %0dx%0d pixels; expect %0d tiles", ACTIVE_W, ACTIVE_H, EXP_TILES);

    // 1) Start sink (default: bp=0 for debug stability; change to 1 after it works)
    fork
      sink_task(1'b0);
    join_none

    // 2) Drive frame with timeout so TB never hangs here
    drive_frame_with_timeout(200);

    // 3) Drain tiles (always force ready=1 here)
    drain_until_done(EXP_TILES, 200);

    $display("[TB] done: got_tiles=%0d/%0d join_ok=%0d mismatch=%0d drop=%0d",
             got_tiles, EXP_TILES, cnt_join_ok, cnt_mismatch, cnt_drop);

    if (got_tiles != EXP_TILES) $fatal(1, "FAIL: got_tiles=%0d expected=%0d", got_tiles, EXP_TILES);

    $display("PASS");
    $finish;
  end

endmodule

`default_nettype wire
