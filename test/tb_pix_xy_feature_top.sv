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

  localparam int unsigned TILES_X = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y = (ACTIVE_H >> TILE_SHIFT); // 2
  localparam int unsigned EXP_TILES = TILES_X * TILES_Y;      // 8

  localparam int unsigned YPIX_W = 8;
  localparam int unsigned FEAT_W = 16;
  localparam int unsigned FEAT_DIM = 4;

  localparam int unsigned TILE_ID_W = (EXP_TILES <= 1) ? 1 : $clog2(EXP_TILES);

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 0;
  always #5 clk = ~clk;

  logic rst;
  logic en;

  // -----------------------------
  // upstream
  // -----------------------------
  logic              pix_valid;
  wire               pix_ready;
  logic              sof, eol, eof;
  logic [YPIX_W-1:0]  y_in;

  // -----------------------------
  // downstream
  // -----------------------------
  wire                  feat_valid;
  logic                 feat_ready;

  wire [15:0]           tile_i_o;
  wire [15:0]           tile_j_o;
  wire [TILE_ID_W-1:0]  tile_id_o;

  wire [FEAT_W-1:0]     feat0, feat1, feat2, feat3;
  wire [8*FEAT_W-1:0]   feat_vec;

  wire err_mismatch_pulse;
  wire [31:0] cnt_join_ok, cnt_mismatch, cnt_drop;

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
    .MATCH_MODE_BUFFERED(1'b0) // strict is ok because bundle is aligned
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
  task automatic tick;
    @(posedge clk);
  endtask

  function automatic logic [YPIX_W-1:0] gen_pix(input int xx, input int yy);
    gen_pix = (xx + (yy<<4)) & 8'hFF;
  endfunction

  // ============================================================
  // SOURCE (IMPORTANT): sof/eof must align to ACCEPTED pixel
  // - valid stays high + data stable until ready
  // - sof asserted only on the first ACCEPTED pixel
  // - eof asserted only on the last ACCEPTED pixel (if you use eof)
  // ============================================================
  task automatic drive_frame_handshake;
    int xx, yy;
    bit sof_pending;
    begin
      // idle defaults
      pix_valid <= 1'b0;
      sof       <= 1'b0;
      eol       <= 1'b0;
      eof       <= 1'b0;
      y_in      <= '0;

      tick();

      xx = 0;
      yy = 0;
      sof_pending = 1'b1;

      // present first beat
      pix_valid <= 1'b1;
      y_in      <= gen_pix(xx, yy);

      // sof/eof/eol are COMBINED with current coordinates,
      // but they are only meaningful when handshake happens.
      while (yy < ACTIVE_H) begin
        // drive control for CURRENT presented pixel
        sof <= sof_pending;
        eol <= 1'b0; // your DUT USE_EOL=0 by default; keep 0
        eof <= (xx == (ACTIVE_W-1)) && (yy == (ACTIVE_H-1));

        tick();

        if (pix_valid && pix_ready) begin
          // accepted this pixel
          if (sof_pending) sof_pending = 1'b0;

          // advance to next pixel
          xx++;
          if (xx == ACTIVE_W) begin
            xx = 0;
            yy++;
          end

          if (yy < ACTIVE_H) begin
            // present next pixel (and hold until accepted)
            y_in <= gen_pix(xx, yy);
          end else begin
            // done
            pix_valid <= 1'b0;
            sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
            y_in <= '0;
          end
        end
        // else: HOLD EVERYTHING (pix_valid=1 and y_in/sof/eof stable)
      end
    end
  endtask

  // ============================================================
  // SINK with optional backpressure (bp)
  // ============================================================
  task automatic run_sink(input bit bp_enable);
    begin
      feat_ready <= 1'b1;
      forever begin
        tick();
        if (bp_enable) begin
          // 75% ready, 25% stall
          feat_ready <= ($urandom_range(0,3) != 0);
        end else begin
          feat_ready <= 1'b1;
        end
      end
    end
  endtask

  int got_tiles;
  always @(posedge clk) if (!rst) begin
    if (feat_valid && feat_ready) begin
      got_tiles++;
      $display("[TB] GOT feat tile_i=%0d tile_j=%0d tile_id=%0d f0=%0d f1=%0d f2=%0d f3=%0d",
               tile_i_o, tile_j_o, tile_id_o, feat0, feat1, feat2, feat3);
    end
  end

  // ============================================================
  // DRAIN: after source finished, force ready=1 to avoid "random stall forever"
  // ============================================================
  task automatic drain_until_done(input int exp_tiles, input int max_cycles);
    int c;
    begin
      feat_ready <= 1'b1; // force drain
      for (c=0; c<max_cycles; c++) begin
        tick();
        if (got_tiles >= exp_tiles) begin
          disable drain_until_done;
        end
      end
      $fatal(1, "[TB] DRAIN TIMEOUT got_tiles=%0d/%0d", got_tiles, exp_tiles);
    end
  endtask

  // -----------------------------
  // main
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_pix_xy_feature_top.vcd");
    $dumpvars(0, tb_pix_xy_feature_top);

    rst = 1'b1;
    en  = 1'b0;

    pix_valid = 1'b0;
    sof = 0; eol = 0; eof = 0;
    y_in = '0;

    feat_ready = 1'b1;
    got_tiles = 0;

    repeat (5) tick();
    rst = 1'b0;
    en  = 1'b1;

    $display("[RUN] drive %0dx%0d pixels; expect %0d tiles; bp=1",
             ACTIVE_W, ACTIVE_H, EXP_TILES);

    fork
      run_sink(1'b0);   // backpressure during streaming
    join_none

    // drive pixels with correct handshake
    drive_frame_handshake();

    // after driving finished, force drain (no bp) until all tiles out
    drain_until_done(EXP_TILES, 200000);

    $display("[TB] done: got_tiles=%0d/%0d join_ok=%0d mismatch=%0d drop=%0d",
             got_tiles, EXP_TILES, cnt_join_ok, cnt_mismatch, cnt_drop);

    if (got_tiles != EXP_TILES) begin
      $fatal(1, "FAIL: got_tiles=%0d expected=%0d", got_tiles, EXP_TILES);
    end

    $display("PASS");
    $finish;
  end

endmodule

`default_nettype wire
