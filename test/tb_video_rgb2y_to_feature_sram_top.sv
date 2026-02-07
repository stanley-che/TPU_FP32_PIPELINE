// ============================================================
// tb_video_rgb2y_to_feature_sram_top.sv  (X-safe + AUTO-PROBE + DIAG)
// FIXED: VSYNC is now a "frame-gate level" (NOT a short pulse)
//
// Why:
//   active_window_tracker defines in_frame as the interval between
//   vs_rise and vs_fall (FRAME_START_ON_VS_RISE=1).
//   If vsync is a short pulse, in_frame lasts only a few cycles -> pix_valid=0 forever.
//   Therefore TB must hold vsync high for the whole frame, then drop it.
//
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_video_rgb2y_to_feature_sram_top.vvp ./test/tb_video_rgb2y_to_feature_sram_top.sv
//
// Run:
// vvp ./vvp/tb_video_rgb2y_to_feature_sram_top.vvp
// ============================================================

`include "./src/AMOLED/video_rgb2y_to_feature_sram_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_video_rgb2y_to_feature_sram_top;

  // ----------------------------
  // Basic params
  // ----------------------------
  integer TCLK_NS; initial TCLK_NS = 10;

  localparam int unsigned ACTIVE_W   = 32;
  localparam int unsigned ACTIVE_H   = 16;

  localparam int unsigned TILE_SHIFT = 2;
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT);
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT);

  localparam int unsigned X_W        = 11;
  localparam int unsigned Y_W        = 10;

  localparam int unsigned PACK_OUT_W = 10;
  localparam int unsigned YPIX_W     = 8;

  localparam int unsigned FEAT_DIM   = 8;
  localparam int unsigned elen_W     = 32;
  localparam int unsigned tag_w      = 16;

  // ----------------------------
  // TB signals
  // ----------------------------
  reg  clk, rst;

  // select ONE of 3 DUTs by enabling only one `en_*`
  reg  en0, en1, en2;

  reg  vsync_i, hsync_i, de_i;
  reg  [23:0] rgb_i;

  reg  clip_en;
  reg  [PACK_OUT_W-1:0] clip_min, clip_max;

  reg                        valid_rd;
  wire                       ready_rd_0, ready_rd_1, ready_rd_2;
  reg  [$clog2(TILES_Y)-1:0] tile_i_rd;
  reg  [$clog2(TILES_X)-1:0] tile_j_rd;
  reg  [tag_w-1:0]           tag_rd;

  // outputs from three DUTs (we mux based on selected)
  wire feat_out_valid_0, feat_out_valid_1, feat_out_valid_2;
  wire feat_out_ready_0, feat_out_ready_1, feat_out_ready_2; // driven by TB (fanout)
  wire [tag_w-1:0] feat_out_tag_0, feat_out_tag_1, feat_out_tag_2;
  wire [FEAT_DIM*elen_W-1:0] feat_out_data_0, feat_out_data_1, feat_out_data_2;

  wire wr_fire_0, wr_fire_1, wr_fire_2;
  wire [15:0] wr_tile_i_0, wr_tile_i_1, wr_tile_i_2;
;
  wire [15:0] wr_tile_j_0, wr_tile_j_1, wr_tile_j_2;

  wire rgb2y_out_valid_0, rgb2y_out_valid_1, rgb2y_out_valid_2;
  wire rgb2y_out_ready_0, rgb2y_out_ready_1, rgb2y_out_ready_2;
  wire [PACK_OUT_W-1:0] rgb2y_Y_0, rgb2y_Y_1, rgb2y_Y_2;
  wire [(4+3+Y_W+X_W)-1:0] rgb2y_sb_out_0, rgb2y_sb_out_1, rgb2y_sb_out_2;

  wire in_frame_0, in_frame_1, in_frame_2;
  wire in_line_0,  in_line_1,  in_line_2;
  wire pix_valid_timing_0, pix_valid_timing_1, pix_valid_timing_2;

  wire [X_W-1:0] x0, x1, x2;
  wire [Y_W-1:0] y0, y1, y2;

  wire err_mismatch_pulse_0, err_mismatch_pulse_1, err_mismatch_pulse_2;
  wire [31:0] cnt_join_ok_0, cnt_join_ok_1, cnt_join_ok_2;
  wire [31:0] cnt_mismatch_0, cnt_mismatch_1, cnt_mismatch_2;
  wire [31:0] cnt_drop_0, cnt_drop_1, cnt_drop_2;

  wire skid_drop_pulse_0, skid_drop_pulse_1, skid_drop_pulse_2;
  wire [31:0] skid_drop_cnt_0, skid_drop_cnt_1, skid_drop_cnt_2;

  // TB-controlled ready (single knob, fanned out)
  reg feat_out_ready;

  assign feat_out_ready_0 = feat_out_ready;
  assign feat_out_ready_1 = feat_out_ready;
  assign feat_out_ready_2 = feat_out_ready;

  // ready_rd mux (only one DUT enabled at a time)
  wire ready_rd = (en0) ? ready_rd_0 :
                  (en1) ? ready_rd_1 :
                  (en2) ? ready_rd_2 : 1'b0;

  // mux “selected” outputs
  wire feat_out_valid = (en0) ? feat_out_valid_0 :
                        (en1) ? feat_out_valid_1 :
                        (en2) ? feat_out_valid_2 : 1'b0;

  wire [tag_w-1:0] feat_out_tag = (en0) ? feat_out_tag_0 :
                                  (en1) ? feat_out_tag_1 :
                                  (en2) ? feat_out_tag_2 : '0;

  wire [FEAT_DIM*elen_W-1:0] feat_out_data = (en0) ? feat_out_data_0 :
                                             (en1) ? feat_out_data_1 :
                                             (en2) ? feat_out_data_2 : '0;

  wire wr_fire = (en0) ? wr_fire_0 :
                 (en1) ? wr_fire_1 :
                 (en2) ? wr_fire_2 : 1'b0;

  wire [15:0] wr_tile_i = (en0) ? wr_tile_i_0 :
                          (en1) ? wr_tile_i_1 :
                          (en2) ? wr_tile_i_2 : '0;

  wire [15:0] wr_tile_j = (en0) ? wr_tile_j_0 :
                          (en1) ? wr_tile_j_1 :
                          (en2) ? wr_tile_j_2 : '0;

  wire rgb2y_out_valid = (en0) ? rgb2y_out_valid_0 :
                         (en1) ? rgb2y_out_valid_1 :
                         (en2) ? rgb2y_out_valid_2 : 1'b0;

  wire rgb2y_out_ready = (en0) ? rgb2y_out_ready_0 :
                         (en1) ? rgb2y_out_ready_1 :
                         (en2) ? rgb2y_out_ready_2 : 1'b0;

  wire [(4+3+Y_W+X_W)-1:0] rgb2y_sb_out = (en0) ? rgb2y_sb_out_0 :
                                          (en1) ? rgb2y_sb_out_1 :
                                          (en2) ? rgb2y_sb_out_2 : '0;

  wire in_frame = (en0) ? in_frame_0 :
                  (en1) ? in_frame_1 :
                  (en2) ? in_frame_2 : 1'b0;

  wire in_line  = (en0) ? in_line_0 :
                  (en1) ? in_line_1 :
                  (en2) ? in_line_2 : 1'b0;

  wire pix_valid_timing = (en0) ? pix_valid_timing_0 :
                          (en1) ? pix_valid_timing_1 :
                          (en2) ? pix_valid_timing_2 : 1'b0;

  wire [X_W-1:0] x = (en0) ? x0 : (en1) ? x1 : (en2) ? x2 : '0;
  wire [Y_W-1:0] y = (en0) ? y0 : (en1) ? y1 : (en2) ? y2 : '0;

  wire [31:0] cnt_join_ok = (en0) ? cnt_join_ok_0 :
                            (en1) ? cnt_join_ok_1 :
                            (en2) ? cnt_join_ok_2 : 32'd0;

  wire [31:0] cnt_mismatch = (en0) ? cnt_mismatch_0 :
                             (en1) ? cnt_mismatch_1 :
                             (en2) ? cnt_mismatch_2 : 32'd0;

  wire [31:0] cnt_drop = (en0) ? cnt_drop_0 :
                         (en1) ? cnt_drop_1 :
                         (en2) ? cnt_drop_2 : 32'd0;

  wire [31:0] skid_drop_cnt = (en0) ? skid_drop_cnt_0 :
                              (en1) ? skid_drop_cnt_1 :
                              (en2) ? skid_drop_cnt_2 : 32'd0;

  wire err_mismatch_pulse = (en0) ? err_mismatch_pulse_0 :
                            (en1) ? err_mismatch_pulse_1 :
                            (en2) ? err_mismatch_pulse_2 : 1'b0;

  wire skid_drop_pulse = (en0) ? skid_drop_pulse_0 :
                         (en1) ? skid_drop_pulse_1 :
                         (en2) ? skid_drop_pulse_2 : 1'b0;

  // ----------------------------
  // Instantiate 3 DUT variants
  // ----------------------------
  // Notes:
  // - Force ROI enabled full-frame to avoid gating surprises
  // - Disable bounds/in_frame gating for bringup
  // - Keep USE_EOL/USE_EOF off (same as your X-safe TB)
  //
  // mode0: DE_INV=0
  video_rgb2y_to_feature_sram_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(elen_W), .tag_w(tag_w),

    .DE_INV(1'b0),
    .USE_HSYNC_FOR_EOL(1'b0),
    .GATE_BY_IN_FRAME(1'b0),
    .ENABLE_BOUNDS(1'b0),

    .ROI_EN_DEFAULT(1'b1),
    .ROI_X0_DEFAULT(0), .ROI_Y0_DEFAULT(0),
    .ROI_X1_DEFAULT(ACTIVE_W-1), .ROI_Y1_DEFAULT(ACTIVE_H-1),

    .USE_SOF(1'b1), .USE_EOL(1'b0), .USE_EOF(1'b0),
    .ENABLE_ASSERT(1'b0), .ASSERT_ON_STATS(1'b0), .ASSERT_ON_EDGE(1'b0)
  ) dut0 (
    .clk(clk), .rst(rst), .en(en0),

    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i), .rgb_i(rgb_i),

    .clip_en(clip_en), .clip_min(clip_min), .clip_max(clip_max),

    .valid_rd(valid_rd), .ready_rd(ready_rd_0),
    .tile_i_rd(tile_i_rd), .tile_j_rd(tile_j_rd), .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid_0), .feat_out_ready(feat_out_ready_0),
    .feat_out_tag(feat_out_tag_0), .feat_out_data(feat_out_data_0),

    .wr_fire(wr_fire_0), .wr_tile_i(wr_tile_i_0), .wr_tile_j(wr_tile_j_0),

    .rgb2y_out_valid(rgb2y_out_valid_0), .rgb2y_out_ready(rgb2y_out_ready_0),
    .rgb2y_Y(rgb2y_Y_0), .rgb2y_sb_out(rgb2y_sb_out_0),

    .in_frame(in_frame_0), .in_line(in_line_0), .pix_valid_timing(pix_valid_timing_0),
    .x(x0), .y(y0),

    .skid_drop_pulse(skid_drop_pulse_0), .skid_drop_cnt(skid_drop_cnt_0),

    .err_mismatch_pulse(err_mismatch_pulse_0),
    .cnt_join_ok(cnt_join_ok_0), .cnt_mismatch(cnt_mismatch_0), .cnt_drop(cnt_drop_0)
  );

  // mode1: DE_INV=1
  video_rgb2y_to_feature_sram_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(elen_W), .tag_w(tag_w),

    .DE_INV(1'b1),
    .USE_HSYNC_FOR_EOL(1'b0),
    .GATE_BY_IN_FRAME(1'b0),
    .ENABLE_BOUNDS(1'b0),

    .ROI_EN_DEFAULT(1'b1),
    .ROI_X0_DEFAULT(0), .ROI_Y0_DEFAULT(0),
    .ROI_X1_DEFAULT(ACTIVE_W-1), .ROI_Y1_DEFAULT(ACTIVE_H-1),

    .USE_SOF(1'b1), .USE_EOL(1'b0), .USE_EOF(1'b0),
    .ENABLE_ASSERT(1'b0), .ASSERT_ON_STATS(1'b0), .ASSERT_ON_EDGE(1'b0)
  ) dut1 (
    .clk(clk), .rst(rst), .en(en1),

    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i), .rgb_i(rgb_i),

    .clip_en(clip_en), .clip_min(clip_min), .clip_max(clip_max),

    .valid_rd(valid_rd), .ready_rd(ready_rd_1),
    .tile_i_rd(tile_i_rd), .tile_j_rd(tile_j_rd), .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid_1), .feat_out_ready(feat_out_ready_1),
    .feat_out_tag(feat_out_tag_1), .feat_out_data(feat_out_data_1),

    .wr_fire(wr_fire_1), .wr_tile_i(wr_tile_i_1), .wr_tile_j(wr_tile_j_1),

    .rgb2y_out_valid(rgb2y_out_valid_1), .rgb2y_out_ready(rgb2y_out_ready_1),
    .rgb2y_Y(rgb2y_Y_1), .rgb2y_sb_out(rgb2y_sb_out_1),

    .in_frame(in_frame_1), .in_line(in_line_1), .pix_valid_timing(pix_valid_timing_1),
    .x(x1), .y(y1),

    .skid_drop_pulse(skid_drop_pulse_1), .skid_drop_cnt(skid_drop_cnt_1),

    .err_mismatch_pulse(err_mismatch_pulse_1),
    .cnt_join_ok(cnt_join_ok_1), .cnt_mismatch(cnt_mismatch_1), .cnt_drop(cnt_drop_1)
  );

  // mode2: HS mode
  video_rgb2y_to_feature_sram_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(elen_W), .tag_w(tag_w),

    .DE_INV(1'b0),
    .USE_HSYNC_FOR_EOL(1'b1),
    .GATE_BY_IN_FRAME(1'b0),
    .ENABLE_BOUNDS(1'b0),

    .ROI_EN_DEFAULT(1'b1),
    .ROI_X0_DEFAULT(0), .ROI_Y0_DEFAULT(0),
    .ROI_X1_DEFAULT(ACTIVE_W-1), .ROI_Y1_DEFAULT(ACTIVE_H-1),

    .USE_SOF(1'b1), .USE_EOL(1'b0), .USE_EOF(1'b0),
    .ENABLE_ASSERT(1'b0), .ASSERT_ON_STATS(1'b0), .ASSERT_ON_EDGE(1'b0)
  ) dut2 (
    .clk(clk), .rst(rst), .en(en2),

    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i), .rgb_i(rgb_i),

    .clip_en(clip_en), .clip_min(clip_min), .clip_max(clip_max),

    .valid_rd(valid_rd), .ready_rd(ready_rd_2),
    .tile_i_rd(tile_i_rd), .tile_j_rd(tile_j_rd), .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid_2), .feat_out_ready(feat_out_ready_2),
    .feat_out_tag(feat_out_tag_2), .feat_out_data(feat_out_data_2),

    .wr_fire(wr_fire_2), .wr_tile_i(wr_tile_i_2), .wr_tile_j(wr_tile_j_2),

    .rgb2y_out_valid(rgb2y_out_valid_2), .rgb2y_out_ready(rgb2y_out_ready_2),
    .rgb2y_Y(rgb2y_Y_2), .rgb2y_sb_out(rgb2y_sb_out_2),

    .in_frame(in_frame_2), .in_line(in_line_2), .pix_valid_timing(pix_valid_timing_2),
    .x(x2), .y(y2),

    .skid_drop_pulse(skid_drop_pulse_2), .skid_drop_cnt(skid_drop_cnt_2),

    .err_mismatch_pulse(err_mismatch_pulse_2),
    .cnt_join_ok(cnt_join_ok_2), .cnt_mismatch(cnt_mismatch_2), .cnt_drop(cnt_drop_2)
  );

  // ----------------------------
  // Clock
  // ----------------------------
  initial begin
    clk = 1'b0;
    forever #(TCLK_NS/2) clk = ~clk;
  end

  // ----------------------------
  // Reset
  // ----------------------------
initial begin
  // ====== X-safe init ======
  vsync_i   = 1'b0;
  hsync_i   = 1'b0;
  de_i      = 1'b0;
  rgb_i     = 24'h0;

  clip_en   = 1'b0;
  clip_min  = '0;
  clip_max  = {PACK_OUT_W{1'b1}}; // max

  valid_rd  = 1'b0;
  tile_i_rd = '0;
  tile_j_rd = '0;
  tag_rd    = '0;

  feat_out_ready = 1'b0;

  rst = 1'b1;

  // reset 期間把三顆都打開
  en0 = 1'b1; en1 = 1'b1; en2 = 1'b1;

  repeat (10) @(posedge clk);
  rst = 1'b0;

  // reset 結束後再關掉
  en0 = 1'b0; en1 = 1'b0; en2 = 1'b0;
end


  // ----------------------------
  // Dump
  // ----------------------------
  initial begin
    $dumpfile("tb_video_rgb2y_to_feature_sram_top.vcd");
    $dumpvars(0, tb_video_rgb2y_to_feature_sram_top);
  end

  // ============================================================
  // Video generator
  // FIXED: vsync_i is held HIGH for the whole frame duration.
  // ============================================================
  localparam int unsigned HBLANK = 4;
  localparam int unsigned VBLANK = 8;  // give some extra blank cycles

  // HS pulse width (still ok if your path uses de_rise/fall only)
  localparam int unsigned HS_PW   = 2;

  // small setup gap after vsync edges
  localparam int unsigned VS_EDGE_GAP = 2;

  task drive_one_frame(input integer frame_id);
  integer yy, xx;
  begin
    // frame start
    @(negedge clk);
    de_i    = 1'b0;
    hsync_i = 1'b0;
    rgb_i   = 24'h0;

    vsync_i = 1'b1;                 // VS 上升緣
    repeat (VS_EDGE_GAP) @(negedge clk);

    for (yy = 0; yy < ACTIVE_H; yy++) begin
      // HS pulse
      @(negedge clk);
      de_i    = 1'b0;
      rgb_i   = 24'h0;
      hsync_i = 1'b1;
      repeat (HS_PW) @(negedge clk);

      @(negedge clk);
      hsync_i = 1'b0;

      // active pixels
      for (xx = 0; xx < ACTIVE_W; xx++) begin
        @(negedge clk);
        de_i = 1'b1;
        rgb_i[23:16] = (xx*8) & 8'hFF;
        rgb_i[15: 8] = (yy*16) & 8'hFF;
        rgb_i[ 7: 0] = (frame_id*64 + xx) & 8'hFF;
      end

      // end of line blank
      @(negedge clk);
      de_i  = 1'b0;
      rgb_i = 24'h0;
      repeat (HBLANK) @(negedge clk);
    end

    // frame end
    @(negedge clk);
    de_i    = 1'b0;
    hsync_i = 1'b0;
    rgb_i   = 24'h0;

    vsync_i = 1'b0;                 // VS 下降緣
    repeat (VS_EDGE_GAP) @(negedge clk);

    repeat (VBLANK) @(negedge clk);
  end
endtask


  // ============================================================
  // Random backpressure (same style)
  // ============================================================
  integer r;
  always @(posedge clk) begin
    if (rst) feat_out_ready <= 1'b0;
    else     feat_out_ready <= 1'b1; 
  end

  // ============================================================
  // Track which tiles have been written
  // ============================================================
  reg written [0:TILES_Y-1][0:TILES_X-1];
  integer ii, jj;

  always @(posedge clk) begin
    if (rst) begin
      for (ii = 0; ii < TILES_Y; ii = ii + 1)
        for (jj = 0; jj < TILES_X; jj = jj + 1)
          written[ii][jj] <= 1'b0;
    end else if (wr_fire) begin
      if (wr_tile_i < TILES_Y && wr_tile_j < TILES_X)
        written[wr_tile_i][wr_tile_j] <= 1'b1;
    end
  end

  reg any_written;
  always @(*) begin
    any_written = 1'b0;
    for (ii = 0; ii < TILES_Y; ii = ii + 1)
      for (jj = 0; jj < TILES_X; jj = jj + 1)
        if (written[ii][jj]) any_written = 1'b1;
  end

  // ============================================================
  // Random read requests (ONLY read written tiles)
  // ============================================================
  reg req_pending;
  integer rr, tries, ti, tj;
integer dbg_n;
always @(posedge clk) begin
  if (rst) dbg_n <= 0;
  else begin
    if (pix_valid_timing && (dbg_n < 20)) begin
      $display("[%0t] pv=%b yv=%b yr=%b in_frame=%b in_line=%b x=%0d y=%0d de=%b vs=%b hs=%b",
               $time, pix_valid_timing, rgb2y_out_valid, rgb2y_out_ready,
               in_frame, in_line, x, y, de_i, vsync_i, hsync_i);
      dbg_n <= dbg_n + 1;
    end
  end
end

  always @(posedge clk) begin
    if (rst) begin
      valid_rd    <= 1'b0;
      req_pending <= 1'b0;
      tile_i_rd   <= '0;
      tile_j_rd   <= '0;
      tag_rd      <= '0;
    end else begin
      if (!req_pending) begin
        rr = $random;
        if (any_written && ((rr % 20) == 0)) begin // ~5%
          ti = 0; tj = 0;
          for (tries = 0; tries < 50; tries = tries + 1) begin
            ti = $random % TILES_Y;
            tj = $random % TILES_X;
            if (written[ti][tj]) tries = 999; // break
          end

          if (written[ti][tj]) begin
            tile_i_rd   <= ti[$clog2(TILES_Y)-1:0];
            tile_j_rd   <= tj[$clog2(TILES_X)-1:0];
            tag_rd      <= $random;
            valid_rd    <= 1'b1;
            req_pending <= 1'b1;
          end
        end
      end

      if (req_pending && valid_rd && ready_rd) begin
        valid_rd    <= 1'b0;
        req_pending <= 1'b0;
      end
    end
  end

  // ============================================================
  // Monitors + PROBE counters
  // ============================================================
  integer wr_count, feat_count, mismatch_pulses;

  integer yv_cnt, pix_fire_cnt;
  integer in_frame_cnt, pixv_cnt, yr_cnt;
  integer x_last, y_last;

  wire pix_fire = rgb2y_out_valid && rgb2y_out_ready;

  always @(posedge clk) begin
    if (rst) begin
      wr_count <= 0;
      feat_count <= 0;
      mismatch_pulses <= 0;

      yv_cnt <= 0;
      pix_fire_cnt <= 0;
      in_frame_cnt <= 0;
      pixv_cnt <= 0;
      yr_cnt <= 0;
      x_last <= 0;
      y_last <= 0;
    end else begin
      if (rgb2y_out_valid === 1'b1) yv_cnt <= yv_cnt + 1;
      if (pix_fire === 1'b1)        pix_fire_cnt <= pix_fire_cnt + 1;
      if (in_frame)         in_frame_cnt <= in_frame_cnt + 1;
      if (pix_valid_timing) pixv_cnt <= pixv_cnt + 1;
      if (rgb2y_out_ready)  yr_cnt <= yr_cnt + 1;

      x_last <= x;
      y_last <= y;



      if (wr_fire) begin
        if ((^dut0.feat_wr_data) === 1'bX) begin
          $display("[%0t] ERROR: feat_wr_data has X at write! tile_i=%0d tile_j=%0d",
             $time, wr_tile_i, wr_tile_j);
          $fatal(1);
        end
      end


      if (feat_out_valid && feat_out_ready) begin
        if ((^feat_out_data) === 1'bX) begin
          $display("[%0t] ERROR: feat_out_data has X  tag=%h", $time, feat_out_tag);

          // 印出每個 element（32-bit）哪個有 X
          for (int k = 0; k < FEAT_DIM; k++) begin
              logic [elen_W-1:0] w;
              w = feat_out_data[k*elen_W +: elen_W];
              if ((^w) === 1'bX) $display("  lane[%0d] = %h  (X)", k, w);
              else               $display("  lane[%0d] = %h", k, w);
          end

          $fatal(1);
        end
      end


      if (err_mismatch_pulse) begin
        mismatch_pulses <= mismatch_pulses + 1;
        $display("[%0t] WARNING: err_mismatch_pulse cnt_mismatch=%0d cnt_drop=%0d cnt_join_ok=%0d",
                 $time, cnt_mismatch, cnt_drop, cnt_join_ok);
      end

      if (skid_drop_pulse) begin
        $display("[%0t] WARNING: skid_drop_pulse skid_drop_cnt=%0d", $time, skid_drop_cnt);
      end
    end
  end

  task clear_probe_counters;
    begin
      yv_cnt = 0;
      pix_fire_cnt = 0;
      in_frame_cnt = 0;
      pixv_cnt = 0;
      yr_cnt = 0;
      x_last = 0;
      y_last = 0;
    end
  endtask

  // ============================================================
  // AUTO-PROBE
  // ============================================================
  task probe_one_mode(input integer mode_id);
    begin
      en0 = 1'b0; en1 = 1'b0; en2 = 1'b0;
      @(posedge clk);

      if (mode_id == 0) en0 = 1'b1;
      else if (mode_id == 1) en1 = 1'b1;
      else en2 = 1'b1;

      clear_probe_counters();

      drive_one_frame(99);
      repeat (50) @(posedge clk);

      $display("PROBE mode%0d: in_frame_cnt=%0d pix_valid_timing_cnt=%0d yv_cnt=%0d pix_fire_cnt=%0d yr_cnt=%0d x_last=%0d y_last=%0d",
               mode_id, in_frame_cnt, pixv_cnt, yv_cnt, pix_fire_cnt, yr_cnt, x_last, y_last);
      $display("DEBUG: yv=%b yr=%b pix_valid_timing=%b", rgb2y_out_valid, rgb2y_out_ready, pix_valid_timing);

    end
  endtask

  integer selected_mode;

  // ============================================================
  // Main
  // ============================================================
  initial begin
    wait(!rst);
    repeat (5) @(posedge clk);

    $display("=== TB start (AUTO-PROBE + X-safe reads + DIAG) ===");

    // Probe all 3 once
    probe_one_mode(0);
    probe_one_mode(1);
    probe_one_mode(2);

    // Selection policy:
    //   Prefer pix_valid_timing activity, else yv activity, else in_frame activity.
    selected_mode = -1;

    probe_one_mode(0); if (pixv_cnt > 0) selected_mode = 0;
    if (selected_mode < 0) begin probe_one_mode(1); if (pixv_cnt > 0) selected_mode = 1; end
    if (selected_mode < 0) begin probe_one_mode(2); if (pixv_cnt > 0) selected_mode = 2; end

    if (selected_mode < 0) begin
      probe_one_mode(0); if (yv_cnt > 0) selected_mode = 0;
      if (selected_mode < 0) begin probe_one_mode(1); if (yv_cnt > 0) selected_mode = 1; end
      if (selected_mode < 0) begin probe_one_mode(2); if (yv_cnt > 0) selected_mode = 2; end
    end

    if (selected_mode < 0) begin
      probe_one_mode(0); if (in_frame_cnt > 0) selected_mode = 0;
      if (selected_mode < 0) begin probe_one_mode(1); if (in_frame_cnt > 0) selected_mode = 1; end
      if (selected_mode < 0) begin probe_one_mode(2); if (in_frame_cnt > 0) selected_mode = 2; end
    end

    if (selected_mode < 0) begin
      $display("FATAL: No mode produced any timing activity (in_frame/pix_valid/yv all 0).");
      $fatal(1);
    end

    $display("=== Selected mode = %0d ===", selected_mode);

    // lock selection
    en0 = (selected_mode == 0);
    en1 = (selected_mode == 1);
    en2 = (selected_mode == 2);

    // run real frames
    drive_one_frame(0);
    drive_one_frame(1);

    repeat (400) @(posedge clk);

    $display("=== TB done wr=%0d feat=%0d mismatch_pulses=%0d cnt_join_ok=%0d cnt_mismatch=%0d cnt_drop=%0d skid_drop_cnt=%0d ===",
             wr_count, feat_count, mismatch_pulses,
             cnt_join_ok, cnt_mismatch, cnt_drop, skid_drop_cnt);

    if (mismatch_pulses != 0) $fatal(1);
    $finish;
  end

endmodule

`default_nettype wire
