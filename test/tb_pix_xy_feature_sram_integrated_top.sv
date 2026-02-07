// ============================================================
// tb_pix_xy_feature_sram_integrated_top.sv
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_pix_xy_feature_sram_integrated_top.vvp ./test/tb_pix_xy_feature_sram_integrated_top.sv
//
// Run:
// vvp ./vvp/tb_pix_xy_feature_sram_integrated_top.vvp
// gtkwave ./vvp/tb_pix_xy_feature_sram_integrated_top.vcd
// ============================================================

`include "./src/AMOLED/pix_xy_feature_sram_integrated_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_pix_xy_feature_sram_integrated_top;

  // ------------------------------------------------------------
  // SMALL demo params (make sim fast)
  // ------------------------------------------------------------
  localparam int unsigned ACTIVE_W   = 16;
  localparam int unsigned ACTIVE_H   = 8;

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT); // 2
  localparam int unsigned EXP_TILES  = TILES_X * TILES_Y;        // 8

  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;
  localparam int unsigned YPIX_W   = 8;

  // packer output lane width
  localparam int unsigned FEAT_W   = 16;
  // SRAM stores FEAT_DIM * elen_W
  localparam int unsigned FEAT_DIM = 8;   // must match your integrated_top
  localparam int unsigned elen_W   = 32;
  localparam int unsigned TAG_W    = 16;

  // ------------------------------------------------------------
  // clock/reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;
  logic en;

  // ------------------------------------------------------------
  // pixel stream in
  // ------------------------------------------------------------
  logic              pix_valid;
  wire               pix_ready;
  logic              sof, eol, eof;
  logic [YPIX_W-1:0]  y_in;

  // ------------------------------------------------------------
  // external read port (debug/verify)
  // ------------------------------------------------------------
  logic                      valid_rd;
  wire                       ready_rd;
  logic [$clog2(TILES_Y)-1:0] tile_i_rd;
  logic [$clog2(TILES_X)-1:0] tile_j_rd;
  logic [TAG_W-1:0]           tag_rd;

  // ------------------------------------------------------------
  // feature out from SRAM (readback)
  // ------------------------------------------------------------
  wire                        feat_out_valid;
  logic                       feat_out_ready;
  wire [TAG_W-1:0]            feat_out_tag;
  wire [FEAT_DIM*elen_W-1:0]  feat_out_data;

  // ------------------------------------------------------------
  // write-side activity
  // ------------------------------------------------------------
  wire                        wr_fire;
  wire [15:0]                 wr_tile_i;
  wire [15:0]                 wr_tile_j;

  // ------------------------------------------------------------
  // packer debug
  // ------------------------------------------------------------
  wire err_mismatch_pulse;
  wire [31:0] cnt_join_ok, cnt_mismatch, cnt_drop;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  pix_xy_feature_sram_integrated_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .YPIX_W(YPIX_W),

    .FEAT_W(FEAT_W),
    .FEAT_DIM(FEAT_DIM),

    .elen_W(elen_W),
    .tag_w(TAG_W),

    .MATCH_MODE_BUFFERED(1'b1),
    .USE_EOL(1'b0),
    .USE_EOF(1'b0),

    .RD_LAT(2)
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

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data),

    .wr_fire(wr_fire),
    .wr_tile_i(wr_tile_i),
    .wr_tile_j(wr_tile_j),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_pix_xy_feature_sram_integrated_top.vcd");
    $dumpvars(0, tb_pix_xy_feature_sram_integrated_top);
  end

  // ------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------
  task automatic tick; @(posedge clk); endtask

  function automatic logic [YPIX_W-1:0] gen_pix(input int xx, input int yy);
    gen_pix = (xx + (yy<<4)) & 'hFF;
  endfunction

  // ------------------------------------------------------------
  // Count writes (tiles stored)
  // ------------------------------------------------------------
  int wr_cnt;
  always_ff @(posedge clk) begin
    if (rst) wr_cnt <= 0;
    else if (wr_fire) begin
      wr_cnt <= wr_cnt + 1;
      $display("[WR] fire ti=%0d tj=%0d wr_cnt=%0d t=%0t",
               wr_tile_i, wr_tile_j, wr_cnt+1, $time);
    end
  end

  // ------------------------------------------------------------
  // feat_out sink: keep ready high (stable debug)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) feat_out_ready <= 1'b0;
    else     feat_out_ready <= 1'b1;
  end

  // ------------------------------------------------------------
  // DEBUG dump on readback HS
  // ------------------------------------------------------------
  integer k;
  always @(posedge clk) begin
    if (!rst && feat_out_valid && feat_out_ready) begin
      $display("[RDRET] tag=0x%h t=%0t", feat_out_tag, $time);
      for (k = 0; k < FEAT_DIM; k = k + 1) begin
        $display("  dim[%0d]=0x%08x", k, feat_out_data[k*elen_W +: elen_W]);
      end
    end
  end

  // ------------------------------------------------------------
  // SOURCE: drive a full frame with timeout (never hangs)
  // - hold pix_valid and sideband stable until pix_ready
  // ------------------------------------------------------------
  task automatic drive_frame_with_timeout(input int max_cycles);
    int xx, yy, cyc;
    bit fire;
    begin
      pix_valid <= 1'b0;
      sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
      y_in <= '0;
      tick();

      xx = 0; yy = 0; cyc = 0;

      pix_valid <= 1'b1;
      y_in      <= gen_pix(xx, yy);
      sof       <= 1'b1;
      eol       <= (xx == ACTIVE_W-1);
      eof       <= (xx == ACTIVE_W-1) && (yy == ACTIVE_H-1);

      while (yy < ACTIVE_H) begin
        tick();
        cyc++;

        fire = pix_valid && pix_ready;
        if (fire) begin
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
            pix_valid <= 1'b0;
            sof <= 1'b0; eol <= 1'b0; eof <= 1'b0;
            y_in <= '0;
          end
        end

        if (cyc > max_cycles) begin
          $fatal(1, "[TB] SOURCE TIMEOUT yy=%0d xx=%0d pix_ready=%0b wr_cnt=%0d join_ok=%0d mismatch=%0d drop=%0d",
                 yy, xx, pix_ready, wr_cnt, cnt_join_ok, cnt_mismatch, cnt_drop);
        end
      end
    end
  endtask

  // ------------------------------------------------------------
  // Wait until we have seen expected number of writes
  // ------------------------------------------------------------
  task automatic wait_wr_cnt(input int exp, input int max_cycles);
    int c;
    begin
      for (c=0; c<max_cycles; c++) begin
        tick();
        if (wr_cnt >= exp) disable wait_wr_cnt;
      end
      $fatal(1, "[TB] wait_wr_cnt TIMEOUT wr_cnt=%0d exp=%0d", wr_cnt, exp);
    end
  endtask

  // ------------------------------------------------------------
  // READ request one-shot (hold until ready_rd)
  // ------------------------------------------------------------
  task automatic rd_req_once(
    input int unsigned ti,
    input int unsigned tj,
    input logic [TAG_W-1:0] tag
  );
    int t;
    begin
      tile_i_rd <= ti[$clog2(TILES_Y)-1:0];
      tile_j_rd <= tj[$clog2(TILES_X)-1:0];
      tag_rd    <= tag;
      valid_rd  <= 1'b1;

      t = 0;
      while (!(valid_rd && ready_rd)) begin
        tick();
        t++;
        if (t > 5000) begin
          $fatal(1, "[TB] rd_req_once TIMEOUT ti=%0d tj=%0d", ti, tj);
        end
      end

      tick();
      valid_rd <= 1'b0;
    end
  endtask

  // ------------------------------------------------------------
  // Wait one read-return handshake (feat_out_valid&&ready)
  // ------------------------------------------------------------
  task automatic wait_one_ret(input int max_cycles);
    int c;
    begin
      for (c=0; c<max_cycles; c++) begin
        tick();
        if (feat_out_valid && feat_out_ready) disable wait_one_ret;
      end
      $fatal(1, "[TB] wait_one_ret TIMEOUT");
    end
  endtask

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  int ti, tj;
  logic [TAG_W-1:0] tag_base;

  initial begin
    // init
    rst = 1'b1;
    en  = 1'b0;

    pix_valid = 1'b0;
    sof = 0; eol = 0; eof = 0;
    y_in = '0;

    valid_rd = 1'b0;
    tile_i_rd = '0;
    tile_j_rd = '0;
    tag_rd = '0;

    wr_cnt = 0;

    repeat (5) tick();
    rst = 1'b0;
    en  = 1'b1;

    $display("[RUN] ACTIVE %0dx%0d, tiles %0dx%0d => exp %0d tiles",
             ACTIVE_W, ACTIVE_H, TILES_X, TILES_Y, EXP_TILES);

    // 1) Drive frame -> expect writes to SRAM per-tile
    drive_frame_with_timeout(2000);

    // 2) Wait until all tiles have been written
    wait_wr_cnt(EXP_TILES, 20000);
    $display("[TB] write phase done: wr_cnt=%0d join_ok=%0d mismatch=%0d drop=%0d",
             wr_cnt, cnt_join_ok, cnt_mismatch, cnt_drop);

    // 3) Read back all tiles one-by-one
    tag_base = 16'h9000;
    for (ti = 0; ti < TILES_Y; ti = ti + 1) begin
      for (tj = 0; tj < TILES_X; tj = tj + 1) begin
        $display("== RD tile (%0d,%0d) ==", ti, tj);
        rd_req_once(ti, tj, tag_base + (ti*TILES_X + tj));
        wait_one_ret(20000);

        // basic sanity: tag should match
        if (feat_out_tag !== (tag_base + (ti*TILES_X + tj))) begin
          $fatal(1, "[TB] TAG mismatch exp=0x%h got=0x%h",
                 (tag_base + (ti*TILES_X + tj)), feat_out_tag);
        end
      end
    end

    $display("PASS ✅ tb_pix_xy_feature_sram_integrated_top");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
