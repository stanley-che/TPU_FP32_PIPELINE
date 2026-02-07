// tb_pix_xy_linebuf_top.sv
// ------------------------------------------------------------
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_pix_xy_linebuf_top.vvp ./test/tb_pixel_xy_linebuf_1line_top.sv
// Run:
//   vvp ./vvp/tb_pix_xy_linebuf_top.vvp
// Wave:
//   gtkwave ./vvp/tb_pix_xy_linebuf_top.vcd
// ------------------------------------------------------------

`include "./src/AMOLED/tile feature_extracter/pixel_xy_linebuf_1line_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_pix_xy_linebuf_top;

  // ============================================================
  // params (縮小加速)
  // ============================================================
  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 6;

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y    = 2; // ceil-ish for 6/4 coverage

  localparam int unsigned YPIX_W = 8;

  // ============================================================
  // clock/reset
  // ============================================================
  logic clk = 1'b0;
  always #5 clk = ~clk;
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
  logic [YPIX_W-1:0]    y_cur, y_left, y_up;

  logic [X_W-1:0]       x;
  logic [Y_W-1:0]       y;

  logic [X_W-1:0]       tile_j;
  logic [Y_W-1:0]       tile_i;
  logic [1:0]           x_mod, y_mod;

  logic                 tile_first, tile_last, tile_x_last, tile_y_last;
  logic                 x_last, y_last, line_start, line_last, frame_start, frame_last;

  logic [$clog2(TILES_X*TILES_Y)-1:0] tile_idx;
  logic                 in_roi;

  logic                 err_sof_midframe, err_eol_mismatch;
  logic [31:0]          frame_cnt, line_cnt;

  // ============================================================
  // DUT
  // ============================================================
  pix_xy_linebuf_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),

    .YPIX_W(YPIX_W),

    .USE_SOF(1'b1),
    .USE_EOL(1'b0),
    .USE_EOF(1'b0),
    .SAT_AT_MAX(1'b0),
    .ADVANCE_ON_VALID_ONLY(1'b1),

    .ROI_EN_DEFAULT(1'b0),
    .DBG_COUNTERS_EN(1'b1)
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
    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up),

    .x(x),
    .y(y),

    .tile_j(tile_j),
    .tile_i(tile_i),
    .x_mod(x_mod),
    .y_mod(y_mod),

    .tile_first(tile_first),
    .tile_last(tile_last),
    .tile_x_last(tile_x_last),
    .tile_y_last(tile_y_last),

    .x_last(x_last),
    .y_last(y_last),
    .line_start(line_start),
    .line_last(line_last),
    .frame_start(frame_start),
    .frame_last(frame_last),

    .tile_idx(tile_idx),
    .in_roi(in_roi),

    .err_sof_midframe(err_sof_midframe),
    .err_eol_mismatch(err_eol_mismatch),
    .frame_cnt(frame_cnt),
    .line_cnt(line_cnt)
  );

  // TRUE accept into linebuf (counter -> linebuf handshake)
  // hierarchical access: v1_valid/v1_ready are internal signals in pix_xy_linebuf_top
  wire v1_accept = dut.v1_valid && dut.v1_ready;

  // ============================================================
  // helper
  // ============================================================
  task automatic step; begin @(posedge clk); #1; end endtask

  // ============================================================
  // downstream ready pattern (backpressure)
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

  wire fire_in  = pix_valid && pix_ready;
  wire fire_out = out_valid && out_ready;

  // ============================================================
  // Golden model for linebuf (prev-line mem + prev-in-line left)
  // Update on v1_accept (TRUE accept), not fire_out.
  // ============================================================
  logic [YPIX_W-1:0] gmem [0:ACTIVE_W-1];
  logic [YPIX_W-1:0] gleft;
  integer i;

  task automatic gold_reset;
    begin
      for (i=0; i<ACTIVE_W; i=i+1) gmem[i] = '0;
      gleft = '0;
    end
  endtask

  // ============================================================
  // Scoreboard queue: expected outputs per accepted beat (v1_accept)
  // Compare only at fire_out.
  // ============================================================
  localparam int unsigned QDEPTH = 4096;
  logic [YPIX_W-1:0] q_ec [0:QDEPTH-1];
  logic [YPIX_W-1:0] q_el [0:QDEPTH-1];
  logic [YPIX_W-1:0] q_eu [0:QDEPTH-1];
  int q_wptr, q_rptr;

  task automatic q_reset;
    begin q_wptr = 0; q_rptr = 0; end
  endtask

  task automatic q_push(input logic [YPIX_W-1:0] ec,
                        input logic [YPIX_W-1:0] el,
                        input logic [YPIX_W-1:0] eu);
    begin
      if (q_wptr >= QDEPTH) $fatal(1, "[TB] queue overflow");
      q_ec[q_wptr] = ec;
      q_el[q_wptr] = el;
      q_eu[q_wptr] = eu;
      q_wptr++;
    end
  endtask

  task automatic q_pop(output logic [YPIX_W-1:0] ec,
                       output logic [YPIX_W-1:0] el,
                       output logic [YPIX_W-1:0] eu);
    begin
      if (q_rptr >= q_wptr) $fatal(1, "[TB] queue underflow");
      ec = q_ec[q_rptr];
      el = q_el[q_rptr];
      eu = q_eu[q_rptr];
      q_rptr++;
    end
  endtask

  // ============================================================
  // Input driver: hold pix_valid until accepted (fire_in)
  // ============================================================
  task automatic send_beat(input bit s, input byte val);
    int guard;
    bit accepted;
    begin
      pix_valid = 1'b1;
      sof       = s;
      eol       = 1'b0;
      eof       = 1'b0;
      y_in      = val[YPIX_W-1:0];

      guard = 0;
      accepted = 0;

      while (!accepted) begin
        step();
        cyc++;

        accepted = fire_in;

        guard++;
        if (guard > 2000) begin
          $display("[TB] TIMEOUT waiting fire_in (pix_ready=%0b out_valid=%0b out_ready=%0b)",
                   pix_ready, out_valid, out_ready);
          $fatal(1, "FAIL");
        end
      end

      // drop after accept
      pix_valid = 1'b0;
      sof       = 1'b0;
      eol       = 1'b0;
      eof       = 1'b0;
      y_in      = '0;

      // optional bubble
      step();
      cyc++;
    end
  endtask

  // ============================================================
  // HOLD check under stall
  // ============================================================
  logic [YPIX_W-1:0] hold_cur, hold_left, hold_up;
  logic hold_active;

  always_ff @(posedge clk) begin
    if (rst || !en) begin
      hold_active <= 1'b0;
      hold_cur    <= '0;
      hold_left   <= '0;
      hold_up     <= '0;
    end else begin
      if (out_valid && !out_ready) begin
        if (!hold_active) begin
          hold_active <= 1'b1;
          hold_cur    <= y_cur;
          hold_left   <= y_left;
          hold_up     <= y_up;
        end else begin
          if (y_cur  !== hold_cur)  $fatal(1, "[TB] HOLD FAIL y_cur changed under stall");
          if (y_left !== hold_left) $fatal(1, "[TB] HOLD FAIL y_left changed under stall");
          if (y_up   !== hold_up)   $fatal(1, "[TB] HOLD FAIL y_up changed under stall");
        end
      end

      if (fire_out) begin
        hold_active <= 1'b0;
      end
    end
  end

  // ============================================================
  // Push expected + update golden on TRUE accept (v1_accept)
  // exp_cur uses y_in (accepted payload), exp_up uses gmem[x] (prev line),
  // exp_left uses gleft (prev in-line) with border handling.
  // ============================================================
  always_ff @(posedge clk) begin
    if (rst || !en) begin
      // do nothing; initial/tasks handle reset
    end else begin
      if (v1_accept) begin
        logic [YPIX_W-1:0] exp_cur, exp_left, exp_up;

        exp_cur  = y_in;
        exp_up   = gmem[x];
        exp_left = (x == '0) ? y_in : gleft;

        q_push(exp_cur, exp_left, exp_up);

        // update golden AFTER push (read-before-write)
        gmem[x] <= y_in;
        gleft   <= y_in;
      end
    end
  end

  // ============================================================
  // Compare only when downstream consumes output (fire_out)
  // ============================================================
  always_ff @(posedge clk) begin
    if (!rst && en) begin
      if (fire_out) begin
        logic [YPIX_W-1:0] ec, el, eu;

        q_pop(ec, el, eu);

        if (y_cur !== ec) begin
          $display("[TB] y_cur mismatch t=%0t got=%0d exp=%0d (x=%0d y=%0d)",
                   $time, y_cur, ec, x, y);
          $fatal(1, "FAIL");
        end
        if (y_left !== el) begin
          $display("[TB] y_left mismatch t=%0t got=%0d exp=%0d (x=%0d y=%0d)",
                   $time, y_left, el, x, y);
          $fatal(1, "FAIL");
        end
        if (y_up !== eu) begin
          $display("[TB] y_up mismatch t=%0t got=%0d exp=%0d (x=%0d y=%0d)",
                   $time, y_up, eu, x, y);
          $fatal(1, "FAIL");
        end

        // (optional) basic meta sanity
        if (x_mod !== x[1:0]) $fatal(1, "[TB] x_mod mismatch");
        if (y_mod !== y[1:0]) $fatal(1, "[TB] y_mod mismatch");
      end
    end
  end

  // ============================================================
  // main
  // ============================================================
  int yy, xx;
  localparam int unsigned LINES = 2;

  initial begin
    $dumpfile("./vvp/tb_pix_xy_linebuf_top.vcd");
    $dumpvars(0, tb_pix_xy_linebuf_top);

    // init
    en        = 1'b1;
    rst       = 1'b1;
    cyc       = 0;
    bp_enable = 1'b0;

    pix_valid = 1'b0;
    sof       = 1'b0;
    eol       = 1'b0;
    eof       = 1'b0;
    y_in      = '0;

    gold_reset();
    q_reset();

    // reset
    repeat (5) step();
    rst = 1'b0;

    // ----------------------------------------------------------
    // 0) SOF beat
    // ----------------------------------------------------------
    $display("[RUN] SOF beat");
    bp_enable = 1'b0;
    gold_reset();
    q_reset();
    send_beat(1'b1, 8'h00);

    // ----------------------------------------------------------
    // 1) run multiple lines with backpressure
    //    y pattern = yy*40 + xx
    // ----------------------------------------------------------
    $display("[RUN] %0d lines x %0d pixels with backpressure", LINES, ACTIVE_W);
    bp_enable = 1'b1;

    for (yy=0; yy<LINES; yy=yy+1) begin
      for (xx=0; xx<ACTIVE_W; xx=xx+1) begin
        send_beat(1'b0, byte'(yy*40 + xx));
      end
    end

    // drain remaining outputs
    begin : DRAIN
      int guard;
      guard = 0;
      while (q_rptr != q_wptr) begin
        step(); cyc++; guard++;
        if (guard > 5000) $fatal(1, "[TB] TIMEOUT draining q_rptr=%0d q_wptr=%0d out_valid=%0b out_ready=%0b",
                                 q_rptr, q_wptr, out_valid, out_ready);
      end
    end

    $display("[PASS] tb_pix_xy_linebuf_top");
    $finish;
  end

endmodule

`default_nettype wire
