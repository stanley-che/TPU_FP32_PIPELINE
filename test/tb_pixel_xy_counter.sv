// tb_pixel_xy_counter.sv — FINAL PASS TB (stall-safe, no deadlock, one score per cycle)
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_pixel_xy_counter.vvp ./test/tb_pixel_xy_counter.sv
// Run:
//   vvp ./vvp/tb_pixel_xy_counter.vvp
//   gtkwave ./vvp/tb_pixel_xy_counter.vcd

`include "./src/AMOLED/tile feature_extracter/pixel_xy_counter.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_pixel_xy_counter;

  // ============================================================
  // clock/reset
  // ============================================================
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst, en;

  // ============================================================
  // params (縮小加速)
  // ============================================================
  localparam int unsigned X_W      = 6;
  localparam int unsigned Y_W      = 6;
  localparam int unsigned ACTIVE_W = 16;  // 16x6
  localparam int unsigned ACTIVE_H = 6;

  localparam int unsigned TILE_SHIFT = 2; // 4x4
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT); // 4
  localparam int unsigned TILES_Y_FIX= 2; // ceil(6/4)=2 for tile_i coverage
  localparam bit TB_USE_SOF = 1'b1;
  localparam bit TB_USE_EOF = 1'b0;

  // ============================================================
  // DUT I/O
  // ============================================================
  logic pix_valid;
  logic pix_ready;
  logic sof, eol, eof;

  logic                 v1_valid;
  logic                 v1_ready;

  logic [X_W-1:0]        x;
  logic [Y_W-1:0]        y;
  logic [X_W-1:0]        tile_j;
  logic [Y_W-1:0]        tile_i;
  logic [1:0]            x_mod;
  logic [1:0]            y_mod;

  logic                 tile_first;
  logic                 tile_last;
  logic                 tile_x_last;
  logic                 tile_y_last;

  logic                 x_last;
  logic                 y_last;
  logic                 line_start;
  logic                 line_last;
  logic                 frame_start;
  logic                 frame_last;

  logic [$clog2(TILES_X*TILES_Y_FIX)-1:0] tile_idx;

  logic                 in_roi;

  logic                 err_sof_midframe;
  logic                 err_eol_mismatch;
  logic [31:0]          frame_cnt;
  logic [31:0]          line_cnt;

  // TB-side fires
  wire fire_in_tb  = pix_valid && pix_ready;
  wire fire_out_tb = v1_valid && v1_ready;

  // ============================================================
  // DUT
  // ============================================================
  pixel_xy_counter #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y_FIX),

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

    .v1_valid(v1_valid),
    .v1_ready(v1_ready),
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

  // ============================================================
  // clock helper
  // ============================================================
  task automatic step;
    begin
       @(posedge clk);
       #1;
    end
  endtask


  // ============================================================
  // golden model
  // ============================================================
  int gx, gy;

  task automatic golden_reset;
    begin gx = 0; gy = 0; end
  endtask

  task automatic golden_advance;
    begin
      if (gx != (ACTIVE_W-1)) begin
        gx = gx + 1;
      end else begin
        gx = 0;
        if (gy != (ACTIVE_H-1)) gy = gy + 1;
        else                    gy = 0;
      end
    end
  endtask

  function automatic bit exp_x_last(input int xx);
    begin exp_x_last = (xx == (ACTIVE_W-1)); end
  endfunction
  function automatic bit exp_y_last(input int yy);
    begin exp_y_last = (yy == (ACTIVE_H-1)); end
  endfunction

  // ============================================================
  // checker
  // ============================================================
  task automatic check_outputs;
    int exp_tile_j, exp_tile_i;
    int exp_xm, exp_ym;
    bit exp_tile_first, exp_tile_last, exp_tile_x_last, exp_tile_y_last;
    bit exp_line_start, exp_line_last, exp_frame_start, exp_frame_last;
    begin
      if (v1_valid) begin
        exp_tile_j = (gx >> TILE_SHIFT);
        exp_tile_i = (gy >> TILE_SHIFT);
        exp_xm     = (gx & 3);
        exp_ym     = (gy & 3);

        exp_tile_first  = ((exp_xm == 0) && (exp_ym == 0));
        exp_tile_last   = ((exp_xm == 3) && (exp_ym == 3));
        exp_tile_x_last = (exp_xm == 3);
        exp_tile_y_last = (exp_ym == 3);

        exp_line_start  = (gx == 0);
        exp_line_last   = exp_x_last(gx);
        exp_frame_start = (gx == 0) && (gy == 0);
        exp_frame_last  = exp_x_last(gx) && exp_y_last(gy);

        if (x !== gx[X_W-1:0] || y !== gy[Y_W-1:0]) begin
          $display("[TB] ERROR x/y mismatch t=%0t got(x,y)=(%0d,%0d) exp=(%0d,%0d)",
                   $time, x, y, gx, gy);
          $fatal;
        end

        if (tile_j !== exp_tile_j[X_W-1:0] || tile_i !== exp_tile_i[Y_W-1:0]) begin
          $display("[TB] ERROR tile_i/j mismatch t=%0t got(ti,tj)=(%0d,%0d) exp=(%0d,%0d)",
                   $time, tile_i, tile_j, exp_tile_i, exp_tile_j);
          $fatal;
        end

        if (x_mod !== exp_xm[1:0] || y_mod !== exp_ym[1:0]) begin
          $display("[TB] ERROR x_mod/y_mod mismatch t=%0t got=(%0d,%0d) exp=(%0d,%0d)",
                   $time, x_mod, y_mod, exp_xm, exp_ym);
          $fatal;
        end

        if (tile_first !== exp_tile_first) begin
          $display("[TB] ERROR tile_first mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, tile_first, exp_tile_first, gx, gy);
          $fatal;
        end
        if (tile_last !== exp_tile_last) begin
          $display("[TB] ERROR tile_last mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, tile_last, exp_tile_last, gx, gy);
          $fatal;
        end
        if (tile_x_last !== exp_tile_x_last) begin
          $display("[TB] ERROR tile_x_last mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, tile_x_last, exp_tile_x_last, gx, gy);
          $fatal;
        end
        if (tile_y_last !== exp_tile_y_last) begin
          $display("[TB] ERROR tile_y_last mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, tile_y_last, exp_tile_y_last, gx, gy);
          $fatal;
        end

        if (line_start !== exp_line_start) begin
          $display("[TB] ERROR line_start mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, line_start, exp_line_start, gx, gy);
          $fatal;
        end
        if (line_last !== exp_line_last) begin
          $display("[TB] ERROR line_last mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, line_last, exp_line_last, gx, gy);
          $fatal;
        end
        if (frame_start !== exp_frame_start) begin
          $display("[TB] ERROR frame_start mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, frame_start, exp_frame_start, gx, gy);
          $fatal;
        end
        if (frame_last !== exp_frame_last) begin
          $display("[TB] ERROR frame_last mismatch t=%0t got=%0b exp=%0b at (x,y)=(%0d,%0d)",
                   $time, frame_last, exp_frame_last, gx, gy);
          $fatal;
        end

        if (x_last !== exp_x_last(gx)) begin
          $display("[TB] ERROR x_last mismatch t=%0t got=%0b exp=%0b at x=%0d",
                   $time, x_last, exp_x_last(gx), gx);
          $fatal;
        end
        if (y_last !== exp_y_last(gy)) begin
          $display("[TB] ERROR y_last mismatch t=%0t got=%0b exp=%0b at y=%0d",
                   $time, y_last, exp_y_last(gy), gy);
          $fatal;
        end

      end else begin
        if (tile_last !== 1'b0 || tile_first !== 1'b0 ||
            line_last !== 1'b0 || frame_last !== 1'b0) begin
          $display("[TB] ERROR pulses asserted while v1_valid=0 t=%0t", $time);
          $fatal;
        end
      end
    end
  endtask

  // ============================================================
  // READY DRIVER (the fix): v1_ready updates EVERY cycle
  // ============================================================
  int cyc;
  bit bp_enable;

  task automatic drive_ready_each_cycle;
  begin
    if (!bp_enable) begin
      v1_ready = 1'b1;
    end else begin
      if ((cyc % 5) == 4) v1_ready = 1'b0;
      else                v1_ready = 1'b1;
    end
  end
  endtask


  // single-cycle tick: update ready -> step -> score
task automatic tick;
  bit will_fire_out;
begin
  drive_ready_each_cycle();
  will_fire_out = (v1_valid && v1_ready);
  step();
  if (will_fire_out) golden_advance();
  cyc++;
end
endtask





  // ============================================================
  // debug helpers
  // ============================================================
  task automatic dump_state(input string tag);
    $display("[%s] t=%0t pv=%0b pr=%0b | v1v=%0b v1r=%0b | sof/eol/eof=%0b%0b%0b | x=%0d y=%0d | gx=%0d gy=%0d",
             tag, $time, pix_valid, pix_ready, v1_valid, v1_ready,
             sof, eol, eof, x, y, gx, gy);
  endtask

  // ============================================================
  // source driver: hold valid until accepted (NO deadlock)
  // ============================================================
  task automatic send_pixel_hold_until_accepted(
  input logic s,
  input logic el,
  input logic ef
);
  int guard;
  bit accepted;
  begin
    // drive inputs
    pix_valid = 1'b1;
    sof       = s;
    eol       = el;
    eof       = ef;

    guard    = 0;
    accepted = 0;

    // loop until accepted (NO break)
    while (!accepted) begin
      // 走一拍：讓 pix_ready / fire_in_tb 在這拍 settle
      tick();

      // 這拍是否真的接受了 input（tick() 後看，不要預測）
      accepted = fire_in_tb;

      // ★同拍如果是 SOF/EOF 且被接受：golden 要先 reset 才能對齊 DUT 的 (0,0)
      if (accepted && ((TB_USE_SOF && s) || (TB_USE_EOF && ef))) begin
        golden_reset();
      end

      // 現在再檢查輸出（gx/gy 已對齊）
      check_outputs();

      guard++;
      if (guard == 50)  dump_state("WAIT50");
      if (guard == 200) dump_state("WAIT200");
      if (guard > 2000) begin
        dump_state("TIMEOUT");
        $fatal(1, "[TB] TIMEOUT waiting fire_in");
      end
    end

    // deassert (blocking)
    pix_valid = 1'b0;
    sof       = 1'b0;
    eol       = 1'b0;
    eof       = 1'b0;

    // separation cycle (可留)
    tick();
    check_outputs();
  end
endtask



task automatic idle_cycles(input int unsigned n);
  int k;
  begin
    pix_valid = 1'b0;
    sof = 1'b0; eol = 1'b0; eof = 1'b0;
    for (k=0; k<n; k=k+1) begin
      tick();
      check_outputs();
    end
  end
endtask


  // ============================================================
  // main
  // ============================================================
  int i;

  initial begin
    $dumpfile("./vvp/tb_pixel_xy_counter.vcd");
    $dumpvars(0, tb_pixel_xy_counter);

    // init
    en        = 1'b1;
    pix_valid = 1'b0;
    sof       = 1'b0;
    eol       = 1'b0;
    eof       = 1'b0;

    cyc = 0;
    bp_enable = 1'b0; // start with no backpressure
    v1_ready  = 1'b1;

    // reset
    rst = 1'b1;
    repeat (5) tick();
    rst = 1'b0;

    golden_reset();

    // 0) idle: no valid expected
    idle_cycles(3);

    // 1) SOF accepted-beat reset
    $display("[RUN] SOF accepted-beat reset");
    bp_enable = 1'b0;
    golden_reset();
    send_pixel_hold_until_accepted(1'b1, 1'b0, 1'b0);

    // 2) one frame with backpressure
    $display("[RUN] one frame with backpressure (v1_ready toggling)");
    bp_enable = 1'b1;

    for (i=0; i<(ACTIVE_W*ACTIVE_H - 1); i=i+1) begin
      send_pixel_hold_until_accepted(1'b0, 1'b0, 1'b0);
    end

    bp_enable = 1'b0;
    idle_cycles(5);

    // 3) bubbles test + backpressure
    $display("[RUN] bubbles test (pix_valid=0) + backpressure");
    bp_enable = 1'b1;
    for (i=0; i<40; i=i+1) begin
      if ((i % 7) == 0) begin
        idle_cycles(1);
      end else begin
        send_pixel_hold_until_accepted(1'b0, 1'b0, 1'b0);
      end
    end
    bp_enable = 1'b0;
    idle_cycles(5);

    // 4) sof mid-stream -> expect err_sof_midframe sticky
    $display("[RUN] sof mid-stream (expect err_sof_midframe=1)");
    bp_enable = 1'b1;
    send_pixel_hold_until_accepted(1'b0, 1'b0, 1'b0);
    send_pixel_hold_until_accepted(1'b0, 1'b0, 1'b0);
    send_pixel_hold_until_accepted(1'b1, 1'b0, 1'b0);

    if (err_sof_midframe !== 1'b1) begin
      $display("[TB] ERROR err_sof_midframe not asserted t=%0t", $time);
      $fatal;
    end

    $display("[PASS] ALL");
    $finish;
  end

endmodule

`default_nettype wire
