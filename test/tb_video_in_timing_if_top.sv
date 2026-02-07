// tb_video_in_timing_if_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_video_in_timing_if_top.vvp \
  ./test/tb_video_in_timing_if_top.sv

vvp ./vvp/tb_video_in_timing_if_top.vvp
gtkwave ./vvp/tb_video_in_timing_if_top.vcd
*/

`include "./src/AMOLED/video_in_timing_if/video_in_timing_if_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_video_in_timing_if_top;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // ------------------------------------------------------------
  // raw inputs
  // ------------------------------------------------------------
  logic        vsync_i, hsync_i, de_i;
  logic [23:0] rgb_i;

  // ------------------------------------------------------------
  // DUT outputs
  // ------------------------------------------------------------
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 6;

  localparam int unsigned X_W = 11;
  localparam int unsigned Y_W = 10;

  logic              vsync, hsync, de;
  logic              in_frame, in_line, pix_valid;
  logic [X_W-1:0]     x;
  logic [Y_W-1:0]     y;

  logic              frame_start_p, line_start_p, line_end_p;
  logic              x_at_last, y_at_last, x_overflow, y_overflow;

  logic              sof, sol, eol, eof;

  logic              pix_valid_o;
  logic [23:0]       pix_rgb_out;
  logic [X_W-1:0]    x_o;
  logic [Y_W-1:0]    y_o;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  video_in_timing_if_top #(
    // sync_edge_top
    .VS_INV(1'b0),
    .HS_INV(1'b0),
    .DE_INV(1'b0),
    .USE_DEGLITCH(1'b0),
    .STABLE_CYCLES(2),

    // active_window_tracker
    .FRAME_START_ON_VS_RISE(1'b1),
    .USE_HSYNC_FOR_EOL(1'b0),

    // xy_counter
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),
    .Y_INC_ON_DE_RISE(1'b0),
    .X_LIMIT_MODE(2), // sat
    .Y_LIMIT_MODE(2),
    .ENABLE_BOUNDS(1'b1),
    .GATE_BY_IN_FRAME(1'b1),
    .ENABLE_ASSERT(1'b0),

    // frame_event_gen
    .FEV_USE_VS_FALL(1'b0),
    .FEV_GATE_BY_FRAME(1'b1)
  ) dut (
    .clk(clk),
    .rst(rst),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i   (de_i),
    .rgb_i  (rgb_i),

    .vsync(vsync),
    .hsync(hsync),
    .de   (de),

    .in_frame(in_frame),
    .in_line (in_line),
    .pix_valid(pix_valid),

    .x(x),
    .y(y),

    .frame_start_p(frame_start_p),
    .line_start_p (line_start_p),
    .line_end_p   (line_end_p),

    .x_at_last(x_at_last),
    .y_at_last(y_at_last),
    .x_overflow(x_overflow),
    .y_overflow(y_overflow),

    .sof(sof),
    .sol(sol),
    .eol(eol),
    .eof(eof),

    .pix_valid_o(pix_valid_o),
    .pix_rgb_out(pix_rgb_out),
    .x_o(x_o),
    .y_o(y_o)
  );

  // ------------------------------------------------------------
  // TB infra
  // ------------------------------------------------------------
  int unsigned err_cnt = 0;

  task automatic tb_err(input string msg);
    begin
      $display("[ERR] t=%0t %s", $time, msg);
      err_cnt++;
    end
  endtask

  task automatic tb_ok(input string msg);
    begin
      $display("[OK ] t=%0t %s", $time, msg);
    end
  endtask

  // ------------------------------------------------------------
  // Drive 1 cycle of raw inputs, then check after DUT updates
  // ------------------------------------------------------------
  task automatic drive_raw(
    input bit         v,
    input bit         h,
    input bit         d,
    input logic [23:0] rgb
  );
    begin
      vsync_i <= v;
      hsync_i <= h;
      de_i    <= d;
      rgb_i   <= rgb;
      @(posedge clk);
      #1;
      check_one_cycle();
    end
  endtask

  task automatic idle_cycles(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i++) begin
        drive_raw(vsync_i, hsync_i, de_i, rgb_i);
      end
    end
  endtask

  // ------------------------------------------------------------
  // XY golden model (uses DUT pulses)
  // ------------------------------------------------------------
  int unsigned exp_x, exp_y;
  logic suppress_next_line_end;

  function automatic int unsigned sat_inc(input int unsigned val, input int unsigned last);
    if (val < last) sat_inc = val + 1;
    else            sat_inc = last;
  endfunction

  task automatic model_step;
    begin
      // In many "gate_by_in_frame" designs, leaving frame resets x/y to 0.
      if (!in_frame) begin
        exp_x = 0;
        exp_y = 0;
        suppress_next_line_end = 1'b0;
      end else begin
        if (frame_start_p) begin
          exp_x = 0;
          exp_y = 0;
          suppress_next_line_end = de; // frame starts while de=1
        end else begin
          if (line_start_p) begin
            exp_x = 0;
          end else if (pix_valid) begin
            exp_x = sat_inc(exp_x, (ACTIVE_W-1));
          end

          if (line_end_p) begin
            if (suppress_next_line_end) begin
              suppress_next_line_end = 1'b0;
            end else begin
              exp_y = sat_inc(exp_y, (ACTIVE_H-1));
            end
          end
        end
      end
    end
  endtask

  // ------------------------------------------------------------
  // Frame-seen state for eof expectation
  // (match intent of your tb_frame_event_gen: only count a "real frame"
  // when vs_rise happens while in_frame=1)
  // ------------------------------------------------------------
  logic frame_seen;

  always_ff @(posedge clk) begin
    if (rst) frame_seen <= 1'b0;
    else if (frame_start_p) frame_seen <= 1'b1; // 用真正的 frame start
  end


  // ------------------------------------------------------------
  // Checks
  // IMPORTANT: use DUT internal pulses (sync_edge_top outputs),
  // not vsync/de level, to match filter/deglitch behavior.
  // ------------------------------------------------------------
  task automatic check_one_cycle;
    logic vs_rise_p, vs_fall_p, de_rise_p, de_fall_p;
    logic sof_exp, sol_exp, eol_exp, eof_exp;
    logic frame_gate;
    begin
      // sample DUT internal pulses
      vs_rise_p = dut.vs_rise;
      vs_fall_p = dut.vs_fall;
      de_rise_p = dut.de_rise;
      de_fall_p = dut.de_fall;

      // model xy
      model_step();

      // ---- xy checks ----
      if (x !== exp_x[X_W-1:0]) begin
        tb_err($sformatf("x mismatch: got=%0d exp=%0d (pv=%0b ls=%0b le=%0b fs=%0b in_frame=%0b de=%0b)",
                         x, exp_x, pix_valid, line_start_p, line_end_p, frame_start_p, in_frame, de));
      end
      if (y !== exp_y[Y_W-1:0]) begin
        tb_err($sformatf("y mismatch: got=%0d exp=%0d (pv=%0b ls=%0b le=%0b fs=%0b in_frame=%0b de=%0b)",
                         y, exp_y, pix_valid, line_start_p, line_end_p, frame_start_p, in_frame, de));
      end

      if (x > (ACTIVE_W-1)) tb_err($sformatf("x out of bounds: x=%0d", x));
      if (y > (ACTIVE_H-1)) tb_err($sformatf("y out of bounds: y=%0d", y));

      if (x_at_last !== (x == (ACTIVE_W-1))) tb_err("x_at_last wrong");
      if (y_at_last !== (y == (ACTIVE_H-1))) tb_err("y_at_last wrong");

      // ---- frame_event_gen expected (match tb_frame_event_gen.sv) ----
      // sof = vs_rise (NOT gated)
      // sol/eol gated by in_frame when FEV_GATE_BY_FRAME=1
      // eof (USE_VS_FALL=0) = vs_rise when frame_seen already true
      sof_exp = frame_start_p;
      
      frame_gate = prev_in_frame;  // 這個才是 DUT 同拍 gate 會用到的 “舊 in_frame”

      sol_exp = line_start_p && frame_gate;
      eol_exp = line_end_p   && frame_gate;


      // eof：如果你的設計是「下一個 frame_start 當作上一個 frame 的 eof」
      eof_exp = frame_start_p && frame_seen;


      if (sof !== sof_exp) tb_err($sformatf("sof mismatch: got=%0b exp=%0b (vs_rise_p=%0b in_frame=%0b)",
                                            sof, sof_exp, vs_rise_p, in_frame));
      if (sol !== sol_exp) tb_err($sformatf("sol mismatch: got=%0b exp=%0b (de_rise_p=%0b in_frame=%0b)",
                                            sol, sol_exp, de_rise_p, in_frame));
      if (eol !== eol_exp) tb_err($sformatf("eol mismatch: got=%0b exp=%0b (de_fall_p=%0b in_frame=%0b)",
                                            eol, eol_exp, de_fall_p, in_frame));
      if (eof !== eof_exp) tb_err($sformatf("eof mismatch: got=%0b exp=%0b (vs_rise_p=%0b frame_seen=%0b)",
                                            eof, eof_exp, vs_rise_p, frame_seen));

      // ---- pixel forwarding ----
      if (pix_valid_o) begin
        if (x_o !== x) tb_err($sformatf("x_o mismatch on pix_valid_o: x_o=%0d x=%0d", x_o, x));
        if (y_o !== y) tb_err($sformatf("y_o mismatch on pix_valid_o: y_o=%0d y=%0d", y_o, y));
        if (pix_rgb_out !== rgb_i) tb_err($sformatf("rgb_out mismatch on pix_valid_o: out=%h in=%h", pix_rgb_out, rgb_i));
      end
    end
  endtask
  logic prev_in_frame;

  always_ff @(posedge clk) begin
      if (rst) prev_in_frame <= 1'b0;
      else     prev_in_frame <= in_frame; // <= RHS 讀舊 in_frame，所以這個會變成“上一拍”的
  end

  // ------------------------------------------------------------
  // Pattern generators (raw)
  // ------------------------------------------------------------
  task automatic vsync_pulse;
    begin
      drive_raw(1'b1, 1'b0, 1'b0, 24'h0);
      drive_raw(1'b0, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic hsync_pulse;
    begin
      drive_raw(vsync_i, 1'b1, 1'b0, 24'h0);
      drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic drive_active_line(input int unsigned w);
    int unsigned i;
    logic [7:0]  p;
    logic [23:0] rgbp;
    begin
      hsync_pulse();
      for (i=0; i<w; i++) begin
        p    = i[7:0];
        rgbp = {p,p,p};
        drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
      end
      for (i=0; i<4; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic drive_blank_line;
    int unsigned i;
    begin
      hsync_pulse();
      for (i=0; i<(ACTIVE_W+4); i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end
  endtask

  // ------------------------------------------------------------
  // Test cases
  // ------------------------------------------------------------
  task automatic tc1_basic_one_frame;
    int unsigned j;
    begin
      $display("\n[TC1] basic one frame");
      vsync_pulse();
      for (j=0; j<ACTIVE_H; j++) drive_active_line(ACTIVE_W);
      drive_blank_line();
      vsync_pulse();
      tb_ok("TC1 done");
    end
  endtask

  task automatic tc2_long_line_saturate;
    begin
      $display("\n[TC2] long active line saturate x");
      vsync_pulse();
      drive_active_line(ACTIVE_W + 5);
      vsync_pulse();
      tb_ok("TC2 done");
    end
  endtask

  task automatic tc3_vsrise_mid_active;
    int unsigned i;
    logic [23:0] rgbp;
    begin
      $display("\n[TC3] vsync pulse in middle of active (de=1) => new frame cut");
      vsync_pulse();

      hsync_pulse();
      for (i=0; i<5; i++) begin
        rgbp = {i[7:0], i[7:0], i[7:0]};
        drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
      end

      // abrupt new frame marker while de=1
      drive_raw(1'b1, 1'b0, 1'b1, 24'hAA_00_00);
      drive_raw(1'b0, 1'b0, 1'b0, 24'h0);

      drive_active_line(ACTIVE_W);
      vsync_pulse();
      tb_ok("TC3 done");
    end
  endtask

  task automatic tc4_de_short_pulses;
  int unsigned i;
  begin
    $display("\n[TC4] de short pulses (1/2 cycles) in blank");
    vsync_pulse();

    // blank line base
    hsync_pulse();
    for (i=0; i<4; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // de=1 for 1 cycle
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h11_11_11);
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // de=1 for 2 cycles
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h22_22_22);
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h33_33_33);
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // finish line
    for (i=0; i<8; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    vsync_pulse();
    tb_ok("TC4 done");
  end
  endtask
  
  task automatic tc4b_count_pixvalid;
  int unsigned i;
  int unsigned pv_cnt_before, pv_cnt_after;
  begin
    $display("\n[TC4b] count pix_valid_o during short de pulses");
    pv_cnt_before = 0;
    pv_cnt_after  = 0;

    vsync_pulse();

    // 前半段：計數視窗
    for (i=0; i<10; i++) begin
      drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
      if (pix_valid_o) pv_cnt_before++;
    end

    // 1-cycle de pulse
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h44_44_44);
    if (pix_valid_o) pv_cnt_after++;
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    if (pix_valid_o) pv_cnt_after++;

    // 2-cycle de pulse
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h55_55_55);
    if (pix_valid_o) pv_cnt_after++;
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h66_66_66);
    if (pix_valid_o) pv_cnt_after++;
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    if (pix_valid_o) pv_cnt_after++;

    vsync_pulse();

    $display("[TC4b] pv_cnt_before=%0d pv_cnt_after=%0d", pv_cnt_before, pv_cnt_after);
    tb_ok("TC4b done");
  end
  endtask

  task automatic tc5_de_split_in_one_line;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC5] de split in one line (1 line -> two de segments)");
    vsync_pulse();

    hsync_pulse();

    // segment A: de=1 for 5 pix
    for (i=0; i<5; i++) begin
      rgbp = {i[7:0], i[7:0], i[7:0]};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // gap: de=0 for 3 pix
    for (i=0; i<3; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // segment B: de=1 for 5 pix
    for (i=0; i<5; i++) begin
      rgbp = {8'hA0+i[7:0], 8'h00, 8'h00};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // tail blank
    for (i=0; i<4; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    vsync_pulse();
    tb_ok("TC5 done");
  end
  endtask
  
  task automatic tc6_no_hsync_de_only_frame;
  int unsigned r, c;
  logic [23:0] rgbp;
  begin
    $display("\n[TC6] no hsync, de-only frame (line end by de_fall)");
    vsync_pulse();

    for (r=0; r<ACTIVE_H; r++) begin
      // de=1 active pixels
      for (c=0; c<ACTIVE_W; c++) begin
        rgbp = {r[7:0], c[7:0], 8'h00};
        drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
      end
      // de=0 blank pixels (line gap)
      for (c=0; c<6; c++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end

    // extra blank
    for (c=0; c<20; c++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    vsync_pulse();
    tb_ok("TC6 done");
  end
  endtask

task automatic tc7_hsync_glitch_mid_active;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC7] hsync glitch (1 cycle) in middle of active de=1");
    vsync_pulse();

    hsync_pulse();

    // 先跑幾個 active
    for (i=0; i<5; i++) begin
      rgbp = {8'h00, i[7:0], 8'h00};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // glitch: hsync=1 for 1 cycle while de=1
    drive_raw(vsync_i, 1'b1, 1'b1, 24'hFF_00_FF);
    drive_raw(vsync_i, 1'b0, 1'b1, 24'h00_00_00);

    // 繼續 active
    for (i=0; i<8; i++) begin
      rgbp = {8'h00, (8'h10+i[7:0]), 8'h00};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // tail blank
    for (i=0; i<6; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    vsync_pulse();
    tb_ok("TC7 done");
  end
endtask

task automatic tc8_vsync_glitch_1cycle;
  int unsigned i;
  begin
    $display("\n[TC8] vsync glitch (1 cycle) in blank");
    vsync_pulse();

    // some blank
    for (i=0; i<10; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // glitch: 1-cycle vsync
    drive_raw(1'b1, 1'b0, 1'b0, 24'h0);
    drive_raw(1'b0, 1'b0, 1'b0, 24'h0);

    // some active lines after glitch
    for (i=0; i<2; i++) drive_active_line(ACTIVE_W);

    vsync_pulse();
    tb_ok("TC8 done");
  end
endtask
task automatic tc9_long_blank_gap;
  int unsigned i;
  begin
    $display("\n[TC9] long blank gap (de=0 for long time)");
    vsync_pulse();

    drive_active_line(ACTIVE_W);

    // long blank gap
    for (i=0; i<200; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    drive_active_line(ACTIVE_W);
    vsync_pulse();

    tb_ok("TC9 done");
  end
endtask
task automatic tc10_more_than_active_h_lines;
  int unsigned j;
  begin
    $display("\n[TC10] send more than ACTIVE_H lines (expect y saturate)");
    vsync_pulse();

    for (j=0; j<(ACTIVE_H+3); j++) begin
      drive_active_line(ACTIVE_W);
    end

    vsync_pulse();
    tb_ok("TC10 done");
  end
endtask
task automatic tc11_back_to_back_vsync;
  int unsigned i;
  begin
    $display("\n[TC11] back-to-back vsync pulses (very short frame)");
    vsync_pulse();
    for (i=0; i<3; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    vsync_pulse(); // next frame almost immediately
    for (i=0; i<2; i++) drive_active_line(ACTIVE_W);
    vsync_pulse();
    tb_ok("TC11 done");
  end
endtask
task automatic tc12_early_de_fall_end_line;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC12] early de_fall in active line (short line) then long blank");
    vsync_pulse();

    hsync_pulse();

    // only 4 active pixels (shorter than ACTIVE_W)
    for (i=0; i<4; i++) begin
      rgbp = {8'h00, 8'h00, i[7:0]};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // drop de early
    for (i=0; i<20; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // next normal line
    drive_active_line(ACTIVE_W);

    vsync_pulse();
    tb_ok("TC12 done");
  end
endtask

task automatic tc13_reset_mid_active;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC13] reset asserted mid-active (de=1)");

    vsync_pulse();
    hsync_pulse();

    // 先送幾個 active pixel
    for (i=0; i<5; i++) begin
      rgbp = {8'h13, i[7:0], 8'h00};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // 在 de=1 中間拉 reset 2 cycles
    rst <= 1'b1;
    @(posedge clk); #1; check_one_cycle();
    @(posedge clk); #1; check_one_cycle();
    rst <= 1'b0;
    @(posedge clk); #1; check_one_cycle();

    // reset 後再送一些像素，確認不亂跳
    for (i=0; i<6; i++) begin
      rgbp = {8'h23, i[7:0], 8'h01};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // 結束
    for (i=0; i<6; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    vsync_pulse();

    tb_ok("TC13 done");
  end
endtask

task automatic tc15_vsync_long_high;
  int unsigned i;
  begin
    $display("\n[TC15] vsync long-high pulse (multi-cycle)");

    // 拉高很多拍
    for (i=0; i<20; i++) drive_raw(1'b1, 1'b0, 1'b0, 24'h0);
    // 回到低
    for (i=0; i<5;  i++) drive_raw(1'b0, 1'b0, 1'b0, 24'h0);

    // 送兩行正常 active，確保 frame 邏輯沒亂
    drive_active_line(ACTIVE_W);
    drive_active_line(ACTIVE_W);

    vsync_pulse();
    tb_ok("TC15 done");
  end
endtask
task automatic tc18_de_hsync_overlap;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC18] de/hsync overlap on same cycle");

    vsync_pulse();

    // case A: 同拍 hsync=1 且 de=1（SOL 與 HS 重疊）
    drive_raw(vsync_i, 1'b1, 1'b1, 24'hAA_00_00);
    drive_raw(vsync_i, 1'b0, 1'b1, 24'hAA_00_01);

    for (i=0; i<6; i++) begin
      rgbp = {8'hAA, i[7:0], 8'h10};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // case B: 同拍 hsync=1 且 de 從 1 掉到 0（EOL 與 HS 重疊）
    drive_raw(vsync_i, 1'b1, 1'b0, 24'h00_00_00); // HS pulse while de=0
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h00_00_00);

    // 再來一行：同拍 hsync=1 且 de=1，然後在某拍同時 de_fall + hsync=1
    drive_raw(vsync_i, 1'b1, 1'b1, 24'hBB_00_00);
    for (i=0; i<4; i++) begin
      rgbp = {8'hBB, i[7:0], 8'h20};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end
    // 同拍：hsync=1，de=0（假設這拍是你想測的同拍結束）
    drive_raw(vsync_i, 1'b1, 1'b0, 24'h00_00_00);
    drive_raw(vsync_i, 1'b0, 1'b0, 24'h00_00_00);

    vsync_pulse();
    tb_ok("TC18 done");
  end
endtask
task automatic tc19_vsrise_derise_same_cycle;
  int unsigned i;
  logic [23:0] rgbp;
  begin
    $display("\n[TC19] vs_rise and de_rise same cycle");

    // 同拍拉起 vsync 與 de (以及給個像素)
    drive_raw(1'b1, 1'b0, 1'b1, 24'hCC_00_01);
    // 下一拍 drop vsync 但保持 de=1
    drive_raw(1'b0, 1'b0, 1'b1, 24'hCC_00_02);

    // 繼續送一些 active
    for (i=0; i<10; i++) begin
      rgbp = {8'hCC, i[7:0], 8'h33};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end

    // 拉低 de
    for (i=0; i<6; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    vsync_pulse();
    tb_ok("TC19 done");
  end
endtask
task automatic tc20_force_overflow_xy;
  int unsigned i, j;
  logic [23:0] rgbp;
  begin
    $display("\n[TC20] force x/y overflow scenarios");

    vsync_pulse();

    // --- X overflow: 一行送超長 ---
    hsync_pulse();
    for (i=0; i<(ACTIVE_W + 50); i++) begin
      rgbp = {8'hD0, i[7:0], 8'h00};
      drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
    end
    // drop de
    for (i=0; i<8; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);

    // --- Y overflow: 送超過 ACTIVE_H 很多行 ---
    for (j=0; j<(ACTIVE_H + 6); j++) begin
      hsync_pulse();
      for (i=0; i<ACTIVE_W; i++) begin
        rgbp = {8'hD1, j[7:0], i[7:0]};
        drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
      end
      for (i=0; i<4; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end

    vsync_pulse();
    tb_ok("TC20 done");
  end
endtask
task automatic tc17_blank_frame_no_de;
  int unsigned i;
  begin
    $display("\n[TC17] blank frame: vsync pulses but no de asserted");

    vsync_pulse();
    // 整段都 blank
    for (i=0; i<200; i++) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    vsync_pulse();

    // 再來一個正常 frame 起來，看看 eof/sof 邏輯
    drive_active_line(ACTIVE_W);
    vsync_pulse();

    tb_ok("TC17 done");
  end
endtask

  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_video_in_timing_if_top.vcd");
    $dumpvars(0, tb_video_in_timing_if_top);

    vsync_i = 1'b0;
    hsync_i = 1'b0;
    de_i    = 1'b0;
    rgb_i   = 24'h0;

    exp_x = 0;
    exp_y = 0;
    suppress_next_line_end = 1'b0;

    rst = 1'b1;
    repeat (8) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);
    #1;

    // let filters settle
    idle_cycles(5);

    tc1_basic_one_frame();
    tc2_long_line_saturate();
    tc3_vsrise_mid_active();
    tc4_de_short_pulses();
    tc5_de_split_in_one_line();
    tc6_no_hsync_de_only_frame();
    tc7_hsync_glitch_mid_active();
    tc8_vsync_glitch_1cycle();
    tc9_long_blank_gap();
    tc10_more_than_active_h_lines();
    tc11_back_to_back_vsync();
    tc12_early_de_fall_end_line();
    tc13_reset_mid_active();
    tc15_vsync_long_high();
    tc17_blank_frame_no_de();
    tc18_de_hsync_overlap();
    tc19_vsrise_derise_same_cycle();
    tc20_force_overflow_xy();
    if (err_cnt == 0) $display("\n[TB] PASS: no errors");
    else              $display("\n[TB] FAIL: err_cnt=%0d", err_cnt);

    $finish;
  end

endmodule

`default_nettype wire
