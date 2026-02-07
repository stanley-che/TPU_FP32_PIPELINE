// tb_xy_counter.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_xy_counter.vvp \
  ./test/tb_xy_counter.sv

vvp ./vvp/tb_xy_counter.vvp
gtkwave ./vvp/tb_xy_counter.vcd
*/

`include "./src/AMOLED/video_in_timing_if/active_window_tracker.sv"
`include "./src/AMOLED/video_in_timing_if/xy_counter.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_xy_counter;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // ------------------------------------------------------------
  // raw sync inputs
  // ------------------------------------------------------------
  logic vsync, hsync, de;

  // ------------------------------------------------------------
  // edge pulses (registered 1-cycle)
  // ------------------------------------------------------------
  logic vs_rise, vs_fall;
  logic hs_rise, hs_fall;
  logic de_rise, de_fall;

  logic vs_d, hs_d, de_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_d    <= 1'b0;
      hs_d    <= 1'b0;
      de_d    <= 1'b0;

      vs_rise <= 1'b0;
      vs_fall <= 1'b0;
      hs_rise <= 1'b0;
      hs_fall <= 1'b0;
      de_rise <= 1'b0;
      de_fall <= 1'b0;
    end else begin
      // pulses computed from previous delayed value
      vs_rise <=  vsync & ~vs_d;
      vs_fall <= ~vsync &  vs_d;

      hs_rise <=  hsync & ~hs_d;
      hs_fall <= ~hsync &  hs_d;

      de_rise <=  de & ~de_d;
      de_fall <= ~de &  de_d;

      // update delayed
      vs_d <= vsync;
      hs_d <= hsync;
      de_d <= de;
    end
  end

  // 1-cycle delayed edge pulses (for comparing registered outputs)
  logic vs_rise_q, de_rise_q, de_fall_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      vs_rise_q <= 1'b0;
      de_rise_q <= 1'b0;
      de_fall_q <= 1'b0;
    end else begin
      vs_rise_q <= vs_rise;
      de_rise_q <= de_rise;
      de_fall_q <= de_fall;
    end
  end

  // ------------------------------------------------------------
  // active_window_tracker
  // ------------------------------------------------------------
  logic in_frame, in_line, pix_valid;

  active_window_tracker #(
    .FRAME_START_ON_VS_RISE(1'b1),
    .USE_HSYNC_FOR_EOL(1'b0)
  ) u_awt (
    .clk(clk),
    .rst(rst),
    .vs_rise(vs_rise),
    .vs_fall(vs_fall),
    .hs_rise(hs_rise),
    .hs_fall(hs_fall),
    .de(de),
    .de_rise(de_rise),
    .de_fall(de_fall),
    .in_frame(in_frame),
    .in_line(in_line),
    .pix_valid(pix_valid)
  );

  // ------------------------------------------------------------
  // xy_counter under test
  // ------------------------------------------------------------
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 6;

  logic [10:0] x;
  logic [9:0]  y;

  logic frame_start_p, line_start_p, line_end_p;
  logic x_at_last, y_at_last, x_overflow, y_overflow;

  xy_counter #(
    .X_W(11),
    .Y_W(10),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),
    .Y_INC_ON_DE_RISE(1'b0),
    .X_LIMIT_MODE(2),
    .Y_LIMIT_MODE(2),
    .ENABLE_BOUNDS(1'b1),
    .GATE_BY_IN_FRAME(1'b1),
    .ENABLE_ASSERT(1'b0)
  ) dut (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .de_rise(de_rise),
    .de_fall(de_fall),
    .vs_rise(vs_rise),
    .in_frame_i(in_frame),
    .x(x),
    .y(y),
    .frame_start_p(frame_start_p),
    .line_start_p(line_start_p),
    .line_end_p(line_end_p),
    .x_at_last(x_at_last),
    .y_at_last(y_at_last),
    .x_overflow(x_overflow),
    .y_overflow(y_overflow)
  );

  // ------------------------------------------------------------
  // TB helpers
  // ------------------------------------------------------------
  int unsigned err_cnt = 0;

  task automatic tb_err(input string msg);
    begin
      $display("[ERR] t=%0t %s", $time, msg);
      err_cnt++;
    end
  endtask

  task automatic drive_pix(input bit v, input bit h, input bit d);
    begin
      vsync <= v;
      hsync <= h;
      de    <= d;
      @(posedge clk);
      #1;
      check_one_cycle();
    end
  endtask

  task automatic idle(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i++) begin
        drive_pix(vsync, hsync, de);
      end
    end
  endtask

  // ------------------------------------------------------------
  // Scoreboard model
  // ------------------------------------------------------------
  int unsigned exp_x, exp_y;
  logic [9:0] y_prev;

  function automatic int unsigned sat_inc(input int unsigned val, input int unsigned last);
    if (val < last) sat_inc = val + 1;
    else            sat_inc = last;
  endfunction

  task automatic model_step;
  begin
    if (vs_rise) begin
      exp_x = 0;
      exp_y = 0;
      suppress_next_de_fall = 1'b1;  // <<< 關鍵
    end else if (in_frame) begin
      if (de_rise) begin
        exp_x = 0;
      end else if (pix_valid) begin
        exp_x = sat_inc(exp_x, (ACTIVE_W-1));
      end

      if (de_fall) begin
        if (suppress_next_de_fall) begin
          // 忽略被 vs_rise 腰斬的 line
          suppress_next_de_fall = 1'b0;
        end else begin
          exp_y = sat_inc(exp_y, (ACTIVE_H-1));
        end
      end
    end
  end
  endtask


  task automatic check_one_cycle;
    begin
      model_step();

      // x 永遠檢查（因為 x 在 !in_frame 時也會保持/可預測）
      if (x !== exp_x[10:0]) begin
        tb_err($sformatf("x mismatch: got=%0d exp=%0d (pv=%0b dr=%0b df=%0b vsr=%0b in_frame=%0b)",
                         x, exp_x, pix_valid, de_rise, de_fall, vs_rise, in_frame));
      end

      // y：只在 in_frame 時檢查值
      if (in_frame_q) begin
        // frame 內檢查 y 值
        if (y !== exp_y[9:0]) begin
          tb_err($sformatf("y mismatch: got=%0d exp=%0d ...", y, exp_y));
        end
      end else begin
        // frame 外只要求 y 不變
        if (y !== y_prev) begin
          tb_err($sformatf("y changed while !in_frame_q: prev=%0d now=%0d", y_prev, y));
        end
      end

      y_prev = y;

      // bounds
      if (x > (ACTIVE_W-1)) tb_err($sformatf("x out of bounds: x=%0d", x));
      if (y > (ACTIVE_H-1)) tb_err($sformatf("y out of bounds: y=%0d", y));

      // pulses are registered => compare with *_q
      if (frame_start_p !== vs_rise_q) tb_err("frame_start_p != vs_rise_q (1-cycle delayed)");
      if (line_start_p  !== de_rise_q) tb_err("line_start_p  != de_rise_q (1-cycle delayed)");
      if (line_end_p    !== de_fall_q) tb_err("line_end_p    != de_fall_q (1-cycle delayed)");

      // status flags
      if (x_at_last !== (x == (ACTIVE_W-1))) tb_err("x_at_last wrong");
      if (y_at_last !== (y == (ACTIVE_H-1))) tb_err("y_at_last wrong");
    end
  endtask

  // ------------------------------------------------------------
  // Timing pattern generators
  // ------------------------------------------------------------
  task automatic frame_start_pulse;
    begin
      drive_pix(1'b1, 1'b0, 1'b0);
      drive_pix(1'b0, 1'b0, 1'b0);
    end
  endtask

  task automatic line_hsync_pulse;
    begin
      drive_pix(vsync, 1'b1, 1'b0);
      drive_pix(vsync, 1'b0, 1'b0);
    end
  endtask

  task automatic drive_active_line(input int unsigned w);
    int unsigned i;
    begin
      line_hsync_pulse();
      for (i=0; i<w; i++) drive_pix(vsync, 1'b0, 1'b1);
      for (i=0; i<4; i++) drive_pix(vsync, 1'b0, 1'b0);
    end
  endtask

  task automatic drive_blank_line;
    int unsigned i;
    begin
      line_hsync_pulse();
      for (i=0; i<(ACTIVE_W+4); i++) drive_pix(vsync, 1'b0, 1'b0);
    end
  endtask

  // ------------------------------------------------------------
  // Test cases
  // ------------------------------------------------------------
  task automatic tc1_basic_frame;
    int unsigned j;
    begin
      $display("[TC1] basic frame counting");
      frame_start_pulse();
      for (j=0; j<ACTIVE_H; j++) drive_active_line(ACTIVE_W);
      drive_blank_line();
    end
  endtask

  task automatic tc2_de_glitch_outside_frame;
    begin
      $display("[TC2] de glitch outside frame");
      vsync = 1'b0; hsync = 1'b0; de = 1'b0;
      idle(5);
      drive_pix(1'b0, 1'b0, 1'b1);
      drive_pix(1'b0, 1'b0, 1'b0);
      idle(5);
    end
  endtask

  task automatic tc3_short_line;
    begin
      $display("[TC3] short active line (w=ACTIVE_W-3)");
      frame_start_pulse();
      drive_active_line(ACTIVE_W-3);
      drive_blank_line();
    end
  endtask

  task automatic tc4_long_line_saturate;
    begin
      $display("[TC4] long active line (w=ACTIVE_W+5) saturate x");
      frame_start_pulse();
      drive_active_line(ACTIVE_W+5);
      drive_blank_line();
    end
  endtask

  task automatic tc5_vsrise_mid_line;
    int unsigned i;
    begin
      $display("[TC5] vs_rise in middle of active line resets x/y");
      frame_start_pulse();
      line_hsync_pulse();

      for (i=0; i<5; i++) drive_pix(vsync, 1'b0, 1'b1);

      // abrupt new frame
      drive_pix(1'b1, 1'b0, 1'b1);
      drive_pix(1'b0, 1'b0, 1'b0);

      drive_active_line(ACTIVE_W);
    end
  endtask
  logic suppress_next_de_fall;
  logic in_frame_q;
  always_ff @(posedge clk) begin
    if (rst) in_frame_q <= 1'b0;
    else     in_frame_q <= in_frame;
  end

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_xy_counter.vcd");
    $dumpvars(0, tb_xy_counter);

    vsync = 1'b0;
    hsync = 1'b0;
    de    = 1'b0;

    exp_x  = 0;
    exp_y  = 0;
    y_prev = '0;
    suppress_next_de_fall = 1'b0;

    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
    tc1_basic_frame();
    tc2_de_glitch_outside_frame();
    tc3_short_line();
    tc4_long_line_saturate();
    tc5_vsrise_mid_line();

    if (err_cnt == 0) $display("[TB] PASS: no errors");
    else              $display("[TB] FAIL: err_cnt=%0d", err_cnt);

    $finish;
  end

endmodule

`default_nettype wire
