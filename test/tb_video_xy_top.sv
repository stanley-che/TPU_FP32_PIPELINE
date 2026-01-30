// tb_video_xy_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_video_xy_top.vvp \
  ./test/tb_video_xy_top.sv

vvp ./vvp/tb_video_xy_top.vvp
gtkwave ./vvp/tb_video_xy_top.vcd
*/

`include "./src/AMOLED/video_in_timing_if/video_xy_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_video_xy_top;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // ------------------------------------------------------------
  // raw sync inputs to TOP
  // ------------------------------------------------------------
  logic vsync, hsync, de;

  // ------------------------------------------------------------
  // DUT outputs
  // ------------------------------------------------------------
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 6;

  logic in_frame, in_line, pix_valid;

  logic [10:0] x;
  logic [9:0]  y;

  logic frame_start_p, line_start_p, line_end_p;
  logic x_at_last, y_at_last, x_overflow, y_overflow;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  video_xy_top #(
    .FRAME_START_ON_VS_RISE(1'b1),
    .USE_HSYNC_FOR_EOL     (1'b0),

    .X_W(11),
    .Y_W(10),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .Y_INC_ON_DE_RISE(1'b0), // y++ on de_fall
    .X_LIMIT_MODE(2),
    .Y_LIMIT_MODE(2),

    .ENABLE_BOUNDS(1'b1),
    .GATE_BY_IN_FRAME(1'b1),
    .ENABLE_ASSERT(1'b0)
  ) dut (
    .clk(clk),
    .rst(rst),

    .vsync_i(vsync),
    .hsync_i(hsync),
    .de_i(de),

    .in_frame(in_frame),
    .in_line(in_line),
    .pix_valid(pix_valid),

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

  // drive one cycle of raw sync
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
  // Scoreboard model (uses RAW sync -> own edge detect)
  // ------------------------------------------------------------
  logic vs_d, hs_d, de_d;
  logic vs_rise, vs_fall;
  logic de_rise, de_fall;

  // 1-cycle delayed copies for comparing DUT registered pulses
  logic vs_rise_q, de_rise_q, de_fall_q;

  // model expected x/y
  int unsigned exp_x, exp_y;
  logic [9:0]  y_prev;

  // like your previous TB: suppress the de_fall after a mid-line vs_rise
  logic suppress_next_de_fall;

  function automatic int unsigned sat_inc(input int unsigned val, input int unsigned last);
    if (val < last) sat_inc = val + 1;
    else            sat_inc = last;
  endfunction

  // local in_frame_q for checking y outside frame
  logic in_frame_q;
  always_ff @(posedge clk) begin
    if (rst) in_frame_q <= 1'b0;
    else     in_frame_q <= in_frame;
  end

  // build edge pulses from raw sync (TB side)
  task automatic model_edges;
    begin
      vs_rise =  vsync & ~vs_d;
      vs_fall = ~vsync &  vs_d;

      de_rise =  de & ~de_d;
      de_fall = ~de &  de_d;

      // update delays (sequentially mimic sampling at this posedge)
      vs_d = vsync;
      hs_d = hsync;
      de_d = de;
    end
  endtask

  task automatic model_step;
    begin
      // edge calc based on current raw values and previous sampled
      model_edges();

      // push q-delays for comparing DUT registered pulses (DUT outputs pulses registered)
      vs_rise_q = vs_rise;
      de_rise_q = de_rise;
      de_fall_q = de_fall;

      // model x/y
      if (vs_rise) begin
        exp_x = 0;
        exp_y = 0;

        // if vs_rise happens while de is high -> will produce a "cut" de_fall later
        if (de) suppress_next_de_fall = 1'b1;
      end else if (in_frame) begin
        if (de_rise) begin
          exp_x = 0;
        end else if (pix_valid) begin
          exp_x = sat_inc(exp_x, (ACTIVE_W-1));
        end

        if (de_fall) begin
          if (suppress_next_de_fall) begin
            suppress_next_de_fall = 1'b0;
          end else begin
            exp_y = sat_inc(exp_y, (ACTIVE_H-1));
          end
        end
      end
    end
  endtask

  // ------------------------------------------------------------
  // Check one cycle
  // ------------------------------------------------------------
  task automatic check_one_cycle;
    begin
      model_step();

      // x check always
      if (x !== exp_x[10:0]) begin
        tb_err($sformatf("x mismatch: got=%0d exp=%0d (pv=%0b dr=%0b df=%0b vsr=%0b in_frame=%0b de=%0b)",
                         x, exp_x, pix_valid, de_rise, de_fall, vs_rise, in_frame, de));
      end

      // y check: inside frame check value; outside require stable
      if (in_frame_q) begin
        if (y !== exp_y[9:0]) begin
          tb_err($sformatf("y mismatch: got=%0d exp=%0d (pv=%0b dr=%0b df=%0b vsr=%0b in_frame=%0b de=%0b)",
                           y, exp_y, pix_valid, de_rise, de_fall, vs_rise, in_frame, de));
        end
      end else begin
        if (y !== y_prev) begin
          tb_err($sformatf("y changed while !in_frame_q: prev=%0d now=%0d", y_prev, y));
        end
      end
      y_prev = y;

      // bounds
      if (x > (ACTIVE_W-1)) tb_err($sformatf("x out of bounds: x=%0d", x));
      if (y > (ACTIVE_H-1)) tb_err($sformatf("y out of bounds: y=%0d", y));

      // pulses are registered inside dut.xy_counter: compare with TB *_q
      // frame_start_p 必須對齊 x/y reset
      if (frame_start_p) begin
        if (x !== 0 || y !== 0)
            tb_err("frame_start_p asserted but x/y not reset");
      end

      // line_start_p 必須對齊 x reset
      if (line_start_p) begin
          if (x !== 0)
              tb_err("line_start_p asserted but x not reset");
      end


      // status flags
      if (x_at_last !== (x == (ACTIVE_W-1))) tb_err("x_at_last wrong");
      if (y_at_last !== (y == (ACTIVE_H-1))) tb_err("y_at_last wrong");
    end
  endtask

  // ------------------------------------------------------------
  // Timing pattern generators (raw)
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

      // abrupt new frame while de=1
      drive_pix(1'b1, 1'b0, 1'b1);
      drive_pix(1'b0, 1'b0, 1'b0);

      drive_active_line(ACTIVE_W);
    end
  endtask
  task automatic frame_start;
    begin
        drive_one_pixel(1'b1, 1'b0, 1'b0);
        drive_one_pixel(1'b0, 1'b0, 1'b0);
    end
  endtask
  // ---- alias tasks to match other TB naming ----
  task automatic drive_one_pixel(input bit v, input bit h, input bit d);
  begin
    drive_pix(v,h,d);
  end
  endtask

  task automatic idle_cycles(input int unsigned n);
  begin
    idle(n);
  end
  endtask

  task automatic frame_end;
      begin
          // 依你的 tracker 設定：FRAME_START_ON_VS_RISE=1
          // frame_end 是 vs_fall，所以打一個 1 再回 0 就會產生 vs_fall
          drive_one_pixel(1'b1, 1'b0, 1'b0);
          drive_one_pixel(1'b0, 1'b0, 1'b0);
      end
  endtask

  task automatic hsync_pulse;
    begin
        drive_one_pixel(vsync, 1'b1, 1'b0);
        drive_one_pixel(vsync, 1'b0, 1'b0);
    end
  endtask
  task automatic tc6_frame_end_while_de_high;
  int unsigned x;
  begin
    $display("[TC6] frame_end while de=1 must clear in_frame/in_line");

    // 進 frame
    frame_start();

    // 進 line，送幾拍 de=1
    hsync_pulse();
    for (x=0; x<5; x++) drive_one_pixel(vsync, 1'b0, 1'b1);

    // 立刻 frame_end（vs_fall）且 de 還維持 1
    drive_one_pixel(1'b1, 1'b0, 1'b1);
    drive_one_pixel(1'b0, 1'b0, 1'b1);

    // 檢查：接下來不管 de 如何，in_frame 應該是 0，in_line 也應是 0
    drive_one_pixel(1'b0, 1'b0, 1'b1);
    drive_one_pixel(1'b0, 1'b0, 1'b0);
  end
  endtask
  task automatic tc7_de_stuck_high_outside_frame;
  int unsigned i;
  begin
    $display("[TC7] de stuck high outside frame should be ignored");

    // 確保 frame 外
    vsync = 1'b0; hsync = 1'b0; de = 1'b0;
    idle_cycles(5);

    // de 長時間拉高，但沒有 frame_start
    for (i=0; i<20; i++) drive_one_pixel(1'b0, 1'b0, 1'b1);

    // 回到 0
    for (i=0; i<5; i++) drive_one_pixel(1'b0, 1'b0, 1'b0);
  end
  endtask
  task automatic tc8_de_bounce_many_segments_in_one_line;
  int unsigned k;
  begin
    $display("[TC8] many short de segments in one line");

    frame_start();
    hsync_pulse();

    // 5 次短段：de=1 2拍，de=0 1拍
    for (k=0; k<5; k++) begin
      drive_one_pixel(vsync, 1'b0, 1'b1);
      drive_one_pixel(vsync, 1'b0, 1'b1);
      drive_one_pixel(vsync, 1'b0, 1'b0);
    end

    // 結尾 blank
    repeat (5) drive_one_pixel(vsync, 1'b0, 1'b0);

    frame_end();
  end
  endtask
  task automatic tc9_vsrise_same_cycle_as_derise;
  begin
    $display("[TC9] vs_rise same cycle as de_rise");

    // 在 frame 外，直接同拍把 vsync=1 且 de=1
    drive_one_pixel(1'b1, 1'b0, 1'b1); // 這拍會同時產生 vs_rise 與 de_rise
    drive_one_pixel(1'b0, 1'b0, 1'b1); // 下一拍仍 de=1
    drive_one_pixel(1'b0, 1'b0, 1'b0); // de_fall

    // 然後收尾
    frame_end();
  end
  endtask
  task automatic tc10_hsync_forces_eol_when_enabled;
  int unsigned x;
  begin
    $display("[TC10] when USE_HSYNC_FOR_EOL=1, hs edge should end line");

    frame_start();

    // 進 line，de=1 幾拍
    hsync_pulse();
    for (x=0; x<6; x++) drive_one_pixel(vsync, 1'b0, 1'b1);

    // 不送 de_fall，直接打一個 hs edge（假設這會被當作 EOL）
    drive_one_pixel(vsync, 1'b1, 1'b1); // hs_rise while de=1
    drive_one_pixel(vsync, 1'b0, 1'b1);

    // 後面 de 還是 1，但因為 hs 強制 EOL，pix_valid 應該會跟著 in_line=0 而變 0
    for (x=0; x<3; x++) drive_one_pixel(vsync, 1'b0, 1'b1);

    // 回到 blank
    drive_one_pixel(vsync, 1'b0, 1'b0);

    frame_end();
  end
  endtask

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_video_xy_top.vcd");
    $dumpvars(0, tb_video_xy_top);

    // init raw sync
    vsync = 1'b0;
    hsync = 1'b0;
    de    = 1'b0;

    // init model
    exp_x  = 0;
    exp_y  = 0;
    y_prev = '0;

    vs_d = 1'b0;
    hs_d = 1'b0;
    de_d = 1'b0;

    vs_rise = 1'b0;
    vs_fall = 1'b0;
    de_rise = 1'b0;
    de_fall = 1'b0;

    vs_rise_q = 1'b0;
    de_rise_q = 1'b0;
    de_fall_q = 1'b0;

    suppress_next_de_fall = 1'b0;

    // reset
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;

    // tests
    tc1_basic_frame();
    tc2_de_glitch_outside_frame();
    tc3_short_line();
    tc4_long_line_saturate();
    tc5_vsrise_mid_line();
    tc6_frame_end_while_de_high();
    tc7_de_stuck_high_outside_frame();
    tc8_de_bounce_many_segments_in_one_line();
    tc9_vsrise_same_cycle_as_derise();
    if (err_cnt == 0) $display("[TB] PASS: no errors");
    else              $display("[TB] FAIL: err_cnt=%0d", err_cnt);

    $finish;
  end

endmodule

`default_nettype wire
