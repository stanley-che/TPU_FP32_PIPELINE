// tb_active_window_tracker.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_active_window_tracker.vvp \
  ./test/tb_active_window_tracker.sv

vvp ./vvp/tb_active_window_tracker.vvp
gtkwave ./vvp/tb_active_window_tracker.vcd
*/
`include "./src/AMOLED/video_in_timing_if/active_window_tracker.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_active_window_tracker;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // ------------------------------------------------------------
  // stimulus: raw sync signals
  // ------------------------------------------------------------
  logic vsync, hsync, de;

  // edges (we generate here)
  logic vs_rise, vs_fall;
  logic hs_rise, hs_fall;
  logic de_rise, de_fall;

  // ------------------------------------------------------------
  // DUT outputs
  // ------------------------------------------------------------
  logic in_frame, in_line, pix_valid;

  // ------------------------------------------------------------
  // Parameters for a "tiny 720p-like" timing (fast sim)
  // ------------------------------------------------------------
  localparam int unsigned ACTIVE_W   = 16; // active pixels per line (de=1 length)
  localparam int unsigned HBLANK_W   = 8;  // blank pixels per line (de=0 length)
  localparam int unsigned ACTIVE_H   = 6;  // active lines per frame
  localparam int unsigned VBLANK_H   = 3;  // blank lines per frame

  localparam int unsigned FRAMES     = 3;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  active_window_tracker #(
    .FRAME_START_ON_VS_RISE(1'b1),
    .USE_HSYNC_FOR_EOL(1'b0)
  ) dut (
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
  // Edge generator (simple 1-cycle pulse)
  // ------------------------------------------------------------
  logic vs_d, hs_d, de_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_d <= 1'b0;
      hs_d <= 1'b0;
      de_d <= 1'b0;
    end else begin
      vs_d <= vsync;
      hs_d <= hsync;
      de_d <= de;
    end
  end

  always_comb begin
    vs_rise = (vsync && !vs_d);
    vs_fall = (!vsync && vs_d);
    hs_rise = (hsync && !hs_d);
    hs_fall = (!hsync && hs_d);
    de_rise = (de && !de_d);
    de_fall = (!de && de_d);
  end

  // ------------------------------------------------------------
  // Assertions / checks (simple, iverilog-friendly)
  // ------------------------------------------------------------
  int unsigned err_cnt = 0;

  task automatic check_one_cycle;
    logic expect_pix_valid;
    begin
      expect_pix_valid = (in_frame && in_line && de);

      if (pix_valid !== expect_pix_valid) begin
        $display("[ERR] t=%0t pix_valid mismatch: got=%0b expect=%0b (in_frame=%0b in_line=%0b de=%0b)",
                 $time, pix_valid, expect_pix_valid, in_frame, in_line, de);
        err_cnt++;
      end

      if (!in_frame && (in_line !== 1'b0)) begin
        $display("[ERR] t=%0t in_line must be 0 when !in_frame (in_line=%0b)", $time, in_line);
        err_cnt++;
      end

      if (!de && (pix_valid !== 1'b0)) begin
        $display("[ERR] t=%0t pix_valid must be 0 when de=0 (pix_valid=%0b)", $time, pix_valid);
        err_cnt++;
      end
    end
  endtask

  // ------------------------------------------------------------
  // Stimulus generator: frames/lines/pixels
  // ------------------------------------------------------------
  task automatic drive_one_pixel(input bit v, input bit h, input bit d);
    begin
      vsync <= v;
      hsync <= h;
      de    <= d;
      @(posedge clk);
      // 檢查用當拍 DUT 輸出（posedge 後穩定）
      #1;
      check_one_cycle();
    end
  endtask

  task automatic drive_one_line(input bit active_line);
    int unsigned x;
    begin
      // line start pulse on hsync (1-cycle)
      drive_one_pixel(vsync, 1'b1, 1'b0);
      drive_one_pixel(vsync, 1'b0, 1'b0);

      // active region (de=1) or full blank
      if (active_line) begin
        for (x = 0; x < ACTIVE_W; x++) begin
          drive_one_pixel(vsync, 1'b0, 1'b1);
        end
        // hblank
        for (x = 0; x < HBLANK_W; x++) begin
          drive_one_pixel(vsync, 1'b0, 1'b0);
        end
      end else begin
        // full blank line
        for (x = 0; x < (ACTIVE_W + HBLANK_W); x++) begin
          drive_one_pixel(vsync, 1'b0, 1'b0);
        end
      end
    end
  endtask

  task automatic drive_one_frame;
    int unsigned y;
    begin
      // frame start pulse on vsync (1-cycle high)
      drive_one_pixel(1'b1, 1'b0, 1'b0);
      drive_one_pixel(1'b0, 1'b0, 1'b0);

      // active lines
      for (y = 0; y < ACTIVE_H; y++) begin
        drive_one_line(1'b1);
      end

      // vblank lines
      for (y = 0; y < VBLANK_H; y++) begin
        drive_one_line(1'b0);
      end

      // frame end pulse (optional) - 我這裡用 vsync 再打一個 fall 的對應
      // 若你 DUT 用 vs_fall 當 end，這個能幫你測到 frame_end 清 line 的行為
      drive_one_pixel(1'b1, 1'b0, 1'b0);
      drive_one_pixel(1'b0, 1'b0, 1'b0);
    end
  endtask
    task automatic idle_cycles(input int unsigned n);
    int unsigned i;
    begin
      for (i = 0; i < n; i++) begin
        drive_one_pixel(vsync, hsync, de);
      end
    end
  endtask

  task automatic glitch_de_onecycle;
    // 在 blank 中插 1-cycle de=1 再回 0
    begin
      drive_one_pixel(vsync, 1'b0, 1'b0);
      drive_one_pixel(vsync, 1'b0, 1'b1);
      drive_one_pixel(vsync, 1'b0, 1'b0);
    end
  endtask
  task automatic drive_line_two_active_segments;
    int unsigned x;
    begin
      // hsync pulse
      drive_one_pixel(vsync, 1'b1, 1'b0);
      drive_one_pixel(vsync, 1'b0, 1'b0);

      // seg1 de=1
      for (x = 0; x < 6; x++) drive_one_pixel(vsync, 1'b0, 1'b1);
      for (x = 0; x < 4; x++) drive_one_pixel(vsync, 1'b0, 1'b0);

      // seg2 de=1
      for (x = 0; x < 5; x++) drive_one_pixel(vsync, 1'b0, 1'b1);
      for (x = 0; x < 5; x++) drive_one_pixel(vsync, 1'b0, 1'b0);
    end
  endtask
  task automatic drive_line_abort_mid_active;
    int unsigned x;
    begin
      // hsync pulse
      drive_one_pixel(vsync, 1'b1, 1'b0);
      drive_one_pixel(vsync, 1'b0, 1'b0);

      // 進入 active 一段，但不送 de_fall
      for (x = 0; x < 6; x++) drive_one_pixel(vsync, 1'b0, 1'b1);
      // 接著立刻 frame_end：打一個 vsync pulse
      drive_one_pixel(1'b1, 1'b0, 1'b1); // 同時 de 還是 1（極端情況）
      drive_one_pixel(1'b0, 1'b0, 1'b1);

      // 回到 0
      drive_one_pixel(1'b0, 1'b0, 1'b0);
      drive_one_pixel(1'b0, 1'b0, 1'b0);
    end
  endtask

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  int unsigned f;

  initial begin
    $dumpfile("./vvp/tb_active_window_tracker.vcd");
    $dumpvars(0, tb_active_window_tracker);

    // init
    vsync = 1'b0;
    hsync = 1'b0;
    de    = 1'b0;

    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;

    // run
    for (f = 0; f < FRAMES; f++) begin
      $display("[TB] drive frame %0d", f);
      drive_one_frame();
    end
    $display("[TC1] Reset release stable zeros");
    vsync = 1'b0; hsync = 1'b0; de = 1'b0;
    idle_cycles(30);
        $display("[TC2] de glitch while not in_frame (should be ignored)");
    // 確保在 frame 外
    vsync = 1'b0; hsync = 1'b0; de = 1'b0;
    idle_cycles(5);

    glitch_de_onecycle(); // 插入 de=1 glitch
    idle_cycles(5);
        $display("[TC3] Two active segments in one line");
    // 先進入 frame
    drive_one_pixel(1'b1, 1'b0, 1'b0);
    drive_one_pixel(1'b0, 1'b0, 1'b0);

    drive_line_two_active_segments();

    // 出 frame（打個 end pulse）
    drive_one_pixel(1'b1, 1'b0, 1'b0);
    drive_one_pixel(1'b0, 1'b0, 1'b0);
    $display("[TC4] Frame end clears in_line even without de_fall");
    // frame start
    drive_one_pixel(1'b1, 1'b0, 1'b0);
    drive_one_pixel(1'b0, 1'b0, 1'b0);

    drive_line_abort_mid_active();
    idle_cycles(10);
        $display("[TC5] hs glitch should not affect when USE_HSYNC_FOR_EOL=0");
    // frame start
    drive_one_pixel(1'b1, 1'b0, 1'b0);
    drive_one_pixel(1'b0, 1'b0, 1'b0);

    // 一行：de=1 期間 hs 亂跳
    drive_one_pixel(vsync, 1'b1, 1'b0);
    drive_one_pixel(vsync, 1'b0, 1'b0);

    repeat (10) begin
      // de=1，同時 hs 每拍亂跳
      hsync = ~$random;
      drive_one_pixel(vsync, hsync, 1'b1);
    end
    repeat (6) begin
      hsync = ~$random;
      drive_one_pixel(vsync, hsync, 1'b0);
    end

    // frame end
    drive_one_pixel(1'b1, 1'b0, 1'b0);
    drive_one_pixel(1'b0, 1'b0, 1'b0);

    // report
    if (err_cnt == 0) begin
      $display("[TB] PASS: no errors");
    end else begin
      $display("[TB] FAIL: err_cnt=%0d", err_cnt);
    end

    $finish;
  end

endmodule

`default_nettype wire
