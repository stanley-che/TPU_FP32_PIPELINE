// tb_tile_stats_4x4.sv
/*
iverilog -g2012 -Wall -o ./vvp/tb_tile_stats_4x4.vvp ./test/tb_tile_stats_4x4.sv
vvp ./vvp/tb_tile_stats_4x4.vvp
gtkwave tb_tile_stats_4x4.vcd
*/

`include "./src/AMOLED/tile feature_extracter/tile_stats_4x4.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_tile_stats_4x4;

  localparam int Y_W = 8;
  localparam int TILE_W = 4;
  localparam int TILE_H = 4;
  localparam int TILE_PIX = TILE_W*TILE_H;

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // DUT inputs
  logic                    v2_valid;
  logic [Y_W-1:0]           y_in;
  logic [$clog2(TILE_W)-1:0] x_mod;
  logic [$clog2(TILE_H)-1:0] y_mod;
  logic                    tile_last;
  logic [15:0]              y_cur;

  logic                    pix_in_tile_valid;
  logic [15:0]              tile_x_idx, tile_y_idx, frame_id;

  // DUT outputs
  logic                    v3_valid;
  logic                    v3_ready;
  logic [11:0]              sumY;       // 4x4 & Y_W=8 => 12bits OK
  logic [Y_W-1:0]           minY, maxY;
  logic [Y_W-1:0]           meanY, rangeY;
  logic [31:0]              sumSq;      // 這裡用大一點接（DUT 內部 SUMSQ_W 可能更小/大都不影響連接）
  logic [15:0]              y_cur_o, tile_x_o, tile_y_o, frame_id_o;

  logic                    tile_err;
  logic [7:0]              err_code;
  logic [$clog2(TILE_PIX+1)-1:0] pix_cnt_o;
  logic                    overflow_sum, overflow_sumsq;

  // instantiate DUT
  tile_stats_4x4 #(
    .Y_W(Y_W),
    .TILE_W(TILE_W),
    .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(1'b1),
    .DO_SUMSQ(1'b0),          // 先關掉 sumSq（你要測也可打開）
    .SATURATE_SUM(1'b0),
    .ASSERT_ON(1'b1)
  ) dut (
    .clk(clk),
    .rst(rst),

    .v2_valid(v2_valid),
    .y_in(y_in),
    .x_mod(x_mod),
    .y_mod(y_mod),
    .tile_last(tile_last),
    .y_cur(y_cur),

    .pix_in_tile_valid(pix_in_tile_valid),
    .tile_x_idx(tile_x_idx),
    .tile_y_idx(tile_y_idx),
    .frame_id(frame_id),

    .v3_valid(v3_valid),
    .v3_ready(v3_ready),

    .sumY(sumY),
    .minY(minY),
    .maxY(maxY),
    .meanY(meanY),
    .rangeY(rangeY),
    .sumSq(sumSq),

    .y_cur_o(y_cur_o),
    .tile_x_o(tile_x_o),
    .tile_y_o(tile_y_o),
    .frame_id_o(frame_id_o),

    .tile_err(tile_err),
    .err_code(err_code),
    .pix_cnt_o(pix_cnt_o),
    .overflow_sum(overflow_sum),
    .overflow_sumsq(overflow_sumsq)
  );

  // ------------------------------------------------------------
  // small utilities
  // ------------------------------------------------------------
  task automatic reset_dut();
    begin
      rst = 1'b1;
      v2_valid = 1'b0;
      x_mod = '0;
      y_mod = '0;
      y_in  = '0;
      tile_last = 1'b0;
      y_cur = '0;

      pix_in_tile_valid = 1'b1;
      tile_x_idx = 16'd0;
      tile_y_idx = 16'd0;
      frame_id   = 16'd0;

      v3_ready = 1'b1;

      repeat (5) @(posedge clk);
      rst = 1'b0;
      repeat (2) @(posedge clk);
    end
  endtask

  task automatic drive_pix(
    input int xm,
    input int ym,
    input int yv,
    input bit is_last,
    input bit pix_ok,
    input int yc
  );
    begin
      @(negedge clk);
      v2_valid          = 1'b1;
      x_mod             = xm[$bits(x_mod)-1:0];
      y_mod             = ym[$bits(y_mod)-1:0];
      y_in              = yv[Y_W-1:0];
      tile_last         = is_last;
      pix_in_tile_valid = pix_ok;
      y_cur             = yc[15:0];

      @(negedge clk);
      v2_valid          = 1'b0;
      tile_last         = 1'b0;
      pix_in_tile_valid = 1'b1;
    end
  endtask

  task automatic bubble_cycle(input int n);
  begin
    repeat (n) @(posedge clk);
  end
  endtask


  task automatic wait_v3_valid_or_timeout(input int max_cycles);
    int cyc;
    begin
      cyc = 0;
      while (v3_valid !== 1'b1 && cyc < max_cycles) begin
        @(posedge clk);
        cyc++;
      end
      if (v3_valid !== 1'b1) $fatal(1, "TIMEOUT: v3_valid not asserted within %0d cycles", max_cycles);
    end
  endtask

  task automatic expect_eq_int(input string name, input int got, input int exp);
    begin
      if (got !== exp) $fatal(1, "%s mismatch: got=%0d exp=%0d", name, got, exp);
    end
  endtask

  task automatic expect_eq_bit(input string name, input bit got, input bit exp);
    begin
      if (got !== exp) $fatal(1, "%s mismatch: got=%0b exp=%0b", name, got, exp);
    end
  endtask

  // ------------------------------------------------------------
  // TC1: normal 4x4 values 1..16
  // ------------------------------------------------------------
  task automatic tc1_basic();
    int v;
    begin
      $display("\n[TC1] basic 4x4 no bubble");
      tile_x_idx = 16'd3;
      tile_y_idx = 16'd5;
      frame_id   = 16'd7;

      v = 1;
      for (int ym=0; ym<4; ym++) begin
        for (int xm=0; xm<4; xm++) begin
          drive_pix(xm, ym, v, (xm==3 && ym==3), 1'b1, 16'd111);
          v++;
        end
      end

      wait_v3_valid_or_timeout(50);

      $display("  GOT sum=%0d min=%0d max=%0d mean=%0d range=%0d err=%0b code=%0h pixcnt=%0d",
               sumY, minY, maxY, meanY, rangeY, tile_err, err_code, pix_cnt_o);

      // expected
      expect_eq_int("sumY",  sumY, 136);
      expect_eq_int("minY",  minY, 1);
      expect_eq_int("maxY",  maxY, 16);
      expect_eq_int("meanY", meanY, (136 >> 4)); // 8
      expect_eq_int("rangeY",rangeY, (16-1));    // 15
      expect_eq_bit("tile_err", tile_err, 1'b0);

      // passthrough
      expect_eq_int("y_cur_o",    y_cur_o,    111);
      expect_eq_int("tile_x_o",   tile_x_o,   3);
      expect_eq_int("tile_y_o",   tile_y_o,   5);
      expect_eq_int("frame_id_o", frame_id_o, 7);

      // pix_cnt_o：期望最後 pixel index=15（若你 RTL off-by-one 這裡會抓到）
      expect_eq_int("pix_cnt_o", pix_cnt_o, 15);
    end
  endtask

  // ------------------------------------------------------------
  // TC2: bubble cycles between pixels (v2_valid断续)
  // ------------------------------------------------------------
  task automatic tc2_bubbles();
    int v;
    begin
      $display("\n[TC2] bubble between pixels");
      tile_x_idx = 16'd1;
      tile_y_idx = 16'd2;
      frame_id   = 16'd9;

      v = 1;
      for (int ym=0; ym<4; ym++) begin
        for (int xm=0; xm<4; xm++) begin
          drive_pix(xm, ym, v, (xm==3 && ym==3), 1'b1, 16'd222);
          bubble_cycle(1); // insert 1 idle cycle
          v++;
        end
      end

      wait_v3_valid_or_timeout(100);

      expect_eq_int("sumY", sumY, 136);
      expect_eq_bit("tile_err", tile_err, 1'b0);
      expect_eq_int("pix_cnt_o", pix_cnt_o, 15);
    end
  endtask

  // ------------------------------------------------------------
  // TC3: missing one pixel but still assert tile_last at end => should error
  // ------------------------------------------------------------
  task automatic tc3_missing_pixel_expect_err();
    int v;
    begin
      $display("\n[TC3] drop one pixel but still tile_last -> expect tile_err");
      tile_x_idx = 16'd4;
      tile_y_idx = 16'd4;
      frame_id   = 16'd4;

      v = 1;
      for (int ym=0; ym<4; ym++) begin
        for (int xm=0; xm<4; xm++) begin
          // drop the pixel at (1,2)
          if (!(xm==1 && ym==2)) begin
            drive_pix(xm, ym, v, (xm==3 && ym==3), 1'b1, 16'd333);
          end else begin
            // skipped: no v2_valid cycle for that pixel
          end
          v++;
        end
      end

      wait_v3_valid_or_timeout(300);

      $display("  GOT err=%0b code=%0h pixcnt=%0d", tile_err, err_code, pix_cnt_o);
      expect_eq_bit("tile_err", tile_err, 1'b1);
      // err_code 可能是 03（長度不對）或其他（看你 RTL 判斷），這裡至少要 != 0
      if (err_code == 8'h00) $fatal(1, "err_code should be nonzero when tile_err=1");
    end
  endtask

  // ------------------------------------------------------------
  // TC4: v3_ready hold behavior
  // ------------------------------------------------------------
  task automatic tc4_output_hold();
    int v;
    int sum_snap, min_snap, max_snap;
    begin
      $display("\n[TC4] v3_ready hold test");
      v3_ready = 1'b1;

      v = 1;
      for (int ym=0; ym<4; ym++) begin
        for (int xm=0; xm<4; xm++) begin
          drive_pix(xm, ym, v, (xm==3 && ym==3), 1'b1, 16'd444);
          v++;
        end
      end

      // 等到 v3_valid 出來
      wait_v3_valid_or_timeout(50);

      // 立刻拉 low，觀察 hold
      sum_snap = sumY;
      min_snap = minY;
      max_snap = maxY;

      v3_ready = 1'b0;
      repeat (5) @(posedge clk) begin
        if (v3_valid !== 1'b1) $fatal(1, "v3_valid should stay high while !v3_ready");
        if (sumY !== sum_snap || minY !== min_snap || maxY !== max_snap)
          $fatal(1, "output data changed during hold!");
      end

      // 放行
      v3_ready = 1'b1;
      @(posedge clk);
      // 下一拍應該可以 deassert（或至少不再保持）
      // 不強制一定為 0（看你 out_hold 實作），但通常會掉
      $display("  After release: v3_valid=%0b", v3_valid);
    end
  endtask

  // ------------------------------------------------------------
  // TC5: ROI / irregular tile via pix_in_tile_valid
  // - 只讓 10 個 pixel 有效，tile_last 打在最後一個有效 pixel
  // - 你目前 RTL 沒有 effective-count 模式 => integrity check 應該會報錯
  // - 同時也驗證「無效 pixel 不更新 sum/min/max」
  // ------------------------------------------------------------
  task automatic tc5_roi_expect_err_and_stats_match_valid_only();
  int v;
  int valid_sum, valid_min, valid_max;
  begin
    $display("\n[TC5] ROI gating: only send 10 valid pixels -> expect tile_err (not full 16)");
    valid_sum = 0;
    valid_min = 9999;
    valid_max = -1;

    // 這裡我們只送 10 個有效 pixel，模擬 ROI tile（不送後面無效的）
    // 位置仍然照 4x4 的掃描順序，送到 (1,2) 為止（共 10 點）
    v = 1;
    for (int ym=0; ym<4; ym++) begin
      for (int xm=0; xm<4; xm++) begin
        if ((ym*4 + xm) < 10) begin
          bit is_last;
          is_last = ((ym*4 + xm) == 9); // 第10個點打 tile_last

          drive_pix(xm, ym, v, is_last, 1'b1, 16'd555);

          valid_sum += v;
          if (v < valid_min) valid_min = v;
          if (v > valid_max) valid_max = v;
        end
        v++;
      end
    end

    wait_v3_valid_or_timeout(200);

    $display("  Valid-only expected sum=%0d min=%0d max=%0d", valid_sum, valid_min, valid_max);
    $display("  GOT sum=%0d min=%0d max=%0d err=%0b code=%0h pixcnt=%0d",
             sumY, minY, maxY, tile_err, err_code, pix_cnt_o);

    // stats 應該只算有效點
    expect_eq_int("sumY(valid-only)", sumY, valid_sum);
    expect_eq_int("minY(valid-only)", minY, valid_min);
    expect_eq_int("maxY(valid-only)", maxY, valid_max);

    // 因為不是滿16點，完整性檢查要報錯
    expect_eq_bit("tile_err", tile_err, 1'b1);
    if (err_code == 8'h00) $fatal(1, "err_code should be nonzero when tile_err=1");
  end
endtask


  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("tb_tile_stats_4x4.vcd");
    $dumpvars(0, tb_tile_stats_4x4);

    reset_dut();

    tc1_basic();
    reset_dut();

    tc2_bubbles();
    reset_dut();

    tc3_missing_pixel_expect_err();
    reset_dut();

    tc4_output_hold();
    reset_dut();

    tc5_roi_expect_err_and_stats_match_valid_only();
    reset_dut();

    $display("\nALL TESTCASES DONE");
    $finish;
  end

endmodule

`default_nettype wire
