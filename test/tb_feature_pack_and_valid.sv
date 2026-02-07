// tb_feature_pack_and_valid.sv
/*
Build:
  iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_feature_pack_and_valid.vvp \
    ./test/tb_feature_pack_and_valid.sv

Run:
  vvp ./vvp/tb_feature_pack_and_valid.vvp

Wave:
  gtkwave ./vvp/tb_feature_pack_and_valid.vcd
*/

`include "./src/AMOLED/tile feature_extracter/feature_pack_and_valid.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_feature_pack_and_valid;

  localparam int unsigned TILES_X = 320;
  localparam int unsigned TILES_Y = 180;

  localparam int unsigned SUM_W  = 12;
  localparam int unsigned Y_W    = 8;
  localparam int unsigned EDGE_W = 32;

  localparam int unsigned FEAT_W = 16;

  localparam int unsigned TILE_I_W  = 16;
  localparam int unsigned TILE_J_W  = 16;
  localparam int unsigned TILE_ID_W = $clog2(TILES_X*TILES_Y);

  localparam bit EDGE_USE_MEAN = 1'b1;

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // -----------------------
  // DUT inputs
  // -----------------------
  logic                 v3_valid;
  logic [SUM_W-1:0]      sumY;
  logic [Y_W-1:0]        minY, maxY;
  logic [TILE_I_W-1:0]   tile_i;
  logic [TILE_J_W-1:0]   tile_j;

  logic                 v4_valid;
  logic [EDGE_W-1:0]     edge_sum;
  logic [TILE_I_W-1:0]   v4_tile_i;
  logic [TILE_J_W-1:0]   v4_tile_j;

  logic                 feat_ready;

  // -----------------------
  // DUT outputs
  // -----------------------
  logic                 v3_ready, v4_ready;
  logic                 feat_valid;

  logic [TILE_I_W-1:0]   tile_i_o;
  logic [TILE_J_W-1:0]   tile_j_o;
  logic [TILE_ID_W-1:0]  tile_id_o;

  logic [FEAT_W-1:0]     feat0, feat1, feat2, feat3;
  logic [FEAT_W-1:0]     feat4, feat5, feat6, feat7;
  logic [8*FEAT_W-1:0]   feat_vec;

  logic                 err_mismatch_pulse;
  logic [31:0]          cnt_join_ok, cnt_mismatch, cnt_drop;

  // ============================================================
  // DUT
  // 重要：這裡用 MATCH_MODE_BUFFERED=1，才能「容忍 v3/v4 skew」
  // ============================================================
  feature_pack_and_valid #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .SUM_W(SUM_W),
    .Y_W(Y_W),
    .EDGE_W(EDGE_W),
    .FEAT_W(FEAT_W),
    .FEAT_DIM(4),
    .EDGE_USE_MEAN(EDGE_USE_MEAN),

    .TILE_PIX_SHIFT(4),
    .MEAN_MODE_SHIFT(1'b1),
    .TILE_PIX_N(16),

    .MATCH_MODE_BUFFERED(1'b1),
    .ZERO_WHEN_INVALID(1'b0),

    .TILE_I_W(TILE_I_W),
    .TILE_J_W(TILE_J_W),
    .TILE_ID_W(TILE_ID_W),

    .ENABLE_CNTS(1'b1)
  ) dut (
    .clk(clk),
    .rst(rst),

    .v3_valid(v3_valid),
    .v3_ready(v3_ready),
    .sumY(sumY),
    .minY(minY),
    .maxY(maxY),
    .tile_i(tile_i),
    .tile_j(tile_j),

    .v4_valid(v4_valid),
    .v4_ready(v4_ready),
    .edge_sum(edge_sum),
    .v4_tile_i(v4_tile_i),
    .v4_tile_j(v4_tile_j),

    .feat_valid(feat_valid),
    .feat_ready(feat_ready),

    .tile_i_o(tile_i_o),
    .tile_j_o(tile_j_o),
    .tile_id_o(tile_id_o),

    .feat0(feat0),
    .feat1(feat1),
    .feat2(feat2),
    .feat3(feat3),
    .feat4(feat4),
    .feat5(feat5),
    .feat6(feat6),
    .feat7(feat7),
    .feat_vec(feat_vec),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

  // ============================================================
  // Helpers
  // ============================================================
  task automatic step;
    begin
      @(posedge clk);
      #1;
    end
  endtask

  function automatic int tile_id_calc(input int ti, input int tj);
    tile_id_calc = ti*TILES_X + tj;
  endfunction

  task automatic apply_reset;
    begin
      rst = 1'b1;

      v3_valid   = 1'b0;
      v4_valid   = 1'b0;
      feat_ready = 1'b1;

      sumY     = '0;
      minY     = '0;
      maxY     = '0;
      tile_i   = '0;
      tile_j   = '0;

      edge_sum  = '0;
      v4_tile_i = '0;
      v4_tile_j = '0;

      repeat (8) @(posedge clk);
      rst = 1'b0;
      repeat (2) @(posedge clk);

      // tests 默认用 hold output
      feat_ready = 1'b0;
      step();
    end
  endtask

  task automatic wait_feat_valid(input int max_cycles);
    int c;
    begin
      c = 0;
      while ((feat_valid !== 1'b1) && (c < max_cycles)) begin
        step();
        c++;
      end
      if (feat_valid !== 1'b1) begin
        $display("[DBG] v3_valid=%0b v3_ready=%0b v4_valid=%0b v4_ready=%0b feat_valid=%0b feat_ready=%0b err_mismatch=%0b",
                 v3_valid, v3_ready, v4_valid, v4_ready, feat_valid, feat_ready, err_mismatch_pulse);
        $display("[DBG] cnt_join_ok=%0d cnt_mismatch=%0d cnt_drop=%0d", cnt_join_ok, cnt_mismatch, cnt_drop);
        $fatal(1, "[TB] TIMEOUT waiting feat_valid");
      end
    end
  endtask

  task automatic consume_one;
    begin
      feat_ready = 1'b1;
      step();
      feat_ready = 1'b0;
      step();
    end
  endtask

  task automatic check_out(
    input int exp_ti,
    input int exp_tj,
    input int exp_sumY,
    input int exp_minY,
    input int exp_maxY,
    input int exp_edge_sum
  );
    int exp_ymean;
    int exp_edge_feat;
    int exp_tile_id;
    begin
      exp_ymean     = (exp_sumY >> 4);
      exp_edge_feat = (EDGE_USE_MEAN) ? (exp_edge_sum >> 4) : exp_edge_sum;
      exp_tile_id   = tile_id_calc(exp_ti, exp_tj);

      if (feat_valid !== 1'b1) $fatal(1, "[TB] expected feat_valid=1");

      if (tile_i_o !== exp_ti[TILE_I_W-1:0]) $fatal(1, "[TB] tile_i_o mismatch got=%0d exp=%0d", tile_i_o, exp_ti);
      if (tile_j_o !== exp_tj[TILE_J_W-1:0]) $fatal(1, "[TB] tile_j_o mismatch got=%0d exp=%0d", tile_j_o, exp_tj);
      if (tile_id_o !== exp_tile_id[TILE_ID_W-1:0]) $fatal(1, "[TB] tile_id_o mismatch got=%0d exp=%0d", tile_id_o, exp_tile_id);

      if (feat0 !== exp_ymean[FEAT_W-1:0]) $fatal(1, "[TB] feat0(Y_mean) mismatch got=%0d exp=%0d", feat0, exp_ymean);
      if (feat1 !== exp_maxY[FEAT_W-1:0])  $fatal(1, "[TB] feat1(Y_max) mismatch got=%0d exp=%0d", feat1, exp_maxY);
      if (feat2 !== exp_minY[FEAT_W-1:0])  $fatal(1, "[TB] feat2(Y_min) mismatch got=%0d exp=%0d", feat2, exp_minY);
      if (feat3 !== exp_edge_feat[FEAT_W-1:0]) $fatal(1, "[TB] feat3(edge) mismatch got=%0d exp=%0d", feat3, exp_edge_feat);

      // 8-wide packing: {feat7..feat0}
      if (feat_vec !== {feat7, feat6, feat5, feat4, feat3, feat2, feat1, feat0})
        $fatal(1, "[TB] feat_vec packing mismatch");
    end
  endtask

  // Drive helpers:
  // 注意：v4_tile_i/j 必須「在 v4_valid=1 的同一段期間」就穩定！
  task automatic drive_v3(
    input int ti, input int tj,
    input int s, input int mn, input int mx,
    input int hold_cycles
  );
    begin
      tile_i   = ti[TILE_I_W-1:0];
      tile_j   = tj[TILE_J_W-1:0];
      sumY     = s[SUM_W-1:0];
      minY     = mn[Y_W-1:0];
      maxY     = mx[Y_W-1:0];
      v3_valid = 1'b1;
      repeat (hold_cycles) step();
    end
  endtask

  task automatic drive_v4(
    input int ti, input int tj,
    input int e,
    input int hold_cycles
  );
    begin
      v4_tile_i = ti[TILE_I_W-1:0];
      v4_tile_j = tj[TILE_J_W-1:0];
      edge_sum  = e[EDGE_W-1:0];
      v4_valid  = 1'b1;
      repeat (hold_cycles) step();
    end
  endtask

  task automatic drop_inputs;
    begin
      v3_valid = 1'b0;
      v4_valid = 1'b0;
      step();
    end
  endtask

  // ============================================================
  // Testcases
  // ============================================================

  // TC1: 同拍 join (buffered mode 也会过)
  task automatic tc1_basic_sync_join;
    begin
      $display("\n[TC1] basic sync join");

      feat_ready = 1'b0;

      drive_v3(5, 7, 136, 1, 16, 0);
      drive_v4(5, 7, 160, 0);

      // 等 join 出来
      wait_feat_valid(200);
      check_out(5, 7, 136, 1, 16, 160);

      drop_inputs();

      // consume
      consume_one();
      if (feat_valid !== 1'b0) $fatal(1, "[TB] feat_valid should drop after consume");
    end
  endtask

  // TC2: v3 早到, v4 晚到 (buffered mode 必过)
  task automatic tc2_skew_v3_early_v4_late;
    begin
      $display("\n[TC2] buffered: v3 early, v4 late");

      feat_ready = 1'b0;

      drive_v3(9, 3, 320, 5, 40, 1);   // keep v3_valid 1-cycle
      v3_valid = 1'b0;
      repeat (5) step();

      drive_v4(9, 3, 64, 1);
      v4_valid = 1'b0;

      wait_feat_valid(200);
      check_out(9, 3, 320, 5, 40, 64);

      consume_one();
    end
  endtask

  // TC3: backpressure hold (feat_ready=0) -> 输出稳定，上游 ready 被拉低
  task automatic tc3_backpressure_hold;
    logic [FEAT_W-1:0] s0, s1, s2, s3;
    logic [TILE_I_W-1:0] sti;
    logic [TILE_J_W-1:0] stj;
    logic [TILE_ID_W-1:0] sid;
    begin
      $display("\n[TC3] output hold / upstream backpressure");

      feat_ready = 1'b0;

      drive_v3(1, 2, 160, 3, 25, 0);
      drive_v4(1, 2, 256, 0);

      wait_feat_valid(200);
      check_out(1, 2, 160, 3, 25, 256);

      s0 = feat0; s1 = feat1; s2 = feat2; s3 = feat3;
      sti = tile_i_o; stj = tile_j_o; sid = tile_id_o;

      repeat (8) begin
        step();
        if (feat_valid !== 1'b1) $fatal(1, "[TB] feat_valid must stay high while feat_ready=0");
        if (feat0 !== s0 || feat1 !== s1 || feat2 !== s2 || feat3 !== s3) $fatal(1, "[TB] features changed during hold");
        if (tile_i_o !== sti || tile_j_o !== stj || tile_id_o !== sid) $fatal(1, "[TB] meta changed during hold");
        // buffered mode 下：v3_ready/v4_ready 取决于 buf 是否空。
        // join 成功后 buf 会清空，但 hold_valid=1 时 can_take_out=0，会让 buffered_take 不发生；
        // 然而 v3_ready/v4_ready 在 buffered 模式是 !buf*_valid，所以可能仍为 1。
        // 因此这里不强制检查 v3_ready/v4_ready。
      end

      drop_inputs();
      consume_one();
    end
  endtask

  // TC4: back-to-back 两笔（每笔都先 hold，再 consume）
  task automatic tc4_two_tiles_back_to_back;
    begin
      $display("\n[TC4] two tiles back-to-back");

      feat_ready = 1'b0;

      // A
      drive_v3(10, 20, 144, 1, 30, 0);
      drive_v4(10, 20, 80,  0);

      wait_feat_valid(200);
      check_out(10, 20, 144, 1, 30, 80);

      drop_inputs();
      consume_one();

      // B
      drive_v3(11, 21, 256, 9, 44, 0);
      drive_v4(11, 21, 160, 0);

      wait_feat_valid(200);
      check_out(11, 21, 256, 9, 44, 160);

      drop_inputs();
      consume_one();
    end
  endtask

  // TC5: mismatch -> drop/resync（buffered mode: buf3/buf4 都满且 id 不同，会丢掉两笔）
  task automatic tc5_mismatch_drop_resync;
    logic [31:0] m0, d0;
    begin
      $display("\n[TC5] mismatch drop/resync (buffered)");

      feat_ready = 1'b0;

      m0 = cnt_mismatch;
      d0 = cnt_drop;

      // mismatched pair
      drive_v3(5, 7, 160, 1, 2, 1);
      v3_valid = 1'b0;

      drive_v4(5, 8, 160, 1); // mismatch (tj=8)
      v4_valid = 1'b0;

      // 等 mismatch 发生（cnt_mismatch / cnt_drop 会增长）
      repeat (10) step();
      if (cnt_mismatch == m0) $fatal(1, "[TB] expected cnt_mismatch to increase");
      if (cnt_drop == d0)     $fatal(1, "[TB] expected cnt_drop to increase");

      // resync: correct pair should work
      drive_v3(6, 9, 144, 3, 7, 0);
      drive_v4(6, 9, 80,  0);

      wait_feat_valid(200);
      check_out(6, 9, 144, 3, 7, 80);

      drop_inputs();
      consume_one();
    end
  endtask

  // TC6: counters sanity (至少 join_ok 会增长)
  task automatic tc6_counters_sanity;
    logic [31:0] j0;
    begin
      $display("\n[TC6] counters sanity");

      j0 = cnt_join_ok;

      feat_ready = 1'b0;

      drive_v3(2, 2, 160, 1, 2, 0);
      drive_v4(2, 2, 160, 0);

      wait_feat_valid(200);
      drop_inputs();
      consume_one();

      if (cnt_join_ok != (j0 + 1)) $fatal(1, "[TB] cnt_join_ok not incremented");
    end
  endtask

  // ============================================================
  // MAIN
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_feature_pack_and_valid.vcd");
    $dumpvars(0, tb_feature_pack_and_valid);

    apply_reset();

    tc1_basic_sync_join();       apply_reset();
    tc2_skew_v3_early_v4_late(); apply_reset();
    tc3_backpressure_hold();     apply_reset();
    tc4_two_tiles_back_to_back();apply_reset();
    tc5_mismatch_drop_resync();  apply_reset();
    tc6_counters_sanity();       apply_reset();

    $display("\nALL TESTS PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
