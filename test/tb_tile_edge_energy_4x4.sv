// tb_tile_edge_energy_4x4.sv
/*
iverilog -g2012 -Wall -o ./vvp/tb_tile_edge_energy_4x4.vvp ./test/tb_tile_edge_energy_4x4.sv
vvp ./vvp/tb_tile_edge_energy_4x4.vvp
gtkwave tb_tile_edge_energy_4x4.vcd
*/

`include "./src/AMOLED/tile feature_extracter/tile_edge_energy_4x4.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_tile_edge_energy;

  localparam int Y_W    = 8;
  localparam int TILE_W = 4;
  localparam int TILE_H = 4;

  // TB 端固定寬度（避免不同 EDGE_W 造成連接麻煩）
  localparam int EDGE_W = 32;
  localparam int CNT_W  = 8;
  localparam int THR    = 8;

  logic clk = 1'b0; always #5 clk = ~clk;
  logic rst;

  // shared stimulus
  logic                     v2_valid;
  logic [Y_W-1:0]            y_cur, y_left, y_up;
  logic [$clog2(TILE_W)-1:0] x_mod;
  logic [$clog2(TILE_H)-1:0] y_mod;
  logic                     tile_last;
  logic                     tile_start;
  logic                     pix_in_tile_valid;

  // per DUT ready/valid
  logic v4_ready_0, v4_ready_1, v4_ready_2, v4_ready_3;

  // MODE0 outputs
  logic v2_ready_0, v4_valid_0;
  logic [EDGE_W-1:0] edge_sum_0, edge_max_0, edge_mean_0;
  logic [CNT_W-1:0]  edge_cnt_0;

  // MODE1 outputs
  logic v2_ready_1, v4_valid_1;
  logic [EDGE_W-1:0] edge_sum_1, edge_max_1, edge_mean_1;
  logic [CNT_W-1:0]  edge_cnt_1;

  // MODE2 outputs
  logic v2_ready_2, v4_valid_2;
  logic [EDGE_W-1:0] edge_sum_2, edge_max_2, edge_mean_2;
  logic [CNT_W-1:0]  edge_cnt_2;

  // MODE3 outputs
  logic v2_ready_3, v4_valid_3;
  logic [EDGE_W-1:0] edge_sum_3, edge_max_3, edge_mean_3;
  logic [CNT_W-1:0]  edge_cnt_3;

  // ----------------------------
  // DUTs (different MODE param)
  // ----------------------------
  tile_edge_energy_4x4 #(
    .Y_W(Y_W), .TILE_W(TILE_W), .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(1'b1),
    .SUPPORT_TILE_START(1'b1),
    .ASSERT_ON(1'b0),
    .MODE(0), .THR(THR),
    .EDGE_W(EDGE_W), .MEAN_W(EDGE_W), .CNT_W(CNT_W)
  ) dut0 (
    .clk, .rst,
    .v2_valid, .v2_ready(v2_ready_0),
    .y_cur, .y_left, .y_up, .x_mod, .y_mod,
    .tile_last, .tile_start, .pix_in_tile_valid,
    .v4_valid(v4_valid_0), .v4_ready(v4_ready_0),
    .edge_sum(edge_sum_0), .edge_max(edge_max_0),
    .edge_cnt(edge_cnt_0), .edge_mean(edge_mean_0)
  );

  tile_edge_energy_4x4 #(
    .Y_W(Y_W), .TILE_W(TILE_W), .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(1'b1),
    .SUPPORT_TILE_START(1'b1),
    .ASSERT_ON(1'b0),
    .MODE(1), .THR(THR),
    .EDGE_W(EDGE_W), .MEAN_W(EDGE_W), .CNT_W(CNT_W)
  ) dut1 (
    .clk, .rst,
    .v2_valid, .v2_ready(v2_ready_1),
    .y_cur, .y_left, .y_up, .x_mod, .y_mod,
    .tile_last, .tile_start, .pix_in_tile_valid,
    .v4_valid(v4_valid_1), .v4_ready(v4_ready_1),
    .edge_sum(edge_sum_1), .edge_max(edge_max_1),
    .edge_cnt(edge_cnt_1), .edge_mean(edge_mean_1)
  );

  tile_edge_energy_4x4 #(
    .Y_W(Y_W), .TILE_W(TILE_W), .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(1'b1),
    .SUPPORT_TILE_START(1'b1),
    .ASSERT_ON(1'b0),
    .MODE(2), .THR(THR),
    .EDGE_W(EDGE_W), .MEAN_W(EDGE_W), .CNT_W(CNT_W)
  ) dut2 (
    .clk, .rst,
    .v2_valid, .v2_ready(v2_ready_2),
    .y_cur, .y_left, .y_up, .x_mod, .y_mod,
    .tile_last, .tile_start, .pix_in_tile_valid,
    .v4_valid(v4_valid_2), .v4_ready(v4_ready_2),
    .edge_sum(edge_sum_2), .edge_max(edge_max_2),
    .edge_cnt(edge_cnt_2), .edge_mean(edge_mean_2)
  );

  tile_edge_energy_4x4 #(
    .Y_W(Y_W), .TILE_W(TILE_W), .TILE_H(TILE_H),
    .SUPPORT_PIX_VALID(1'b1),
    .SUPPORT_TILE_START(1'b1),
    .ASSERT_ON(1'b0),
    .MODE(3), .THR(THR),
    .EDGE_W(EDGE_W), .MEAN_W(EDGE_W), .CNT_W(CNT_W)
  ) dut3 (
    .clk, .rst,
    .v2_valid, .v2_ready(v2_ready_3),
    .y_cur, .y_left, .y_up, .x_mod, .y_mod,
    .tile_last, .tile_start, .pix_in_tile_valid,
    .v4_valid(v4_valid_3), .v4_ready(v4_ready_3),
    .edge_sum(edge_sum_3), .edge_max(edge_max_3),
    .edge_cnt(edge_cnt_3), .edge_mean(edge_mean_3)
  );

  // ----------------------------
  // Tile data stored as 1D arrays
  // idx = ym*4 + xm
  // ----------------------------
  integer tileA [0:15];
  integer tileB [0:15];
  reg     valid_all [0:15];
  reg     valid_mask [0:15];

  function integer idx(input integer xm, input integer ym);
    idx = ym*4 + xm;
  endfunction

  function integer iabs(input integer a);
    if (a < 0) iabs = -a; else iabs = a;
  endfunction

  function integer sq(input integer a);
    sq = a*a;
  endfunction

  // ----------------------------
  // Init
  // ----------------------------
  task init_signals;
    begin
      v2_valid = 0;
      y_cur = '0; y_left='0; y_up='0;
      x_mod='0; y_mod='0;
      tile_last=0; tile_start=0;
      pix_in_tile_valid=1;

      v4_ready_0=1; v4_ready_1=1; v4_ready_2=1; v4_ready_3=1;
    end
  endtask

  // wait selected DUT input ready
  task wait_in_ready(input integer dut_id);
    begin
      while (1) begin
        @(posedge clk);
        if ((dut_id==0 && v2_ready_0) ||
            (dut_id==1 && v2_ready_1) ||
            (dut_id==2 && v2_ready_2) ||
            (dut_id==3 && v2_ready_3)) begin
          disable wait_in_ready;
        end
      end
    end
  endtask

  // wait selected DUT output accepted (valid & ready)
  task wait_out_accept(input integer dut_id, input integer max_cycles);
    integer cyc;
    begin
      for (cyc=0; cyc<max_cycles; cyc=cyc+1) begin
        @(posedge clk);
        if ((dut_id==0 && v4_valid_0 && v4_ready_0) ||
            (dut_id==1 && v4_valid_1 && v4_ready_1) ||
            (dut_id==2 && v4_valid_2 && v4_ready_2) ||
            (dut_id==3 && v4_valid_3 && v4_ready_3)) begin
          disable wait_out_accept;
        end
      end
      $fatal(1, "TIMEOUT: out_accept dut=%0d", dut_id);
    end
  endtask

  // drive single pixel (x_mod/y_mod given), waits v2_ready of that DUT
  task drive_pix(
    input integer dut_id,
    input integer xm,
    input integer ym,
    input integer cur,
    input integer left,
    input integer up,
    input reg     is_first,
    input reg     is_last,
    input reg     pix_ok
  );
    begin
      wait_in_ready(dut_id);

      @(negedge clk);
      v2_valid = 1'b1;
      x_mod = xm[$bits(x_mod)-1:0];
      y_mod = ym[$bits(y_mod)-1:0];
      y_cur  = cur[Y_W-1:0];
      y_left = left[Y_W-1:0];
      y_up   = up[Y_W-1:0];

      tile_start = is_first;
      tile_last  = is_last;
      pix_in_tile_valid = pix_ok;

      @(posedge clk);
      @(negedge clk);
      v2_valid = 1'b0;
      tile_start = 1'b0;
      tile_last  = 1'b0;
    end
  endtask

  // compute expected using 1D arrays (no unpacked array args)
  task compute_expected(
    input integer mode,
    input integer thr,
    input integer use_tileA,     // 1: tileA, 0: tileB
    input integer use_mask,      // 1: valid_mask, 0: valid_all
    output integer exp_sum,
    output integer exp_max,
    output integer exp_cnt
  );
    integer ym, xm;
    integer cur, left, up;
    integer dx, dy, e_abs, e, valid;
    integer base_idx;
    begin
      exp_sum = 0; exp_max = 0; exp_cnt = 0;

      for (ym=0; ym<4; ym=ym+1) begin
        for (xm=0; xm<4; xm=xm+1) begin
          base_idx = idx(xm, ym);
          cur  = use_tileA ? tileA[base_idx] : tileB[base_idx];

          if (xm==0) left = cur;
          else      left = use_tileA ? tileA[idx(xm-1,ym)] : tileB[idx(xm-1,ym)];

          if (ym==0) up = cur;
          else      up = use_tileA ? tileA[idx(xm,ym-1)] : tileB[idx(xm,ym-1)];

          valid = use_mask ? valid_mask[base_idx] : valid_all[base_idx];

          dx = (xm==0) ? 0 : iabs(cur - left);
          dy = (ym==0) ? 0 : iabs(cur - up);
          e_abs = dx + dy;

          // cnt definition: valid && (e_abs > thr)
          if (valid && (e_abs > thr)) exp_cnt = exp_cnt + 1;

          if (!valid) begin
            e = 0; // B-mode invalid => 0 contribution
          end else begin
            case (mode)
              0: e = e_abs;
              1: e = (dx>dy) ? dx : dy;
              2: e = (e_abs > thr) ? 1 : 0;
              3: e = sq(dx) + sq(dy);
              default: e = e_abs;
            endcase
          end

          exp_sum = exp_sum + e;
          if (e > exp_max) exp_max = e;
        end
      end
    end
  endtask

  // drive full tile (tileA or tileB, with mask or all-valid)
  task drive_tile(
    input integer dut_id,
    input integer use_tileA,
    input integer use_mask,
    input reg     use_tile_start_pulse
  );
    integer ym, xm;
    integer cur, left, up;
    integer base_idx;
    reg first, last, ok;
    begin
      for (ym=0; ym<4; ym=ym+1) begin
        for (xm=0; xm<4; xm=xm+1) begin
          base_idx = idx(xm, ym);
          cur = use_tileA ? tileA[base_idx] : tileB[base_idx];

          if (xm==0) left = cur;
          else      left = use_tileA ? tileA[idx(xm-1,ym)] : tileB[idx(xm-1,ym)];

          if (ym==0) up = cur;
          else      up = use_tileA ? tileA[idx(xm,ym-1)] : tileB[idx(xm,ym-1)];

          ok = use_mask ? valid_mask[base_idx] : valid_all[base_idx];

          first = (xm==0 && ym==0) ? 1'b1 : 1'b0;
          last  = (xm==3 && ym==3) ? 1'b1 : 1'b0;

          drive_pix(
            dut_id, xm, ym, cur, left, up,
            (use_tile_start_pulse ? first : 1'b0),
            last,
            ok
          );
        end
      end
    end
  endtask

  task check_outputs(
    input integer dut_id,
    input integer exp_sum,
    input integer exp_max,
    input integer exp_cnt
  );
    integer exp_mean;
    begin
      exp_mean = exp_sum / 16;

      case (dut_id)
        0: begin
          if (edge_sum_0  !== exp_sum)  $fatal(1,"dut0 sum got=%0d exp=%0d", edge_sum_0, exp_sum);
          if (edge_max_0  !== exp_max)  $fatal(1,"dut0 max got=%0d exp=%0d", edge_max_0, exp_max);
          if (edge_cnt_0  !== exp_cnt[CNT_W-1:0]) $fatal(1,"dut0 cnt got=%0d exp=%0d", edge_cnt_0, exp_cnt);
          if (edge_mean_0 !== exp_mean) $fatal(1,"dut0 mean got=%0d exp=%0d", edge_mean_0, exp_mean);
        end
        1: begin
          if (edge_sum_1  !== exp_sum)  $fatal(1,"dut1 sum got=%0d exp=%0d", edge_sum_1, exp_sum);
          if (edge_max_1  !== exp_max)  $fatal(1,"dut1 max got=%0d exp=%0d", edge_max_1, exp_max);
          if (edge_cnt_1  !== exp_cnt[CNT_W-1:0]) $fatal(1,"dut1 cnt got=%0d exp=%0d", edge_cnt_1, exp_cnt);
          if (edge_mean_1 !== exp_mean) $fatal(1,"dut1 mean got=%0d exp=%0d", edge_mean_1, exp_mean);
        end
        2: begin
          if (edge_sum_2  !== exp_sum)  $fatal(1,"dut2 sum got=%0d exp=%0d", edge_sum_2, exp_sum);
          if (edge_max_2  !== exp_max)  $fatal(1,"dut2 max got=%0d exp=%0d", edge_max_2, exp_max);
          if (edge_cnt_2  !== exp_cnt[CNT_W-1:0]) $fatal(1,"dut2 cnt got=%0d exp=%0d", edge_cnt_2, exp_cnt);
          if (edge_mean_2 !== exp_mean) $fatal(1,"dut2 mean got=%0d exp=%0d", edge_mean_2, exp_mean);
        end
        3: begin
          if (edge_sum_3  !== exp_sum)  $fatal(1,"dut3 sum got=%0d exp=%0d", edge_sum_3, exp_sum);
          if (edge_max_3  !== exp_max)  $fatal(1,"dut3 max got=%0d exp=%0d", edge_max_3, exp_max);
          if (edge_cnt_3  !== exp_cnt[CNT_W-1:0]) $fatal(1,"dut3 cnt got=%0d exp=%0d", edge_cnt_3, exp_cnt);
          if (edge_mean_3 !== exp_mean) $fatal(1,"dut3 mean got=%0d exp=%0d", edge_mean_3, exp_mean);
        end
      endcase
    end
  endtask

  // ----------------------------
  // MAIN
  // ----------------------------
  integer exp_sum, exp_max, exp_cnt;
  integer i;

  initial begin
    $dumpfile("tb_tile_edge_energy.vcd");
    $dumpvars(0, tb_tile_edge_energy);

    // init tileA = 1..16
    for (i=0; i<16; i=i+1) begin
      tileA[i] = i+1;
      valid_all[i]  = 1'b1;
      valid_mask[i] = 1'b1;
    end
    // tileB = 16..1
    for (i=0; i<16; i=i+1) begin
      tileB[i] = 16 - i;
    end
    // holes in mask
    valid_mask[idx(1,1)] = 1'b0;
    valid_mask[idx(0,2)] = 1'b0;
    valid_mask[idx(2,3)] = 1'b0;

    // reset
    rst = 1'b1;
    init_signals();
    repeat (5) @(posedge clk);
    rst = 1'b0;

    // ----------------------------
    // Case0: MODE0 basic
    // ----------------------------
    $display("\n[CASE0] MODE0 basic");
    compute_expected(0, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 1, 0, 1'b1);
    wait_out_accept(0, 200);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case1: MODE0 backpressure at tile_last
    // ----------------------------
    $display("\n[CASE1] MODE0 backpressure");
    fork
      begin
        repeat (20) @(posedge clk);
        v4_ready_0 = 1'b0;
        repeat (4) @(posedge clk);
        v4_ready_0 = 1'b1;
      end
    join_none

    compute_expected(0, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 1, 0, 1'b1);
    wait_out_accept(0, 400);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case2: tile_start pulse
    // ----------------------------
    $display("\n[CASE2] tile_start alignment pulse");
    compute_expected(0, THR, 0, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 0, 0, 1'b1);
    wait_out_accept(0, 200);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case3: pix_in_tile_valid holes (B-mode => 0 contribution)
    // ----------------------------
    $display("\n[CASE3] pix_in_tile_valid holes");
    compute_expected(0, THR, 1, 1, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 1, 1, 1'b1);
    wait_out_accept(0, 200);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case4: MODE1 max(|dx|,|dy|)
    // ----------------------------
    $display("\n[CASE4] MODE1 max(|dx|,|dy|)");
    compute_expected(1, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(1, 1, 0, 1'b1);
    wait_out_accept(1, 200);
    check_outputs(1, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case5: MODE2 threshold-count (sum==hits)
    // ----------------------------
    $display("\n[CASE5] MODE2 threshold-count");
    compute_expected(2, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(2, 1, 0, 1'b1);
    wait_out_accept(2, 200);
    check_outputs(2, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case6: MODE3 squared-sum
    // ----------------------------
    $display("\n[CASE6] MODE3 squared-sum");
    compute_expected(3, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(3, 1, 0, 1'b1);
    wait_out_accept(3, 200);
    check_outputs(3, exp_sum, exp_max, exp_cnt);

    // ----------------------------
    // Case7: two tiles back-to-back (no contamination)
    // ----------------------------
    $display("\n[CASE7] two tiles back-to-back");
    compute_expected(0, THR, 1, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 1, 0, 1'b1);
    wait_out_accept(0, 200);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    compute_expected(0, THR, 0, 0, exp_sum, exp_max, exp_cnt);
    drive_tile(0, 0, 0, 1'b1);
    wait_out_accept(0, 200);
    check_outputs(0, exp_sum, exp_max, exp_cnt);

    $display("\nALL CASES PASS ✅");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire

