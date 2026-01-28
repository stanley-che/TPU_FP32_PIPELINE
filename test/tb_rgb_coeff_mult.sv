// tb_rgb_coeff_mult.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_coeff_mult.vvp \
  ./test/tb_rgb_coeff_mult.sv

vvp ./vvp/tb_rgb_coeff_mult.vvp
gtkwave ./vvp/tb_rgb_coeff_mult.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb_coeff_mult.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_coeff_mult;

  // -----------------------------
  // clock / reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // -----------------------------
  // inputs
  // -----------------------------
  logic       en;        // <<< NEW
  logic       v1_valid;
  logic [7:0] r8, g8, b8;

  // -----------------------------
  // DUT outputs: instantiate matrix
  // -----------------------------
  localparam int L0 = 0;
  localparam int L1 = 1;
  localparam int L2 = 2;
  localparam int L5 = 5;

  // shift-add (USE_DSP=0)
  logic v2_0_s, v2_1_s, v2_2_s, v2_5_s;
  logic [15:0] rt_0_s, gt_0_s, bt_0_s;
  logic [15:0] rt_1_s, gt_1_s, bt_1_s;
  logic [15:0] rt_2_s, gt_2_s, bt_2_s;
  logic [15:0] rt_5_s, gt_5_s, bt_5_s;

  // dsp (USE_DSP=1)
  logic v2_0_d, v2_1_d, v2_2_d, v2_5_d;
  logic [15:0] rt_0_d, gt_0_d, bt_0_d;
  logic [15:0] rt_1_d, gt_1_d, bt_1_d;
  logic [15:0] rt_2_d, gt_2_d, bt_2_d;
  logic [15:0] rt_5_d, gt_5_d, bt_5_d;

  // bypass DUT (debug) pick LAT=1 only
  logic        v2_byp;
  logic [15:0] rt_byp, gt_byp, bt_byp;

  // ---- DUTs (shift-add)
  rgb_coeff_mult #(.LAT(L0), .USE_DSP(1'b0)) dut_l0_s (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_0_s),.r_term(rt_0_s),.g_term(gt_0_s),.b_term(bt_0_s)
  );
  rgb_coeff_mult #(.LAT(L1), .USE_DSP(1'b0)) dut_l1_s (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_1_s),.r_term(rt_1_s),.g_term(gt_1_s),.b_term(bt_1_s)
  );
  rgb_coeff_mult #(.LAT(L2), .USE_DSP(1'b0)) dut_l2_s (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_2_s),.r_term(rt_2_s),.g_term(gt_2_s),.b_term(bt_2_s)
  );
  rgb_coeff_mult #(.LAT(L5), .USE_DSP(1'b0)) dut_l5_s (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_5_s),.r_term(rt_5_s),.g_term(gt_5_s),.b_term(bt_5_s)
  );

  // ---- DUTs (dsp mult)
  rgb_coeff_mult #(.LAT(L0), .USE_DSP(1'b1)) dut_l0_d (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_0_d),.r_term(rt_0_d),.g_term(gt_0_d),.b_term(bt_0_d)
  );
  rgb_coeff_mult #(.LAT(L1), .USE_DSP(1'b1)) dut_l1_d (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_1_d),.r_term(rt_1_d),.g_term(gt_1_d),.b_term(bt_1_d)
  );
  rgb_coeff_mult #(.LAT(L2), .USE_DSP(1'b1)) dut_l2_d (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_2_d),.r_term(rt_2_d),.g_term(gt_2_d),.b_term(bt_2_d)
  );
  rgb_coeff_mult #(.LAT(L5), .USE_DSP(1'b1)) dut_l5_d (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_5_d),.r_term(rt_5_d),.g_term(gt_5_d),.b_term(bt_5_d)
  );

  // ---- BYPASS DUT (LAT=1, USE_DSP irrelevant)
  rgb_coeff_mult #(.LAT(L1), .USE_DSP(1'b0), .BYPASS(1'b1)) dut_bypass (
    .clk(clk),.rst(rst),.en(en),
    .v1_valid(v1_valid),.r8(r8),.g8(g8),.b8(b8),
    .v2_valid(v2_byp),.r_term(rt_byp),.g_term(gt_byp),.b_term(bt_byp)
  );

  // -----------------------------
  // expected pipelines per LAT
  // -----------------------------
  integer i;

  // v shift regs
  logic [L0:0] vsh0;
  logic [L1:0] vsh1;
  logic [L2:0] vsh2;
  logic [L5:0] vsh5;

  // terms shift regs (mult)
  logic [15:0] rsh0 [0:L0], gsh0 [0:L0], bsh0 [0:L0];
  logic [15:0] rsh1 [0:L1], gsh1 [0:L1], bsh1 [0:L1];
  logic [15:0] rsh2 [0:L2], gsh2 [0:L2], bsh2 [0:L2];
  logic [15:0] rsh5 [0:L5], gsh5 [0:L5], bsh5 [0:L5];

  // expected for bypass (LAT=1)
  logic [L1:0] vbyp;
  logic [15:0] rbyp [0:L1], gbyp [0:L1], bbyp [0:L1];

  // -----------------------------
  // helpers
  // -----------------------------
  function automatic [31:0] lfsr32_next(input [31:0] s);
    logic fb;
    begin
      fb = s[31] ^ s[21] ^ s[1] ^ s[0];
      lfsr32_next = {s[30:0], fb};
    end
  endfunction

  function automatic [15:0] exp_mul77(input [7:0] x);
    begin exp_mul77 = x * 16'd77; end
  endfunction
  function automatic [15:0] exp_mul150(input [7:0] x);
    begin exp_mul150 = x * 16'd150; end
  endfunction
  function automatic [15:0] exp_mul29(input [7:0] x);
    begin exp_mul29 = x * 16'd29; end
  endfunction

  task automatic clear_expected;
    integer t;
    begin
      vsh0 = '0; vsh1 = '0; vsh2 = '0; vsh5 = '0;
      for (t=0; t<=L0; t=t+1) begin rsh0[t]=16'h0; gsh0[t]=16'h0; bsh0[t]=16'h0; end
      for (t=0; t<=L1; t=t+1) begin rsh1[t]=16'h0; gsh1[t]=16'h0; bsh1[t]=16'h0; end
      for (t=0; t<=L2; t=t+1) begin rsh2[t]=16'h0; gsh2[t]=16'h0; bsh2[t]=16'h0; end
      for (t=0; t<=L5; t=t+1) begin rsh5[t]=16'h0; gsh5[t]=16'h0; bsh5[t]=16'h0; end

      vbyp = '0;
      for (t=0; t<=L1; t=t+1) begin rbyp[t]=16'h0; gbyp[t]=16'h0; bbyp[t]=16'h0; end
    end
  endtask

  task automatic shift_expected(
    input logic vin,
    input logic [15:0] r0,
    input logic [15:0] g0,
    input logic [15:0] b0
  );
    integer t;
    begin
      // LAT0
      vsh0[0]  = vin;
      rsh0[0]  = r0;  gsh0[0] = g0;  bsh0[0] = b0;

      // LAT1
      for (t=L1; t>=1; t=t-1) begin
        vsh1[t] = vsh1[t-1];
        rsh1[t] = rsh1[t-1];
        gsh1[t] = gsh1[t-1];
        bsh1[t] = bsh1[t-1];
      end
      vsh1[0] = vin; rsh1[0]=r0; gsh1[0]=g0; bsh1[0]=b0;

      // LAT2
      for (t=L2; t>=1; t=t-1) begin
        vsh2[t] = vsh2[t-1];
        rsh2[t] = rsh2[t-1];
        gsh2[t] = gsh2[t-1];
        bsh2[t] = bsh2[t-1];
      end
      vsh2[0] = vin; rsh2[0]=r0; gsh2[0]=g0; bsh2[0]=b0;

      // LAT5
      for (t=L5; t>=1; t=t-1) begin
        vsh5[t] = vsh5[t-1];
        rsh5[t] = rsh5[t-1];
        gsh5[t] = gsh5[t-1];
        bsh5[t] = bsh5[t-1];
      end
      vsh5[0] = vin; rsh5[0]=r0; gsh5[0]=g0; bsh5[0]=b0;
    end
  endtask

  task automatic shift_expected_bypass_lat1(
    input logic vin,
    input logic [15:0] rb0,
    input logic [15:0] gb0,
    input logic [15:0] bb0
  );
    begin
      // LAT=1 only
      vbyp[1] = vbyp[0];
      rbyp[1] = rbyp[0];
      gbyp[1] = gbyp[0];
      bbyp[1] = bbyp[0];

      vbyp[0] = vin;
      rbyp[0] = rb0;
      gbyp[0] = gb0;
      bbyp[0] = bb0;
    end
  endtask

  task automatic check_one(
    input int idx,
    input logic v_act,
    input logic [15:0] r_act,
    input logic [15:0] g_act,
    input logic [15:0] b_act,
    input logic v_exp,
    input logic [15:0] r_exp,
    input logic [15:0] g_exp,
    input logic [15:0] b_exp,
    input [127:0] tag
  );
    begin
      if (v_act !== v_exp) begin
        $display("FAIL %s valid idx=%0d got=%b exp=%b", tag, idx, v_act, v_exp);
        $fatal;
      end
      if (v_act === 1'b1) begin
        if (r_act !== r_exp || g_act !== g_exp || b_act !== b_exp) begin
          $display("FAIL %s data idx=%0d got r=%0d g=%0d b=%0d exp r=%0d g=%0d b=%0d",
                   tag, idx, r_act, g_act, b_act, r_exp, g_exp, b_exp);
          $fatal;
        end
      end
    end
  endtask

  task automatic step_drive_update_check(
    input int idx,
    input logic vin,
    input logic [7:0] r_in,
    input logic [7:0] g_in,
    input logic [7:0] b_in,
    input bit drive_x_on_invalid
  );
    logic [15:0] r0, g0, b0;
    logic [15:0] rb0, gb0, bb0; // bypass stage0 expected
    begin
      // drive inputs
      v1_valid = vin;
      if (!vin && drive_x_on_invalid) begin
        r8 = 8'hxx; g8 = 8'hxx; b8 = 8'hxx;
      end else begin
        r8 = r_in;  g8 = g_in;  b8 = b_in;
      end

      // expected stage0 (computed from numeric inputs when vin=1)
      if (vin) begin
        r0  = exp_mul77(r_in);
        g0  = exp_mul150(g_in);
        b0  = exp_mul29(b_in);

        rb0 = {8'h00, r_in};
        gb0 = {8'h00, g_in};
        bb0 = {8'h00, b_in};
      end else begin
        r0  = 16'h0; g0  = 16'h0; b0  = 16'h0;
        rb0 = 16'h0; gb0 = 16'h0; bb0 = 16'h0;
      end

      @(posedge clk);
      #1;

      // update expected pipelines
      if (rst) begin
          clear_expected();
      end else begin
          // -----------------------------
          // LAT0 expected MUST track input every cycle (combinational)
          // -----------------------------
          vsh0[0] = vin;
          rsh0[0] = r0;
          gsh0[0] = g0;
          bsh0[0] = b0;

          // -----------------------------
          // LAT>=1 pipelines advance only when en=1
          // -----------------------------
          if (en) begin
              // shift_expected() 目前也會寫 vsh0/rsh0/...，但我們上面已經覆蓋了
              // 你可以保留 shift_expected()，但建議把它改成「只處理 LAT>=1」更乾淨
              shift_expected(vin, r0, g0, b0);
              shift_expected_bypass_lat1(vin, rb0, gb0, bb0);
          end
      end

      // en==0: hold expected (do nothing)

      // check shift-add vs dsp should match expected exactly (when valid)
      check_one(idx, v2_0_s, rt_0_s, gt_0_s, bt_0_s, vsh0[L0], rsh0[L0], gsh0[L0], bsh0[L0], "L0_SHIFT");
      check_one(idx, v2_1_s, rt_1_s, gt_1_s, bt_1_s, vsh1[L1], rsh1[L1], gsh1[L1], bsh1[L1], "L1_SHIFT");
      check_one(idx, v2_2_s, rt_2_s, gt_2_s, bt_2_s, vsh2[L2], rsh2[L2], gsh2[L2], bsh2[L2], "L2_SHIFT");
      check_one(idx, v2_5_s, rt_5_s, gt_5_s, bt_5_s, vsh5[L5], rsh5[L5], gsh5[L5], bsh5[L5], "L5_SHIFT");

      check_one(idx, v2_0_d, rt_0_d, gt_0_d, bt_0_d, vsh0[L0], rsh0[L0], gsh0[L0], bsh0[L0], "L0_DSP  ");
      check_one(idx, v2_1_d, rt_1_d, gt_1_d, bt_1_d, vsh1[L1], rsh1[L1], gsh1[L1], bsh1[L1], "L1_DSP  ");
      check_one(idx, v2_2_d, rt_2_d, gt_2_d, bt_2_d, vsh2[L2], rsh2[L2], gsh2[L2], bsh2[L2], "L2_DSP  ");
      check_one(idx, v2_5_d, rt_5_d, gt_5_d, bt_5_d, vsh5[L5], rsh5[L5], gsh5[L5], bsh5[L5], "L5_DSP  ");

      // check bypass DUT (LAT=1)
      check_one(idx, v2_byp, rt_byp, gt_byp, bt_byp,
                vbyp[L1], rbyp[L1], gbyp[L1], bbyp[L1],
                "BYPASS ");
    end
  endtask

  task automatic warmup(input int n);
    integer t;
    begin
      for (t = 0; t < n; t = t + 1)
        step_drive_update_check(-1000+t, 1'b0, 8'h0, 8'h0, 8'h0, 1'b0);
    end
  endtask

  // -----------------------------
  // tests
  // -----------------------------
  task automatic test_ramp;
    integer t;
    logic [7:0] rr, gg, bb;
    begin
      $display("[RUN ] RAMP");
      warmup(8);
      for (t = 0; t < 64; t = t + 1) begin
        rr = t[7:0];
        gg = (t * 3);
        bb = 8'hFF - t[7:0];
        step_drive_update_check(10+t, 1'b1, rr, gg, bb, 1'b0);
      end
      $display("[PASS] RAMP");
    end
  endtask

  task automatic test_boundary;
    integer t;
    logic [7:0] vals [0:7];
    begin
      $display("[RUN ] BOUNDARY");
      vals[0]=8'h00; vals[1]=8'h01; vals[2]=8'h02; vals[3]=8'h03;
      vals[4]=8'h7F; vals[5]=8'h80; vals[6]=8'hFE; vals[7]=8'hFF;

      warmup(8);
      for (t = 0; t < 8; t = t + 1) begin
        step_drive_update_check(200+t, 1'b1, vals[t], 8'h55, 8'hAA, 1'b0);
        step_drive_update_check(220+t, 1'b1, 8'h55, vals[t], 8'hAA, 1'b0);
        step_drive_update_check(240+t, 1'b1, 8'h55, 8'hAA, vals[t], 1'b0);
      end
      $display("[PASS] BOUNDARY");
    end
  endtask

  task automatic test_random_bubbles;
    integer t;
    logic [31:0] s;
    begin
      $display("[RUN ] RANDOM_BUBBLES");
      warmup(8);
      s = 32'h1ACE_B00C;
      for (t = 0; t < 400; t = t + 1) begin
        s = lfsr32_next(s);
        if (s[24]) begin
          step_drive_update_check(500+t, 1'b0, 8'h0, 8'h0, 8'h0, 1'b1);
        end else begin
          step_drive_update_check(500+t, 1'b1, s[7:0], s[15:8], s[23:16], 1'b0);
        end
      end
      $display("[PASS] RANDOM_BUBBLES");
    end
  endtask

  task automatic test_midstream_reset;
    integer t;
    logic [7:0] rr, gg, bb;
    begin
      $display("[RUN ] MIDSTREAM_RESET");
      warmup(8);

      for (t = 0; t < 30; t = t + 1) begin
        rr = t[7:0];
        gg = (t * 5);
        bb = (t * 7);
        step_drive_update_check(1000+t, 1'b1, rr, gg, bb, 1'b0);
      end

      rst = 1'b1;
      step_drive_update_check(1100, 1'b0, 8'h0, 8'h0, 8'h0, 1'b0);
      step_drive_update_check(1101, 1'b0, 8'h0, 8'h0, 8'h0, 1'b0);
      rst = 1'b0;

      warmup(8);
      for (t = 0; t < 30; t = t + 1) begin
        rr = (t + 8);
        gg = (t * 3);
        bb = 8'hFF - t[7:0];
        step_drive_update_check(1200+t, 1'b1, rr, gg, bb, 1'b0);
      end

      $display("[PASS] MIDSTREAM_RESET");
    end
  endtask

  // NEW: en=0 should HOLD pipeline (no shift, outputs constant)
    // NEW: en=0 should HOLD pipeline (only meaningful for LAT>=1)
  task automatic test_en_hold;
    integer t;
    logic [7:0] rr, gg, bb;

    // snapshot outputs (shift-add path) for LAT>=1 only
    logic v1_s, v2_s, v5_s;
    logic [15:0] r1_s, g1_s, b1_s;
    logic [15:0] r2_s, g2_s, b2_s;
    logic [15:0] r5_s, g5_s, b5_s;

    // snapshot bypass (LAT=1)
    logic vb_s;
    logic [15:0] rb_s, gb_s, bb_s;

    begin
      $display("[RUN ] EN_HOLD");
      warmup(8);

      // load some non-zero state into pipelines
      for (t=0; t<8; t=t+1) begin
        rr = t[7:0];
        gg = (t * 3);
        bb = (t * 5);
        step_drive_update_check(1500+t, 1'b1, rr, gg, bb, 1'b0);
      end

      // snapshot outputs (LAT>=1 only)
      v1_s = v2_1_s; r1_s = rt_1_s; g1_s = gt_1_s; b1_s = bt_1_s;
      v2_s = v2_2_s; r2_s = rt_2_s; g2_s = gt_2_s; b2_s = bt_2_s;
      v5_s = v2_5_s; r5_s = rt_5_s; g5_s = gt_5_s; b5_s = bt_5_s;

      // snapshot bypass (LAT=1)
      vb_s = v2_byp; rb_s = rt_byp; gb_s = gt_byp; bb_s = bt_byp;

      // hold 10 cycles, inputs toggling, outputs must not change
      en = 1'b0;
      for (t=0; t<10; t=t+1) begin
        rr = (t + 8);
        gg = (t * 7);
        bb = (8'hFF - t[7:0]);

        step_drive_update_check(1600+t, 1'b1, rr, gg, bb, 1'b0);

        // check hold for LAT>=1
        if (v2_1_s !== v1_s || rt_1_s !== r1_s || gt_1_s !== g1_s || bt_1_s !== b1_s) begin
          $display("FAIL EN_HOLD L1");
          $fatal;
        end
        if (v2_2_s !== v2_s || rt_2_s !== r2_s || gt_2_s !== g2_s || bt_2_s !== b2_s) begin
          $display("FAIL EN_HOLD L2");
          $fatal;
        end
        if (v2_5_s !== v5_s || rt_5_s !== r5_s || gt_5_s !== g5_s || bt_5_s !== b5_s) begin
          $display("FAIL EN_HOLD L5");
          $fatal;
        end

        // check bypass also holds (LAT=1)
        if (v2_byp !== vb_s || rt_byp !== rb_s || gt_byp !== gb_s || bt_byp !== bb_s) begin
          $display("FAIL EN_HOLD BYPASS");
          $fatal;
        end
      end

      // resume
      en = 1'b1;
      warmup(4);

      $display("[PASS] EN_HOLD");
    end
  endtask


  // NEW: BYPASS DUT already checked every cycle by step_drive_update_check()
  // This testcase just exercises patterns + bubbles to cover it.
  task automatic test_bypass;
    integer t;
    logic [7:0] rr, gg, bb;
    begin
      $display("[RUN ] BYPASS");
      warmup(8);

      for (t=0; t<80; t=t+1) begin
        rr = (t*9);
        gg = (t*5);
        bb = (t*3);

        if ((t % 7) == 3) begin
          step_drive_update_check(2000+t, 1'b0, 8'h0, 8'h0, 8'h0, 1'b1);
        end else begin
          step_drive_update_check(2000+t, 1'b1, rr, gg, bb, 1'b0);
        end
      end

      $display("[PASS] BYPASS");
    end
  endtask

  // -----------------------------
  // main
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_rgb_coeff_mult.vcd");
    $dumpvars(0, tb_rgb_coeff_mult);

    // init
    rst = 1'b1;
    en  = 1'b1;
    v1_valid = 1'b0;
    r8 = 8'h0; g8 = 8'h0; b8 = 8'h0;
    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_ramp();
    test_boundary();
    test_random_bubbles();
    test_midstream_reset();
    test_en_hold();
    test_bypass();

    $display("PASS tb_rgb_coeff_mult ALL");
    $finish;
  end

endmodule

`default_nettype wire
