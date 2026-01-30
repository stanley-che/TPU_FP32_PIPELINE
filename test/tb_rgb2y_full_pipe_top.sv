// tb_rgb2y_full_pipe_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb2y_full_pipe_top.vvp \
  ./test/tb_rgb2y_full_pipe_top.sv

vvp ./vvp/tb_rgb2y_full_pipe_top.vvp
gtkwave ./vvp/tb_rgb2y_full_pipe_top.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb2y_full_pipe_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb2y_full_pipe_top;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  reg clk = 1'b0;
  always #5 clk = ~clk;
  reg rst;

  // ------------------------------------------------------------
  // DUT I/O
  // ------------------------------------------------------------
  reg         en;
  reg         pix_valid;
  reg [35:0]  rgb_in;

  reg         out_ready;
  wire        in_ready;

  wire        v1_valid;
  wire [7:0]  r8, g8, b8;

  wire        v2_valid;
  wire [15:0] r_term, g_term, b_term;

  wire        v4_valid;
  wire [16:0] sum17;
  wire [9:0]  y10;
  wire [7:0]  y8;
  wire        ovf_sum;
  wire        ovf_y;

  // ------------------------------------------------------------
  // params (match DUT)
  // ------------------------------------------------------------
  localparam integer FORMAT = 0; // 0 RGB888, 1 RGB101010, 2 RGB121212, 3 RGB565

  localparam integer LAT_U  = 1;
  localparam integer LAT_M  = 2;

  localparam ZERO_WHEN_INVALID = 1'b1;
  localparam USE_DSP = 1'b0;
  localparam BYPASS  = 1'b0;

  localparam integer LAT_SUM = 2;
  localparam integer LAT_RND = 2;
  localparam HOLD_VALID_WHEN_STALL = 1'b1;

  localparam integer ROUND_MODE = 1; // r>=0x80 round-up
  localparam integer Y8_MODE    = 0; // y8 = y10[9:2]

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  rgb2y_full_pipe_top #(
    .FORMAT(FORMAT),
    .LAT_U(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),

    .LAT_M(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS),

    .LAT_SUM(LAT_SUM),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),
    .LAT_RND(LAT_RND),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .rgb_in(rgb_in),

    .out_ready(out_ready),
    .in_ready(in_ready),

    .v4_valid(v4_valid),
    .sum17(sum17),
    .y10(y10),
    .y8(y8),
    .ovf_sum(ovf_sum),
    .ovf_y(ovf_y),

    .v1_valid(v1_valid),
    .r8(r8), .g8(g8), .b8(b8),

    .v2_valid(v2_valid),
    .r_term(r_term), .g_term(g_term), .b_term(b_term)
  );

  // ============================================================
  // helpers: pack by FORMAT
  // ============================================================
  task automatic pack_rgb888(
    input  [7:0] rr, input [7:0] gg, input [7:0] bb,
    output [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[23:16] = rr;
      bus[15:8]  = gg;
      bus[7:0]   = bb;
    end
  endtask

  task automatic pack_rgb565(
    input [4:0] r5, input [5:0] g6, input [4:0] b5,
    output [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[15:11] = r5;
      bus[10:5]  = g6;
      bus[4:0]   = b5;
    end
  endtask

  task automatic pack_rgb101010(
    input [9:0] r10, input [9:0] g10, input [9:0] b10,
    output [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[29:20] = r10;
      bus[19:10] = g10;
      bus[9:0]   = b10;
    end
  endtask

  task automatic pack_rgb121212(
    input [11:0] r12, input [11:0] g12, input [11:0] b12,
    output [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[35:24] = r12;
      bus[23:12] = g12;
      bus[11:0]  = b12;
    end
  endtask

  task automatic pack_by_format(
    input  [7:0] rr, input [7:0] gg, input [7:0] bb,
    output [35:0] bus
  );
    reg [4:0]  r5, b5;
    reg [5:0]  g6;
    reg [9:0]  r10, g10, b10;
    reg [11:0] r12, g12, b12;
    begin
      if (FORMAT == 0) begin
        pack_rgb888(rr, gg, bb, bus);
      end else if (FORMAT == 1) begin
        r10 = {rr, rr[7:6]};
        g10 = {gg, gg[7:6]};
        b10 = {bb, bb[7:6]};
        pack_rgb101010(r10, g10, b10, bus);
      end else if (FORMAT == 2) begin
        r12 = {rr, rr[7:4]};
        g12 = {gg, gg[7:4]};
        b12 = {bb, bb[7:4]};
        pack_rgb121212(r12, g12, b12, bus);
      end else begin
        r5 = rr[7:3];
        g6 = gg[7:2];
        b5 = bb[7:3];
        pack_rgb565(r5, g6, b5, bus);
      end
    end
  endtask

  // ============================================================
  // expected model (same as your blocks combined)
  // ============================================================
  function automatic [31:0] lfsr32_next(input [31:0] s);
    reg fb;
    begin
      fb = s[31] ^ s[21] ^ s[1] ^ s[0];
      lfsr32_next = {s[30:0], fb};
    end
  endfunction

  function automatic [7:0] exp5_to_8(input [4:0] x);
    begin exp5_to_8 = {x, x[4:2]}; end
  endfunction
  function automatic [7:0] exp6_to_8(input [5:0] x);
    begin exp6_to_8 = {x, x[5:4]}; end
  endfunction
  function automatic [7:0] msb10_to_8(input [9:0] x);
    begin msb10_to_8 = x[9:2]; end
  endfunction
  function automatic [7:0] msb12_to_8(input [11:0] x);
    begin msb12_to_8 = x[11:4]; end
  endfunction

  // coeffs
  function automatic [15:0] mul77(input [7:0] x);
    begin mul77 = x * 16'd77; end
  endfunction
  function automatic [15:0] mul150(input [7:0] x);
    begin mul150 = x * 16'd150; end
  endfunction
  function automatic [15:0] mul29(input [7:0] x);
    begin mul29 = x * 16'd29; end
  endfunction

  // sum/round
  localparam integer SUM_MAX = 65280;

  function automatic [16:0] f_sum17(input [15:0] r, input [15:0] g, input [15:0] b);
    reg [16:0] s;
    begin
      s = {1'b0,r} + {1'b0,g} + {1'b0,b};
      f_sum17 = s;
    end
  endfunction

  function automatic f_ovf_sum(input [16:0] s);
    begin
      f_ovf_sum = (s > SUM_MAX);
    end
  endfunction

  function automatic [9:0] f_y10_round(input [16:0] s);
    reg [9:0] q;
    reg [7:0] r;
    reg inc;
    reg [10:0] qinc;
    begin
      q = s[16:8];
      r = s[7:0];
      // ROUND_MODE=1 => inc when r>=0x80
      inc  = (r >= 8'h80);
      qinc = {1'b0,q} + {10'd0,inc};
      if (qinc[10]) f_y10_round = 10'h3FF;
      else         f_y10_round = qinc[9:0];
    end
  endfunction

  function automatic [7:0] f_y8_from_y10(input [9:0] y10_in);
    begin
      if (Y8_MODE == 0) begin
        f_y8_from_y10 = y10_in[9:2];
      end else begin
        if (y10_in > 10'd255) f_y8_from_y10 = 8'hFF;
        else                  f_y8_from_y10 = y10_in[7:0];
      end
    end
  endfunction

  function automatic f_ovf_y_from(input [9:0] y10_in);
    begin
      if (Y8_MODE == 1 && (y10_in > 10'd255)) f_ovf_y_from = 1'b1;
      else                                    f_ovf_y_from = 1'b0;
    end
  endfunction

  // stage0 expected unpack from bus_in (NOT X)
  task automatic unpack_expected0(
    input         v_in,
    input  [35:0] bus_in,
    output        v0,
    output [7:0]  r0,
    output [7:0]  g0,
    output [7:0]  b0
  );
    reg [7:0] rc, gc, bc;
    begin
      rc = 8'h00; gc = 8'h00; bc = 8'h00;

      if (FORMAT == 0) begin
        rc = bus_in[23:16];
        gc = bus_in[15:8];
        bc = bus_in[7:0];
      end else if (FORMAT == 1) begin
        rc = msb10_to_8(bus_in[29:20]);
        gc = msb10_to_8(bus_in[19:10]);
        bc = msb10_to_8(bus_in[9:0]);
      end else if (FORMAT == 2) begin
        rc = msb12_to_8(bus_in[35:24]);
        gc = msb12_to_8(bus_in[23:12]);
        bc = msb12_to_8(bus_in[11:0]);
      end else begin
        rc = exp5_to_8(bus_in[15:11]);
        gc = exp6_to_8(bus_in[10:5]);
        bc = exp5_to_8(bus_in[4:0]);
      end

      v0 = v_in;

      if (ZERO_WHEN_INVALID && !v_in) begin
        r0 = 8'h00; g0 = 8'h00; b0 = 8'h00;
      end else begin
        r0 = rc;    g0 = gc;    b0 = bc;
      end
    end
  endtask

  // ============================================================
  // expected pipeline state (ADVANCES ONLY WHEN adv=in_ready)
  // ============================================================
  localparam integer RND_DEPTH = (LAT_RND==0) ? 1 : LAT_RND;
  localparam integer TOT = LAT_U + LAT_M + LAT_SUM + RND_DEPTH;

  reg [TOT:0] exp_v4; // final valid shift
  reg [7:0]   exp_r8 [0:TOT];
  reg [7:0]   exp_g8 [0:TOT];
  reg [7:0]   exp_b8 [0:TOT];

  reg [15:0]  exp_rt [0:TOT];
  reg [15:0]  exp_gt [0:TOT];
  reg [15:0]  exp_bt [0:TOT];

  reg [16:0]  exp_sum [0:TOT];
  reg [9:0]   exp_y10 [0:TOT];
  reg [7:0]   exp_y8  [0:TOT];
  reg         exp_os  [0:TOT];
  reg         exp_oy  [0:TOT];

  integer i;

  task automatic clear_expected;
    integer k;
    begin
      exp_v4 = { (TOT+1){1'b0} };
      for (k=0; k<=TOT; k=k+1) begin
        exp_r8[k]  = 8'h00; exp_g8[k]  = 8'h00; exp_b8[k]  = 8'h00;
        exp_rt[k]  = 16'h0; exp_gt[k]  = 16'h0; exp_bt[k]  = 16'h0;
        exp_sum[k] = 17'h0; exp_y10[k] = 10'h0; exp_y8[k]  = 8'h00;
        exp_os[k]  = 1'b0;  exp_oy[k]  = 1'b0;
      end
    end
  endtask

  // ============================================================
  // one step: drive -> posedge -> update expected if adv -> check
  // ============================================================
  task automatic step_drive_update_check(
    input integer idx,
    input         want_valid,
    input  [35:0] bus_in,
    input         inject_x_when_invalid,
    input         en_in,
    input         out_ready_in
  );
    reg do_send;
    reg adv;

    // stage0 expected unpack
    reg u0v;
    reg [7:0] u0r, u0g, u0b;

    // stage1 tail (after LAT_U) pre-advance snapshot
    reg [7:0] r8_pre, g8_pre, b8_pre;
    reg       v1_pre;

    // stage2 coeff pre (from stage1 tail pre)
    reg [15:0] rt_pre, gt_pre, bt_pre;
    reg        v2_pre;

    // stage3 sum/round pre
    reg [16:0] sum_pre;
    reg [9:0]  y10_pre;
    reg [7:0]  y8_pre;
    reg        os_pre, oy_pre;

    integer k;
    begin
      en        = en_in;
      out_ready = out_ready_in;

      // DUT's in_ready already includes en && can_accept; we mimic upstream rule:
      do_send = (want_valid && in_ready);

      pix_valid = do_send;

      if (!do_send && inject_x_when_invalid) rgb_in = 36'hx;
      else                                  rgb_in = bus_in;

      // expected stage0 uses bus_in (not X)
      unpack_expected0(do_send, bus_in, u0v, u0r, u0g, u0b);

      // adv condition matches DUT: whole pipe advances on in_ready
      adv = in_ready;

      // pre-snapshot of stage1 tail (LAT_U) from expected arrays at proper delay point:
      // Because EVERYTHING advances only on adv, we can treat the expected arrays as a single "time axis".
      // We'll build the per-stage expected by inserting into [0] and shifting on adv, then compare taps at fixed offsets.
      // However for computing coeff/sum at insertion time, we only need u0r/u0g/u0b at this cycle.
      //
      // We'll compute full-chain expected at insertion (position 0) based on current input.
      //
      if (do_send) begin
        // unpack at insertion
        r8_pre = u0r; g8_pre = u0g; b8_pre = u0b;
        v1_pre = 1'b1;

        // coeff at insertion
        if (BYPASS) begin
          rt_pre = {8'h00, r8_pre};
          gt_pre = {8'h00, g8_pre};
          bt_pre = {8'h00, b8_pre};
        end else begin
          rt_pre = mul77(r8_pre);
          gt_pre = mul150(g8_pre);
          bt_pre = mul29(b8_pre);
        end
        v2_pre = 1'b1;

        // sum/round at insertion
        sum_pre = f_sum17(rt_pre, gt_pre, bt_pre);
        os_pre  = f_ovf_sum(sum_pre);
        y10_pre = f_y10_round(sum_pre);
        y8_pre  = f_y8_from_y10(y10_pre);
        oy_pre  = f_ovf_y_from(y10_pre);
      end else begin
        r8_pre = 8'h00; g8_pre = 8'h00; b8_pre = 8'h00; v1_pre = 1'b0;
        rt_pre = 16'h0; gt_pre = 16'h0; bt_pre = 16'h0; v2_pre = 1'b0;
        sum_pre = 17'h0; os_pre = 1'b0; y10_pre = 10'h0; y8_pre = 8'h00; oy_pre = 1'b0;
      end

      @(posedge clk);
      #1;

      if (rst) begin
        clear_expected();
      end else if (adv) begin
        // shift whole expected timeline
        for (k=TOT; k>=1; k=k-1) begin
          exp_v4[k]  = exp_v4[k-1];

          exp_r8[k]  = exp_r8[k-1];
          exp_g8[k]  = exp_g8[k-1];
          exp_b8[k]  = exp_b8[k-1];

          exp_rt[k]  = exp_rt[k-1];
          exp_gt[k]  = exp_gt[k-1];
          exp_bt[k]  = exp_bt[k-1];

          exp_sum[k] = exp_sum[k-1];
          exp_y10[k] = exp_y10[k-1];
          exp_y8[k]  = exp_y8[k-1];
          exp_os[k]  = exp_os[k-1];
          exp_oy[k]  = exp_oy[k-1];
        end

        // insert new token at time 0
        exp_v4[0]  = do_send;

        exp_r8[0]  = r8_pre;
        exp_g8[0]  = g8_pre;
        exp_b8[0]  = b8_pre;

        exp_rt[0]  = rt_pre;
        exp_gt[0]  = gt_pre;
        exp_bt[0]  = bt_pre;

        exp_sum[0] = sum_pre;
        exp_y10[0] = y10_pre;
        exp_y8[0]  = y8_pre;
        exp_os[0]  = os_pre;
        exp_oy[0]  = oy_pre;
      end

      // ----------------------------------------------------------
      // check taps at their offsets:
      // v1: after LAT_U
      // v2: after LAT_U + LAT_M
      // v4: after LAT_U + LAT_M + LAT_SUM + RND_DEPTH
      // ----------------------------------------------------------
      if (v1_valid !== exp_v4[LAT_U]) begin
        $display("FAIL v1_valid idx=%0d got=%b exp=%b (in_ready=%b en=%b out_ready=%b)",
                 idx, v1_valid, exp_v4[LAT_U], in_ready, en, out_ready);
        $finish;
      end
      if (v1_valid) begin
        if (r8 !== exp_r8[LAT_U] || g8 !== exp_g8[LAT_U] || b8 !== exp_b8[LAT_U]) begin
          $display("FAIL unpack idx=%0d got R=%h G=%h B=%h exp R=%h G=%h B=%h",
                   idx, r8, g8, b8, exp_r8[LAT_U], exp_g8[LAT_U], exp_b8[LAT_U]);
          $finish;
        end
      end

      if (v2_valid !== exp_v4[LAT_U + LAT_M]) begin
        $display("FAIL v2_valid idx=%0d got=%b exp=%b", idx, v2_valid, exp_v4[LAT_U + LAT_M]);
        $finish;
      end
      if (v2_valid) begin
        if (r_term !== exp_rt[LAT_U + LAT_M] || g_term !== exp_gt[LAT_U + LAT_M] || b_term !== exp_bt[LAT_U + LAT_M]) begin
          $display("FAIL coeff idx=%0d got r=%0d g=%0d b=%0d exp r=%0d g=%0d b=%0d",
                   idx, r_term, g_term, b_term,
                   exp_rt[LAT_U + LAT_M], exp_gt[LAT_U + LAT_M], exp_bt[LAT_U + LAT_M]);
          $finish;
        end
      end

      if (v4_valid !== exp_v4[TOT]) begin
        $display("FAIL v4_valid idx=%0d got=%b exp=%b", idx, v4_valid, exp_v4[TOT]);
        $finish;
      end
      if (sum17 !== exp_sum[TOT]) begin
        $display("FAIL sum17 idx=%0d got=%0d exp=%0d", idx, sum17, exp_sum[TOT]);
        $finish;
      end
      if (ovf_sum !== exp_os[TOT]) begin
        $display("FAIL ovf_sum idx=%0d got=%b exp=%b", idx, ovf_sum, exp_os[TOT]);
        $finish;
      end
      if (y10 !== exp_y10[TOT]) begin
        $display("FAIL y10 idx=%0d got=%0d exp=%0d", idx, y10, exp_y10[TOT]);
        $finish;
      end
      if (y8 !== exp_y8[TOT]) begin
        $display("FAIL y8 idx=%0d got=%0d exp=%0d", idx, y8, exp_y8[TOT]);
        $finish;
      end
      if (ovf_y !== exp_oy[TOT]) begin
        $display("FAIL ovf_y idx=%0d got=%b exp=%b", idx, ovf_y, exp_oy[TOT]);
        $finish;
      end
    end
  endtask

  task automatic warmup(input integer n);
    integer k;
    begin
      for (k=0; k<n; k=k+1)
        step_drive_update_check(-1000+k, 1'b0, 36'h0, 1'b0, 1'b1, 1'b1);
    end
  endtask

  task automatic drain(input integer n);
    integer k;
    begin
      for (k=0; k<n; k=k+1)
        step_drive_update_check(900000+k, 1'b0, 36'h0, 1'b0, 1'b1, 1'b1);
    end
  endtask

  // ============================================================
  // tests
  // ============================================================
  task automatic test_mix;
    integer t;
    reg [31:0] s;
    reg [7:0] rr, gg, bb;
    reg [35:0] bus;
    reg en_pat;
    reg rdy_pat;
    begin
      $display("[RUN ] E2E_MIX (FORMAT=%0d LAT_U=%0d LAT_M=%0d LAT_SUM=%0d LAT_RND=%0d)",
               FORMAT, LAT_U, LAT_M, LAT_SUM, LAT_RND);

      warmup(10);
      s = 32'h1ACE_B00C;

      for (t=0; t<1200; t=t+1) begin
        s  = lfsr32_next(s);
        rr = s[7:0];
        gg = s[15:8];
        bb = s[23:16];
        pack_by_format(rr, gg, bb, bus);

        // en: long/short holds
        if ((t%17)==8 || (t%17)==9 || (t%17)==10) en_pat = 1'b0;
        else if ((t%9)==4)                         en_pat = 1'b0;
        else                                        en_pat = 1'b1;

        // out_ready: backpressure bursts
        if ((t%23)>=16 && (t%23)<=20) rdy_pat = 1'b0;
        else                          rdy_pat = 1'b1;

        // want_valid bubbles ~30%
        if (s[31] & s[28]) begin
          step_drive_update_check(100+t, 1'b0, 36'h0, 1'b1, en_pat, rdy_pat); // invalid + X
        end else begin
          step_drive_update_check(100+t, 1'b1, bus, 1'b0, en_pat, rdy_pat);
        end
      end

      drain(TOT+10);
      $display("[PASS] E2E_MIX");
    end
  endtask

  task automatic test_known_vectors;
  reg [35:0] bus;
  begin
    $display("[RUN ] KNOWN_VECTORS");
    warmup(8);

    pack_by_format(8'hFF,8'h00,8'h00,bus);
    step_drive_update_check(30000, 1'b1, bus, 1'b0, 1'b1, 1'b1);

    pack_by_format(8'h00,8'hFF,8'h00,bus);
    step_drive_update_check(30001, 1'b1, bus, 1'b0, 1'b1, 1'b1);

    pack_by_format(8'h00,8'h00,8'hFF,bus);
    step_drive_update_check(30002, 1'b1, bus, 1'b0, 1'b1, 1'b1);

    pack_by_format(8'hFF,8'hFF,8'hFF,bus);
    step_drive_update_check(30003, 1'b1, bus, 1'b0, 1'b1, 1'b1);

    drain(TOT+10);
    $display("[PASS] KNOWN_VECTORS");
  end
  endtask


  // small helper returning bus (iverilog-safe: function with tasks not allowed, so use function with FORMAT=0 only? We'll avoid.)
  // We'll just do explicit pack_by_format into a local var where needed.

  // ============================================================
  // main
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_rgb2y_full_pipe_top.vcd");
    $dumpvars(0, tb_rgb2y_full_pipe_top);

    rst = 1'b1;
    en  = 1'b1;
    pix_valid = 1'b0;
    rgb_in    = 36'h0;
    out_ready = 1'b1;

    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_mix();

    // known vectors (explicit packing)
    begin
      reg [35:0] bus;
      warmup(8);

      pack_by_format(8'hFF,8'h00,8'h00,bus); step_drive_update_check(30000, 1'b1, bus, 1'b0, 1'b1, 1'b1);
      pack_by_format(8'h00,8'hFF,8'h00,bus); step_drive_update_check(30001, 1'b1, bus, 1'b0, 1'b1, 1'b1);
      pack_by_format(8'h00,8'h00,8'hFF,bus); step_drive_update_check(30002, 1'b1, bus, 1'b0, 1'b1, 1'b1);
      pack_by_format(8'hFF,8'hFF,8'hFF,bus); step_drive_update_check(30003, 1'b1, bus, 1'b0, 1'b1, 1'b1);

      drain(TOT+10);
      $display("[PASS] KNOWN_VECTORS (inline)");
    end

    // midstream reset
    begin
      integer t;
      reg [35:0] bus;
      reg [7:0] rr, gg, bb;
      $display("[RUN ] MIDSTREAM_RESET");
      warmup(8);

      for (t=0; t<40; t=t+1) begin
        rr = t[7:0];
        gg = (t*3);
        bb = 8'hFF - t[7:0];
        pack_by_format(rr,gg,bb,bus);
        step_drive_update_check(40000+t, 1'b1, bus, 1'b0, 1'b1, 1'b1);
      end

      rst = 1'b1;
      step_drive_update_check(40100, 1'b0, 36'h0, 1'b0, 1'b1, 1'b1);
      step_drive_update_check(40101, 1'b0, 36'h0, 1'b0, 1'b1, 1'b1);
      rst = 1'b0;

      warmup(12);

      for (t=0; t<40; t=t+1) begin
        rr = (t+8);
        gg = (t*5);
        bb = (t*7);
        pack_by_format(rr,gg,bb,bus);
        step_drive_update_check(40200+t, 1'b1, bus, 1'b0, 1'b1, 1'b1);
      end

      drain(TOT+10);
      $display("[PASS] MIDSTREAM_RESET");
    end

    $display("PASS tb_rgb2y_full_pipe_top ALL");
    $finish;
  end

endmodule

`default_nettype wire
