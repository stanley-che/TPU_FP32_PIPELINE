// tb_rgb_y_pipe_top.sv (iverilog-safe)
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_y_pipe_top.vvp \
  ./test/tb_rgb_y_pipe_top.sv

vvp ./vvp/tb_rgb_y_pipe_top.vvp
gtkwave ./vvp/tb_rgb_y_pipe_top.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb_y_pipe_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_y_pipe_top;

  // clock / reset
  reg clk = 1'b0;
  always #5 clk = ~clk;
  reg rst;

  // DUT I/O
  reg         en;
  reg         v2_valid;
  reg [15:0]  r_term, g_term, b_term;

  reg         out_ready;
  wire        in_ready;

  wire        v4_valid;
  wire [16:0] sum17;
  wire [9:0]  y10;
  wire [7:0]  y8;
  wire        ovf_sum;
  wire        ovf_y;

  // params
  localparam integer LAT_SUM = 2;
  localparam integer LAT_RND = 2;

  localparam ZERO_WHEN_INVALID      = 1'b1;
  localparam HOLD_VALID_WHEN_STALL  = 1'b1;

  localparam integer ROUND_MODE = 1;
  localparam integer Y8_MODE    = 0;

  localparam integer RND_DEPTH = (LAT_RND==0) ? 1 : LAT_RND;
  localparam integer TOT = LAT_SUM + RND_DEPTH;

  // DUT
  rgb_y_pipe_top #(
    .LAT_SUM(LAT_SUM),
    .LAT_RND(LAT_RND),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE)
  ) dut (
    .clk(clk), .rst(rst),
    .en(en),

    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term),

    .out_ready(out_ready),
    .in_ready(in_ready),

    .v4_valid(v4_valid),
    .sum17(sum17),
    .y10(y10),
    .y8(y8),
    .ovf_sum(ovf_sum),
    .ovf_y(ovf_y)
  );

  // expected model
  localparam integer SUM_MAX = 65280;

  reg [TOT:0]  exp_v;
  reg [16:0]   exp_sum [0:TOT];
  reg [9:0]    exp_y10 [0:TOT];
  reg [7:0]    exp_y8  [0:TOT];
  reg          exp_ovf_sum [0:TOT];
  reg          exp_ovf_y   [0:TOT];

  integer k;

  task clear_expected;
    integer i;
    begin
      exp_v = { (TOT+1){1'b0} };
      for (i=0; i<=TOT; i=i+1) begin
        exp_sum[i]     = 17'h0;
        exp_y10[i]     = 10'h0;
        exp_y8[i]      = 8'h00;
        exp_ovf_sum[i] = 1'b0;
        exp_ovf_y[i]   = 1'b0;
      end
    end
  endtask

  function [16:0] f_sum17;
    input [15:0] r;
    input [15:0] g;
    input [15:0] b;
    reg [16:0] s;
    begin
      s = {1'b0,r} + {1'b0,g} + {1'b0,b};
      f_sum17 = s;
    end
  endfunction

  function f_ovf_sum;
    input [16:0] s;
    begin
      f_ovf_sum = (s > SUM_MAX);
    end
  endfunction

  function [9:0] f_y10_round;
    input [16:0] s;
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

  function [7:0] f_y8_from_y10;
    input [9:0] y10_in;
    begin
      if (Y8_MODE == 0) begin
        f_y8_from_y10 = y10_in[9:2];
      end else begin
        if (y10_in > 10'd255) f_y8_from_y10 = 8'hFF;
        else                  f_y8_from_y10 = y10_in[7:0];
      end
    end
  endfunction

  function f_ovf_y_from;
    input [9:0] y10_in;
    begin
      if (Y8_MODE == 1 && (y10_in > 10'd255)) f_ovf_y_from = 1'b1;
      else                                    f_ovf_y_from = 1'b0;
    end
  endfunction

  // one-cycle step (handshake-correct)
  task step_and_check;
    input integer idx;
    input         want_valid;
    input [15:0]  r_in;
    input [15:0]  g_in;
    input [15:0]  b_in;
    input         en_in;
    input         out_ready_in;
    reg do_send;
    reg adv;
    reg [16:0] s0;
    reg o_sum0;
    reg [9:0] y10_0;
    reg [7:0] y8_0;
    reg o_y0;
    begin
      en        = en_in;
      out_ready = out_ready_in;

      do_send = (want_valid && in_ready);

      v2_valid = do_send;
      r_term   = r_in;
      g_term   = g_in;
      b_term   = b_in;

      adv = in_ready;

      if (do_send) begin
        s0     = f_sum17(r_in,g_in,b_in);
        o_sum0 = f_ovf_sum(s0);
        y10_0  = f_y10_round(s0);
        y8_0   = f_y8_from_y10(y10_0);
        o_y0   = f_ovf_y_from(y10_0);
      end else begin
        s0     = 17'h0;
        o_sum0 = 1'b0;
        y10_0  = 10'h0;
        y8_0   = 8'h00;
        o_y0   = 1'b0;
      end

      @(posedge clk);
      #1;

      if (rst) begin
        clear_expected();
      end else if (adv) begin
        for (k=TOT; k>=1; k=k-1) begin
          exp_v[k]        = exp_v[k-1];
          exp_sum[k]      = exp_sum[k-1];
          exp_ovf_sum[k]  = exp_ovf_sum[k-1];
          exp_y10[k]      = exp_y10[k-1];
          exp_y8[k]       = exp_y8[k-1];
          exp_ovf_y[k]    = exp_ovf_y[k-1];
        end
        exp_v[0]        = do_send;
        exp_sum[0]      = s0;
        exp_ovf_sum[0]  = o_sum0;
        exp_y10[0]      = y10_0;
        exp_y8[0]       = y8_0;
        exp_ovf_y[0]    = o_y0;
      end

      if (v4_valid !== exp_v[TOT]) begin
        $display("FAIL v4_valid idx=%0d got=%b exp=%b (in_ready=%b en=%b out_ready=%b)",
                 idx, v4_valid, exp_v[TOT], in_ready, en, out_ready);
        $finish;
      end
      if (sum17 !== exp_sum[TOT]) begin
        $display("FAIL sum17 idx=%0d got=%0d exp=%0d", idx, sum17, exp_sum[TOT]);
        $finish;
      end
      if (ovf_sum !== exp_ovf_sum[TOT]) begin
        $display("FAIL ovf_sum idx=%0d got=%b exp=%b", idx, ovf_sum, exp_ovf_sum[TOT]);
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
      if (ovf_y !== exp_ovf_y[TOT]) begin
        $display("FAIL ovf_y idx=%0d got=%b exp=%b", idx, ovf_y, exp_ovf_y[TOT]);
        $finish;
      end
    end
  endtask

  task warmup;
    input integer n;
    integer i;
    begin
      for (i=0; i<n; i=i+1)
        step_and_check(-1000+i, 1'b0, 16'h0,16'h0,16'h0, 1'b1, 1'b1);
    end
  endtask

  task test_basic;
    begin
      $display("[RUN ] BASIC");
      warmup(6);

      step_and_check(0, 1'b1, 16'd0,   16'd0,   16'd0,   1'b1, 1'b1);
      step_and_check(1, 1'b1, 16'd128, 16'd0,   16'd0,   1'b1, 1'b1);
      step_and_check(2, 1'b1, 16'd255, 16'd0,   16'd0,   1'b1, 1'b1);
      step_and_check(3, 1'b1, 16'd0,   16'd255, 16'd0,   1'b1, 1'b1);
      step_and_check(4, 1'b1, 16'd0,   16'd0,   16'd255, 1'b1, 1'b1);

      warmup(TOT+4);
      $display("[PASS] BASIC");
    end
  endtask

  task test_ramp;
    integer n;
    integer a,b,c;
    begin
      $display("[RUN ] RAMP");
      warmup(4);
      for (n=0; n<200; n=n+1) begin
        a = (n*3) & 16'hFFFF;
        b = (n*5) & 16'hFFFF;
        c = (n*7) & 16'hFFFF;
        step_and_check(1000+n, 1'b1, a[15:0], b[15:0], c[15:0], 1'b1, 1'b1);
      end
      warmup(TOT+4);
      $display("[PASS] RAMP");
    end
  endtask

  task test_bubbles;
    integer n;
    integer a,b,c;
    begin
      $display("[RUN ] BUBBLES");
      warmup(4);
      for (n=0; n<200; n=n+1) begin
        if ((n%7)==3 || (n%11)==5) begin
          step_and_check(2000+n, 1'b0, 16'h1234,16'h5678,16'h9abc, 1'b1, 1'b1);
        end else begin
          a = (n*9) & 16'hFFFF;
          b = (n*2) & 16'hFFFF;
          c = (n*4) & 16'hFFFF;
          step_and_check(2000+n, 1'b1, a[15:0], b[15:0], c[15:0], 1'b1, 1'b1);
        end
      end
      warmup(TOT+4);
      $display("[PASS] BUBBLES");
    end
  endtask

  task test_en_hold;
    integer n;
    reg v_snap;
    reg [16:0] s_snap;
    reg [9:0]  y10_snap;
    reg [7:0]  y8_snap;
    reg os_snap, oy_snap;
    integer a,b,c;
    begin
      $display("[RUN ] EN_HOLD");
      warmup(6);

      for (n=0; n<20; n=n+1) begin
        a = (n*3) & 16'hFFFF;
        b = (n*5) & 16'hFFFF;
        c = (n*7) & 16'hFFFF;
        step_and_check(3000+n, 1'b1, a[15:0], b[15:0], c[15:0], 1'b1, 1'b1);
      end

      v_snap   = v4_valid;
      s_snap   = sum17;
      y10_snap = y10;
      y8_snap  = y8;
      os_snap  = ovf_sum;
      oy_snap  = ovf_y;

      for (n=0; n<20; n=n+1) begin
        step_and_check(3100+n, 1'b1, 16'hffff,16'hffff,16'hffff, 1'b0, 1'b1);
        if (v4_valid !== v_snap || sum17 !== s_snap || y10 !== y10_snap || y8 !== y8_snap ||
            ovf_sum !== os_snap || ovf_y !== oy_snap) begin
          $display("FAIL EN_HOLD outputs changed at n=%0d", n);
          $finish;
        end
      end

      warmup(8);
      $display("[PASS] EN_HOLD");
    end
  endtask

  task test_out_ready_stall;
    integer n;
    reg v_snap;
    reg [16:0] s_snap;
    reg [9:0]  y10_snap;
    reg [7:0]  y8_snap;
    integer a,b,c;
    begin
      $display("[RUN ] OUT_READY_STALL");
      warmup(6);

      for (n=0; n<20; n=n+1) begin
        a = (n*11) & 16'hFFFF;
        b = (n*13) & 16'hFFFF;
        c = (n*17) & 16'hFFFF;
        step_and_check(4000+n, 1'b1, a[15:0], b[15:0], c[15:0], 1'b1, 1'b1);
      end

      v_snap   = v4_valid;
      s_snap   = sum17;
      y10_snap = y10;
      y8_snap  = y8;

      for (n=0; n<20; n=n+1) begin
        step_and_check(4100+n, 1'b1, 16'h1111,16'h2222,16'h3333, 1'b1, 1'b0);
        if (v4_valid !== v_snap || sum17 !== s_snap || y10 !== y10_snap || y8 !== y8_snap) begin
          $display("FAIL OUT_READY_STALL outputs changed at n=%0d", n);
          $finish;
        end
      end

      warmup(10);
      $display("[PASS] OUT_READY_STALL");
    end
  endtask

  task test_midstream_reset;
    integer n;
    integer a,b,c;
    begin
      $display("[RUN ] MIDSTREAM_RESET");
      warmup(6);

      for (n=0; n<20; n=n+1) begin
        a = (n*4) & 16'hFFFF;
        b = (n*6) & 16'hFFFF;
        c = (n*8) & 16'hFFFF;
        step_and_check(5000+n, 1'b1, a[15:0], b[15:0], c[15:0], 1'b1, 1'b1);
      end

      rst = 1'b1;
      step_and_check(5100, 1'b0, 16'h0,16'h0,16'h0, 1'b1, 1'b1);
      rst = 1'b0;

      warmup(10);
      $display("[PASS] MIDSTREAM_RESET");
    end
  endtask

  // main
  initial begin
    $dumpfile("./vvp/tb_rgb_y_pipe_top.vcd");
    $dumpvars(0, tb_rgb_y_pipe_top);

    rst = 1'b1;
    en  = 1'b1;

    v2_valid = 1'b0;
    r_term = 16'h0; g_term = 16'h0; b_term = 16'h0;

    out_ready = 1'b1;

    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_basic();
    test_ramp();
    test_bubbles();
    test_en_hold();
    test_out_ready_stall();
    test_midstream_reset();

    $display("PASS tb_rgb_y_pipe_top ALL");
    $finish;
  end

endmodule

`default_nettype wire
