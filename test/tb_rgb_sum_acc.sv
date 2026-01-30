// tb_rgb_sum_acc.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_sum_acc.vvp \
  ./test/tb_rgb_sum_acc.sv

vvp ./vvp/tb_rgb_sum_acc.vvp
gtkwave ./vvp/tb_rgb_sum_acc.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb_sum_acc.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_sum_acc;

  // -----------------------------
  // clock / reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // -----------------------------
  // DUT I/O
  // -----------------------------
  logic        en;
  logic        v2_valid;
  logic [15:0] r_term, g_term, b_term;

  logic        v3_valid;
  logic [16:0] sum;
  logic [7:0]  y_out;
  logic        ovf;

  // -----------------------------
  // params (match DUT)
  // -----------------------------
  localparam int unsigned LAT = 2;

  localparam bit          ZERO_WHEN_INVALID      = 1'b1;
  localparam bit          HOLD_VALID_WHEN_STALL  = 1'b1;

  localparam int unsigned SHIFT     = 8;
  localparam bit          ROUND     = 1'b1;
  localparam int unsigned YW        = 8;
  localparam bit          SATURATE  = 1'b1;

  // -----------------------------
  // DUT
  // -----------------------------
  rgb_sum_acc #(
    .LAT(LAT),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),
    .SHIFT(SHIFT),
    .ROUND(ROUND),
    .YW(YW),
    .SATURATE(SATURATE)
  ) dut (
    .clk(clk), .rst(rst),
    .en(en),
    .v2_valid(v2_valid),
    .r_term(r_term), .g_term(g_term), .b_term(b_term),
    .v3_valid(v3_valid),
    .sum(sum),
    .y_out(y_out),
    .ovf(ovf)
  );

  // -----------------------------
  // helpers (iverilog-safe casting)
  // -----------------------------
  function automatic [15:0] u16(input integer x);
    begin
      u16 = x[15:0];
    end
  endfunction

  // -----------------------------
  // expected pipeline (advance only when en=1)
  // -----------------------------
  localparam logic [16:0] SUM_MAX = 17'd65280;
  localparam integer BIAS_VAL = (ROUND && (SHIFT > 0)) ? (1 << (SHIFT-1)) : 0;

  logic [LAT:0] exp_v;
  logic [16:0]  exp_s   [0:LAT];
  logic         exp_ovf [0:LAT];

  task automatic clear_expected;
    integer i;
    begin
      exp_v = '0;
      for (i=0; i<=LAT; i=i+1) begin
        exp_s[i]   = 17'h0;
        exp_ovf[i] = 1'b0;
      end
    end
  endtask

  function automatic [16:0] exp_sum(input [15:0] r, input [15:0] g, input [15:0] b);
    begin
      exp_sum = {1'b0,r} + {1'b0,g} + {1'b0,b};
    end
  endfunction

  function automatic [7:0] exp_y_from_sum(input [16:0] s17);
    integer s18_bias;
    integer shifted;
    begin
      s18_bias = (s17 + BIAS_VAL);

      if (SHIFT >= 18) shifted = 0;
      else             shifted = (s18_bias >>> SHIFT);

      if (SATURATE) begin
        if (shifted > 255) exp_y_from_sum = 8'hFF;
        else if (shifted < 0) exp_y_from_sum = 8'h00;
        else exp_y_from_sum = shifted[7:0];
      end else begin
        exp_y_from_sum = shifted[7:0];
      end
    end
  endfunction

  task automatic step_and_check(
    input integer idx,
    input logic vin,
    input logic [15:0] r_in,
    input logic [15:0] g_in,
    input logic [15:0] b_in
  );
    logic [16:0] s0;
    logic        o0;
    integer k;
    logic [7:0]  yexp;
    begin
      v2_valid = vin;
      r_term   = r_in;
      g_term   = g_in;
      b_term   = b_in;

      s0 = exp_sum(r_in, g_in, b_in);
      o0 = (s0 > SUM_MAX);

      if (ZERO_WHEN_INVALID && !vin) begin
        s0 = 17'h0;
        o0 = 1'b0;
      end

      @(posedge clk);
      #1;

      if (rst) begin
        clear_expected();
      end else if (en) begin
        for (k=LAT; k>=1; k=k-1) begin
          exp_v[k]   = exp_v[k-1];
          exp_s[k]   = exp_s[k-1];
          exp_ovf[k] = exp_ovf[k-1];
        end
        exp_v[0]   = vin;
        exp_s[0]   = s0;
        exp_ovf[0] = o0;
      end
      // en==0: hold expected

      yexp = exp_y_from_sum(exp_s[LAT]);

      // valid check with stall policy
      if (!HOLD_VALID_WHEN_STALL && !en) begin
        if (v3_valid !== 1'b0) begin
          $display("FAIL valid(stall) idx=%0d got=%b exp=0", idx, v3_valid);
          $fatal;
        end
      end else begin
        if (v3_valid !== exp_v[LAT]) begin
          $display("FAIL valid idx=%0d got=%b exp=%b", idx, v3_valid, exp_v[LAT]);
          $fatal;
        end
      end

      // If ZERO_WHEN_INVALID=1, we can always check outputs (invalid cycles should be 0)
      if (ZERO_WHEN_INVALID) begin
        if (sum !== exp_s[LAT]) begin
          $display("FAIL sum idx=%0d got=%0d exp=%0d (v3_valid=%b)", idx, sum, exp_s[LAT], v3_valid);
          $fatal;
        end
        if (y_out !== yexp) begin
          $display("FAIL y_out idx=%0d got=%0d exp=%0d (sum=%0d)", idx, y_out, yexp, sum);
          $fatal;
        end
        if (ovf !== exp_ovf[LAT]) begin
          $display("FAIL ovf idx=%0d got=%b exp=%b (sum=%0d)", idx, ovf, exp_ovf[LAT], sum);
          $fatal;
        end
      end else begin
        if (v3_valid === 1'b1) begin
          if (sum !== exp_s[LAT]) begin
            $display("FAIL sum idx=%0d got=%0d exp=%0d", idx, sum, exp_s[LAT]);
            $fatal;
          end
          if (y_out !== yexp) begin
            $display("FAIL y_out idx=%0d got=%0d exp=%0d", idx, y_out, yexp);
            $fatal;
          end
          if (ovf !== exp_ovf[LAT]) begin
            $display("FAIL ovf idx=%0d got=%b exp=%b", idx, ovf, exp_ovf[LAT]);
            $fatal;
          end
        end
      end
    end
  endtask

  task automatic warmup(input integer n);
    integer i;
    begin
      for (i=0; i<n; i=i+1)
        step_and_check(-1000+i, 1'b0, 16'h0, 16'h0, 16'h0);
    end
  endtask

  // -----------------------------
  // tests
  // -----------------------------
  task automatic test_ramp;
    integer i;
    integer a, b, c;
    begin
      $display("[RUN ] RAMP");
      en = 1'b1;
      warmup(6);
      for (i=0; i<80; i=i+1) begin
        a = i;
        b = i*3;
        c = i*5;
        step_and_check(100+i, 1'b1, u16(a), u16(b), u16(c));
      end
      $display("[PASS] RAMP");
    end
  endtask

  task automatic test_max;
    begin
      $display("[RUN ] MAX");
      en = 1'b1;
      warmup(4);
      step_and_check(500, 1'b1, 16'd19635, 16'd38250, 16'd7395);
      warmup(LAT+4);
      $display("[PASS] MAX");
    end
  endtask

  task automatic test_bubbles;
    integer i;
    integer a, b, c;
    begin
      $display("[RUN ] BUBBLES");
      en = 1'b1;
      warmup(4);
      for (i=0; i<120; i=i+1) begin
        if ((i % 5)==2 || (i % 7)==3) begin
          // bubble invalid: drive nonzero to prove ZERO_WHEN_INVALID works
          step_and_check(800+i, 1'b0, 16'h1234, 16'h5678, 16'h9ABC);
        end else begin
          a = i+1;
          b = i+2;
          c = i+3;
          step_and_check(800+i, 1'b1, u16(a), u16(b), u16(c));
        end
      end
      $display("[PASS] BUBBLES");
    end
  endtask

  task automatic test_en_hold;
    integer i;
    integer a, b, c;
    logic v_snap;
    logic [16:0] s_snap;
    logic [7:0]  y_snap;
    logic        o_snap;
    begin
      $display("[RUN ] EN_HOLD");
      en = 1'b1;
      warmup(6);

      for (i=0; i<10; i=i+1) begin
        a = i*11;
        b = i*7;
        c = i*3;
        step_and_check(1200+i, 1'b1, u16(a), u16(b), u16(c));
      end

      v_snap = v3_valid;
      s_snap = sum;
      y_snap = y_out;
      o_snap = ovf;

      en = 1'b0;
      for (i=0; i<12; i=i+1) begin
        a = i*9;
        b = i*5;
        c = i*2;
        step_and_check(1300+i, 1'b1, u16(a), u16(b), u16(c));

        if (sum !== s_snap || y_out !== y_snap || ovf !== o_snap) begin
          $display("FAIL EN_HOLD data changed");
          $fatal;
        end
        if (HOLD_VALID_WHEN_STALL) begin
          if (v3_valid !== v_snap) begin
            $display("FAIL EN_HOLD valid changed");
            $fatal;
          end
        end else begin
          if (v3_valid !== 1'b0) begin
            $display("FAIL EN_HOLD valid not forced 0");
            $fatal;
          end
        end
      end

      en = 1'b1;
      warmup(4);
      $display("[PASS] EN_HOLD");
    end
  endtask

  task automatic test_midstream_reset;
    integer i;
    integer a, b, c;
    begin
      $display("[RUN ] MIDSTREAM_RESET");
      en = 1'b1;
      warmup(4);

      for (i=0; i<16; i=i+1) begin
        a = i;
        b = i*2;
        c = i*4;
        step_and_check(2000+i, 1'b1, u16(a), u16(b), u16(c));
      end

      rst = 1'b1;
      step_and_check(2100, 1'b0, 16'h0, 16'h0, 16'h0);
      rst = 1'b0;

      warmup(6);

      for (i=0; i<16; i=i+1) begin
        a = i+3;
        b = i+5;
        c = i+7;
        step_and_check(2200+i, 1'b1, u16(a), u16(b), u16(c));
      end

      $display("[PASS] MIDSTREAM_RESET");
    end
  endtask

  task automatic test_ovf;
    begin
      $display("[RUN ] OVF");
      en = 1'b1;
      warmup(4);

      step_and_check(3000, 1'b1, 16'd65535, 16'd65535, 16'd65535);
      warmup(LAT+2);

      $display("[PASS] OVF");
    end
  endtask

  // -----------------------------
  // main
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_rgb_sum_acc.vcd");
    $dumpvars(0, tb_rgb_sum_acc);

    rst = 1'b1;
    en  = 1'b1;
    v2_valid = 1'b0;
    r_term = 16'h0; g_term = 16'h0; b_term = 16'h0;
    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_ramp();
    test_max();
    test_bubbles();
    test_en_hold();
    test_midstream_reset();
    test_ovf();

    $display("PASS tb_rgb_sum_acc ALL");
    $finish;
  end

endmodule

`default_nettype wire
