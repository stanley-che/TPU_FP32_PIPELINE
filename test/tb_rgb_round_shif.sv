// tb_rgb_round_shift.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_round_shift.vvp \
  ./test/tb_rgb_round_shif.sv

vvp ./vvp/tb_rgb_round_shift.vvp
gtkwave ./vvp/tb_rgb_round_shift.vcd

*/

`include "./src/AMOLED/rgb2y_luma/rgb_round_shift.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_round_shift;

  // clock / reset
  reg clk = 1'b0;
  always #5 clk = ~clk;
  reg rst;

  // DUT I/O
  reg         en;
  reg         v3_valid;
  reg [16:0]  sum;

  wire        v4_valid;
  wire [9:0]  y10;
  wire [7:0]  y8;
  reg  out_ready;
  wire in_ready;
  wire ovf;

  // params
  localparam integer LAT = 2;
  localparam ZERO_WHEN_INVALID = 1'b1;

  // DUT
    rgb_round_shift #(
    .LAT(LAT),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID)
  ) dut (
    .clk(clk), .rst(rst),
    .en(en),
    .v3_valid(v3_valid),
    .sum(sum),
    .out_ready(out_ready),
    .in_ready(in_ready),
    .v4_valid(v4_valid),
    .y10(y10),
    .y8(y8),
    .ovf(ovf)
  );


  // expected pipeline (advance only when en=1)
  integer i;
  reg [LAT:0] exp_v;
  reg [9:0]   exp_y10 [0:LAT];

  // ------------------------------------------------------------
  // tasks / functions (iverilog-safe style)
  // ------------------------------------------------------------
  task clear_expected;
    integer k;
    begin
      exp_v = { (LAT+1){1'b0} };
      for (k=0; k<=LAT; k=k+1)
        exp_y10[k] = 10'h000;
    end
  endtask

  function [9:0] f_y10;
    input [16:0] s17;
    reg   [17:0] bias;
    begin
      bias = {1'b0, s17} + 18'd128;
      f_y10 = bias[17:8]; // >> 8
    end
  endfunction

  task step_and_check;
    input integer idx;
    input         vin;
    input [16:0]  sum_in;
    integer k;
    reg [9:0] y0;
    begin
      // drive inputs
      v3_valid = vin;
      sum      = sum_in;

      // stage0 expected
      if (ZERO_WHEN_INVALID && !vin)
        y0 = 10'h000;
      else
        y0 = f_y10(sum_in);

      @(posedge clk);
      #1;

      if (rst) begin
        clear_expected();
      end else if (en) begin
        // shift pipeline
        for (k=LAT; k>=1; k=k-1) begin
          exp_v[k]   = exp_v[k-1];
          exp_y10[k] = exp_y10[k-1];
        end
        exp_v[0]   = vin;
        exp_y10[0] = y0;
      end
      // en==0: hold expected

      // checks
      if (v4_valid !== exp_v[LAT]) begin
        $display("FAIL valid idx=%0d got=%b exp=%b", idx, v4_valid, exp_v[LAT]);
        $finish;
      end

      // if ZERO_WHEN_INVALID=1, we can always check y outputs
      if (ZERO_WHEN_INVALID) begin
        if (y10 !== exp_y10[LAT]) begin
          $display("FAIL y10 idx=%0d got=%0d exp=%0d", idx, y10, exp_y10[LAT]);
          $finish;
        end
        if (y8 !== exp_y10[LAT][9:2]) begin
          $display("FAIL y8 idx=%0d got=%0d exp=%0d", idx, y8, exp_y10[LAT][9:2]);
          $finish;
        end
      end else begin
        // otherwise, check only when valid
        if (v4_valid === 1'b1) begin
          if (y10 !== exp_y10[LAT]) begin
            $display("FAIL y10 idx=%0d got=%0d exp=%0d", idx, y10, exp_y10[LAT]);
            $finish;
          end
          if (y8 !== exp_y10[LAT][9:2]) begin
            $display("FAIL y8 idx=%0d got=%0d exp=%0d", idx, y8, exp_y10[LAT][9:2]);
            $finish;
          end
        end
      end
    end
  endtask

  task warmup;
    input integer n;
    integer k;
    begin
      for (k=0; k<n; k=k+1)
        step_and_check(-1000+k, 1'b0, 17'h0);
    end
  endtask

  // ------------------------------------------------------------
  // tests
  // ------------------------------------------------------------
  task test_basic;
    begin
      $display("[RUN ] BASIC");
      en = 1'b1;
      warmup(6);

      step_and_check(0, 1'b1, 17'd0);
      step_and_check(1, 1'b1, 17'd127);
      step_and_check(2, 1'b1, 17'd128);
      step_and_check(3, 1'b1, 17'd255);
      step_and_check(4, 1'b1, 17'd256);
      step_and_check(5, 1'b1, 17'd383);
      step_and_check(6, 1'b1, 17'd384);

      warmup(LAT+4);
      $display("[PASS] BASIC");
    end
  endtask

  task test_ramp;
    integer n;
    begin
      $display("[RUN ] RAMP");
      en = 1'b1;
      warmup(4);
      for (n=0; n<300; n=n+7) begin
        step_and_check(1000+n, 1'b1, (n & 17'h1FFFF));
      end
      warmup(LAT+4);
      $display("[PASS] RAMP");
    end
  endtask

  task test_bubbles;
    integer n;
    integer tmp;
    begin
      $display("[RUN ] BUBBLES");
      en = 1'b1;
      warmup(4);
      for (n=0; n<120; n=n+1) begin
        if ((n % 5)==2 || (n % 7)==3)
          step_and_check(2000+n, 1'b0, 17'd60000); // invalid but nonzero
        else begin
          tmp = n * 321;
          step_and_check(2000+n, 1'b1, (tmp & 17'h1FFFF));
        end
      end
      warmup(LAT+4);
      $display("[PASS] BUBBLES");
    end
  endtask

  task test_en_hold;
    integer n;
    integer tmp;
    reg v_snap;
    reg [9:0] y10_snap;
    reg [7:0] y8_snap;
    begin
      $display("[RUN ] EN_HOLD");
      en = 1'b1;
      warmup(6);

      // load pipeline
      for (n=0; n<10; n=n+1) begin
        tmp = n * 5000;
        step_and_check(3000+n, 1'b1, (tmp & 17'h1FFFF));
      end

      // snapshot
      v_snap   = v4_valid;
      y10_snap = y10;
      y8_snap  = y8;

      // hold
      en = 1'b0;
      for (n=0; n<12; n=n+1) begin
        tmp = n * 7777;
        step_and_check(3100+n, 1'b1, (tmp & 17'h1FFFF));
        if (v4_valid !== v_snap || y10 !== y10_snap || y8 !== y8_snap) begin
          $display("FAIL EN_HOLD outputs changed");
          $finish;
        end
      end

      // resume
      en = 1'b1;
      warmup(6);
      $display("[PASS] EN_HOLD");
    end
  endtask

  task test_midstream_reset;
    integer n;
    integer tmp;
    begin
      $display("[RUN ] MIDSTREAM_RESET");
      en = 1'b1;
      warmup(4);

      for (n=0; n<16; n=n+1) begin
        tmp = n * 1234;
        step_and_check(4000+n, 1'b1, (tmp & 17'h1FFFF));
      end

      rst = 1'b1;
      step_and_check(4100, 1'b0, 17'h0);
      rst = 1'b0;

      warmup(6);

      for (n=0; n<16; n=n+1) begin
        tmp = n * 4321;
        step_and_check(4200+n, 1'b1, (tmp & 17'h1FFFF));
      end

      $display("[PASS] MIDSTREAM_RESET");
    end
  endtask

  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_rgb_round_shift.vcd");
    $dumpvars(0, tb_rgb_round_shift);
    out_ready = 1'b1;

    rst = 1'b1;
    en  = 1'b1;
    v3_valid = 1'b0;
    sum = 17'h0;
    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_basic();
    test_ramp();
    test_bubbles();
    test_en_hold();
    test_midstream_reset();

    $display("PASS tb_rgb_round_shift ALL");
    $finish;
  end

endmodule

`default_nettype wire
