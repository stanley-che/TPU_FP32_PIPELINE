// ============================================================
// tb_attn_norm_relu_fp32.sv  (IVERILOG-SAFE)
// - Tests attn_norm_relu_fp32 (normalized ReLU softmax approx)
// - Random input + random backpressure
// - Scoreboard compare (bit-exact FP32)
// - Hold check under stall: out_valid && !out_ready => w_flat stable
// - Drives inputs on negedge to avoid posedge race
// ============================================================
/*
iverilog -g2012 -Wall -o ./vvp/tb_attn_norm_relu \
  ./test/tb_attn_norm_relu_fp32.sv

vvp ./vvp/tb_attn_norm_relu
gtkwave tb_attn_norm_relu_fp32.vcd

*/
`include "./src/AMOLED/atten_core/attn_norm_relu_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// Provide rv_pipe if not already compiled elsewhere
// ------------------------------------------------------------
`ifndef RV_PIPE_INCLUDED
`define RV_PIPE_INCLUDED

module rv_pipe_stage #(
  parameter int unsigned WIDTH = 32
)(
  input  wire                  clk,
  input  wire                  rst_n,
  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [WIDTH-1:0]      in_data,
  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [WIDTH-1:0]      out_data
);
  reg             vld;
  reg [WIDTH-1:0] dat;

  assign in_ready  = (~vld) | out_ready;
  assign out_valid = vld;
  assign out_data  = dat;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vld <= 1'b0;
      dat <= '0;
    end else begin
      if (in_valid && in_ready) begin
        dat <= in_data;
        vld <= 1'b1;
      end else if (vld && out_ready) begin
        vld <= 1'b0;
      end
    end
  end
endmodule

module rv_pipe #(
  parameter int unsigned WIDTH  = 32,
  parameter int unsigned STAGES = 1
)(
  input  wire                  clk,
  input  wire                  rst_n,
  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [WIDTH-1:0]      in_data,
  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [WIDTH-1:0]      out_data
);
  generate
    if (STAGES == 0) begin : g_bypass
      assign in_ready  = out_ready;
      assign out_valid = in_valid;
      assign out_data  = in_data;
    end else begin : g_pipe
      wire [STAGES:0]  vld;
      wire [STAGES:0]  rdy;
      wire [WIDTH-1:0] dat [0:STAGES];

      assign vld[0]   = in_valid;
      assign dat[0]   = in_data;
      assign in_ready = rdy[0];

      assign out_valid   = vld[STAGES];
      assign out_data    = dat[STAGES];
      assign rdy[STAGES] = out_ready;

      genvar i;
      for (i = 0; i < STAGES; i = i + 1) begin : g_stage
        rv_pipe_stage #(.WIDTH(WIDTH)) u_stage (
          .clk       (clk),
          .rst_n     (rst_n),
          .in_valid  (vld[i]),
          .in_ready  (rdy[i]),
          .in_data   (dat[i]),
          .out_valid (vld[i+1]),
          .out_ready (rdy[i+1]),
          .out_data  (dat[i+1])
        );
      end
    end
  endgenerate
endmodule

`endif // RV_PIPE_INCLUDED


module tb_attn_norm_relu_fp32;

  // ----------------------------
  // Parameters
  // ----------------------------
  localparam int unsigned TOKENS      = 9;
  localparam int unsigned PIPE_STAGES = 2;
  localparam int unsigned W_W         = TOKENS*32;
  localparam real MARGIN = 1.0;

  localparam int PRINT_N = 5; // print first N outputs

  // ----------------------------
  // Clock/Reset
  // ----------------------------
  reg clk;
  reg rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                 in_valid;
  wire                in_ready;
  reg  [TOKENS*32-1:0] score_flat;

  wire                out_valid;
  reg                 out_ready;
  wire [TOKENS*32-1:0] w_flat;

  attn_norm_relu_fp32 #(
    .TOKENS(TOKENS),
    .PIPE_STAGES(PIPE_STAGES)
  ) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .in_valid  (in_valid),
    .in_ready  (in_ready),
    .score_flat(score_flat),
    .out_valid (out_valid),
    .out_ready (out_ready),
    .w_flat    (w_flat)
  );

  // ----------------------------------------------------------
  // pow2i + FP32 <-> real (match DUT style)
  // ----------------------------------------------------------
  function real pow2i(input integer e);
    integer i;
    real v;
    begin
      v = 1.0;
      if (e >= 0) begin
        for (i = 0; i < e; i = i + 1) v = v * 2.0;
      end else begin
        for (i = 0; i < (-e); i = i + 1) v = v / 2.0;
      end
      pow2i = v;
    end
  endfunction

  function real fp32_to_real(input [31:0] f);
    reg sign;
    integer exp, mant;
    real frac, val;
    begin
      sign = f[31];
      exp  = f[30:23];
      mant = f[22:0];

      if (exp == 0) begin
        if (mant == 0) val = 0.0;
        else begin
          frac = mant / 8388608.0;
          val  = frac * pow2i(-126);
        end
      end else if (exp == 255) begin
        val = (mant == 0) ? 1.0e30 : 0.0;
      end else begin
        frac = 1.0 + (mant / 8388608.0);
        val  = frac * pow2i(exp - 127);
      end

      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign;
    real a, v;
    integer e, exp_i, mant_i;
    real frac;
    reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;
      out  = 32'h0;

      if (a == 0.0) begin
        out = 32'h0;
      end else if (a > 3.4e38) begin
        out = {sign, 8'hFF, 23'h0};
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end
        exp_i = e + 127;

        if (exp_i <= 0) begin
          frac   = a / pow2i(-126);
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i < 0) mant_i = 0;
          if (mant_i > 8388607) mant_i = 8388607;
          out = {sign, 8'h00, mant_i[22:0]};
        end else if (exp_i >= 255) begin
          out = {sign, 8'hFF, 23'h0};
        end else begin
          frac   = v - 1.0;
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i >= 8388608) begin
            mant_i = 0;
            exp_i  = exp_i + 1;
            if (exp_i >= 255) out = {sign, 8'hFF, 23'h0};
            else              out = {sign, exp_i[7:0], mant_i[22:0]};
          end else begin
            out = {sign, exp_i[7:0], mant_i[22:0]};
          end
        end
      end

      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // Random FP32 generator (avoid NaN/Inf)
  // - produce around [-4,4)
  // ----------------------------------------------------------
  function [31:0] rand_fp32();
    integer s;
    integer e;
    integer m;
    reg [31:0] f;
    begin
      s = ($urandom() & 1);
      e = 125 + ($urandom() % 5); // {125..129} ~ [0.25..4)
      m = $urandom() & 23'h7FFFFF;
      f = {s[0], e[7:0], m[22:0]};
      rand_fp32 = f;
    end
  endfunction

  // ----------------------------------------------------------
  // Golden normalized ReLU
  // ----------------------------------------------------------
  function automatic [W_W-1:0] golden_w(input [TOKENS*32-1:0] sflat);
    integer t;
    real s [0:TOKENS-1];
    real m, sum;
    real wraw [0:TOKENS-1];
    reg [W_W-1:0] tmp;
    real u;
    begin
      tmp = '0;

      for (t = 0; t < TOKENS; t = t + 1)
        s[t] = fp32_to_real(sflat[t*32 +: 32]);

      m = s[0];
      for (t = 1; t < TOKENS; t = t + 1)
        if (s[t] > m) m = s[t];

      sum = 0.0;
      for (t = 0; t < TOKENS; t = t + 1) begin
        thr = m - MARGIN;
        wraw[t] = s[t] - thr;

        if (wraw[t] < 0.0) wraw[t] = 0.0;
        sum = sum + wraw[t];
      end

      if (sum == 0.0) begin
        u = 1.0 / TOKENS;
        for (t = 0; t < TOKENS; t = t + 1)
          tmp[t*32 +: 32] = real_to_fp32(u);
      end else begin
        for (t = 0; t < TOKENS; t = t + 1)
          tmp[t*32 +: 32] = real_to_fp32(wraw[t] / sum);
      end

      golden_w = tmp;
    end
  endfunction

  // ----------------------------------------------------------
  // Scoreboard (store expected output AND corresponding input)
  // ----------------------------------------------------------
  localparam int MAXQ = 1024;

  reg [W_W-1:0]          exp_w_q   [0:MAXQ-1];
  reg [TOKENS*32-1:0]    exp_s_q   [0:MAXQ-1];
  integer q_wr, q_rd, q_count;

  task sb_push_all(input [W_W-1:0] wexp, input [TOKENS*32-1:0] sin);
    begin
      if (q_count >= MAXQ) begin
        $display("[TB][FATAL] Scoreboard overflow");
        $finish;
      end
      exp_w_q[q_wr] = wexp;
      exp_s_q[q_wr] = sin;
      q_wr = (q_wr + 1) % MAXQ;
      q_count = q_count + 1;
    end
  endtask

  task sb_pop_all(output [W_W-1:0] wexp, output [TOKENS*32-1:0] sin);
    begin
      if (q_count <= 0) begin
        $display("[TB][FATAL] Scoreboard underflow");
        $finish;
      end
      wexp = exp_w_q[q_rd];
      sin  = exp_s_q[q_rd];
      q_rd = (q_rd + 1) % MAXQ;
      q_count = q_count - 1;
    end
  endtask

  // ----------------------------------------------------------
  // Hold-under-stall check
  // ----------------------------------------------------------
  reg [W_W-1:0] hold_prev;
  reg           hold_prev_valid;

  // ----------------------------------------------------------
  // Run control
  // ----------------------------------------------------------
  integer tx_sent, tx_goal;
  integer rx_cnt;
  reg done;

  initial begin
    // safer than void' cast for iverilog
    tx_goal = 300;
    tx_sent = 0;
    rx_cnt  = 0;
    done    = 1'b0;

    in_valid   = 1'b0;
    score_flat = '0;
    out_ready  = 1'b1;

    q_wr = 0; q_rd = 0; q_count = 0;
    hold_prev = '0;
    hold_prev_valid = 1'b0;

    $dumpfile("tb_attn_norm_relu_fp32.vcd");
    $dumpvars(0, tb_attn_norm_relu_fp32);

    @(posedge rst_n);
    repeat (5) @(posedge clk);

    fork
      drive_in();
      drive_out_ready();
      monitor_and_check();
    join_any
    disable fork;

    $display("[TB] DONE");
    $finish;
  end

  // ----------------------------------------------------------
  // Drive input on negedge to avoid race
  // ----------------------------------------------------------
  task automatic drive_in();
    integer t;
    reg [W_W-1:0] wexp;
    begin
      in_valid <= 1'b0;

      while (tx_sent < tx_goal) begin
        @(negedge clk);

        if (!rst_n) begin
          in_valid <= 1'b0;
        end else begin
          // if stalled, hold score_flat and in_valid (upstream compliant)
          if (!(in_valid && !in_ready)) begin
            if (($urandom() % 100) < 65) begin
              for (t = 0; t < TOKENS; t = t + 1)
                score_flat[t*32 +: 32] <= rand_fp32();
              in_valid <= 1'b1;
            end else begin
              in_valid <= 1'b0;
            end
          end
        end

        @(posedge clk);
        if (!rst_n) begin
          // no-op
        end else if (in_valid && in_ready) begin
          wexp = golden_w(score_flat);
          sb_push_all(wexp, score_flat);
          tx_sent = tx_sent + 1;
        end
      end

      while (in_valid && !in_ready) @(posedge clk);
      in_valid <= 1'b0;
    end
  endtask

  // ----------------------------------------------------------
  // Random out_ready
  // ----------------------------------------------------------
  task automatic drive_out_ready();
    begin
      forever begin
        @(posedge clk);
        if (!rst_n) out_ready <= 1'b0;
        else       out_ready <= (($urandom() % 100) < 70); // 70% ready
      end
    end
  endtask

  // ----------------------------------------------------------
  // Monitor/Check/Print and terminate
  // ----------------------------------------------------------
  task automatic monitor_and_check();
  reg [W_W-1:0]       wexp;
  reg [TOKENS*32-1:0] sin;
  integer t;
  integer idle_cycles;
  real sum_r;
  begin
    idle_cycles = 0;

    // iverilog 不支援 task 內 return，所以用 done flag 控制跳出
    while (!done) begin
      @(posedge clk);
      if (!rst_n) begin
        hold_prev_valid <= 1'b0;
        idle_cycles = 0;
      end else begin
        // hold check
        if (out_valid && !out_ready) begin
          if (hold_prev_valid) begin
            if (w_flat !== hold_prev) begin
              $display("[TB][FAIL] w_flat changed under stall!");
              $display("  prev=%h", hold_prev);
              $display("  now =%h", w_flat);
              $finish;
            end
          end
          hold_prev <= w_flat;
          hold_prev_valid <= 1'b1;
        end else begin
          hold_prev_valid <= 1'b0;
        end

        // consume + compare
        if (out_valid && out_ready) begin
          sb_pop_all(wexp, sin);

          // optional print first N
          if (rx_cnt < PRINT_N) begin
            $display("------------------------------------------------------------");
            $display("[TB] RX #%0d time=%0t", rx_cnt, $time);
            $display("[TB] score_in:");
            for (t = 0; t < TOKENS; t = t + 1)
              $display("  s[%0d]=%h (%f)", t, sin[t*32 +: 32], fp32_to_real(sin[t*32 +: 32]));

            sum_r = 0.0;
            $display("[TB] w_out (exp vs got):");
            for (t = 0; t < TOKENS; t = t + 1) begin
              sum_r = sum_r + fp32_to_real(w_flat[t*32 +: 32]);
              $display("  t=%0d exp=%h (%f)  got=%h (%f)",
                t,
                wexp[t*32 +: 32], fp32_to_real(wexp[t*32 +: 32]),
                w_flat[t*32 +: 32], fp32_to_real(w_flat[t*32 +: 32])
              );
            end
            $display("[TB] sum(got)=%f", sum_r);
          end

          rx_cnt = rx_cnt + 1;

          if (w_flat !== wexp) begin
            $display("[TB][FAIL] Mismatch at consume!");
            for (t = 0; t < TOKENS; t = t + 1) begin
              $display("  t=%0d exp=%h got=%h exp_r=%f got_r=%f",
                t,
                wexp[t*32 +: 32],
                w_flat[t*32 +: 32],
                fp32_to_real(wexp[t*32 +: 32]),
                fp32_to_real(w_flat[t*32 +: 32])
              );
            end
            $finish;
          end
        end

        // termination: sent enough, drained scoreboard, input not stuck
        if ((tx_sent >= tx_goal) && (q_count == 0) && !(in_valid && !in_ready))
          idle_cycles = idle_cycles + 1;
        else
          idle_cycles = 0;

        if (idle_cycles >= 30) begin
          done = 1'b1; // 讓 while(!done) 結束，thread 結束 -> join_any 觸發
        end
      end
    end
  end
endtask


endmodule

`default_nettype wire
