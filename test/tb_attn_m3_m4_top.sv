// ============================================================
// tb_attn_m3_m4_top.sv  (IVERILOG-SAFE / NO automatic / NO real arrays)
// - Test attn_m3_m4_top (M3 weighted-sum -> M4 alpha)
// - Golden:
//    out_vec[d] = sum_t w[t] * V[t][d]
//    alpha      = g(out_vec[0])  (MODE selectable, same approx as DUT)
// - Checks:
//   1) out_vec_dbg matches golden out_vec
//   2) alpha_fp32  matches golden alpha
//   3) hold under stall: out_valid && !out_ready => out_vec_dbg/alpha stable
// ============================================================
/*
iverilog -g2012 -Wall \
  ./test/tb_attn_m3_m4_top.sv \
  -o ./vvp/tb_m34

vvp ./vvp/tb_m34
gtkwave tb_attn_m3_m4_top.vcd
*/
`include "./src/AMOLED/atten_core/attn_m3_m4_top.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_attn_m3_m4_top;

  // ----------------------------------------------------------
  // Parameters (match your RTL defaults)
  // ----------------------------------------------------------
  localparam int unsigned TOKENS      = 9;
  localparam int unsigned D           = 8;

  localparam int unsigned M3_PIPE_STG = 1;
  localparam int unsigned M4_PIPE_STG = 1;

  // 0=sigmoid, 1=clamp, 2=linear+clamp
  localparam int unsigned ALPHA_MODE  = 1;
  localparam real         ALPHA_SCALE = 1.0;
  localparam real         ALPHA_BIAS  = 0.0;
  localparam real         ALPHA_SIG_A = 1.0;

  localparam int unsigned W_W   = TOKENS*32;
  localparam int unsigned V_W   = TOKENS*D*32;
  localparam int unsigned OUT_W = D*32;

  localparam int TX_GOAL = 200;
  localparam int PRINT_N = 8;

  // ----------------------------------------------------------
  // Clock / Reset
  // ----------------------------------------------------------
  reg clk, rst_n;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end
  initial begin
    rst_n = 1'b0;
    repeat (6) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------------------------------------
  // DUT I/O
  // ----------------------------------------------------------
  reg                 in_valid;
  wire                in_ready;
  reg  [W_W-1:0]       w_flat;
  reg  [V_W-1:0]       v_vecs;

  wire                out_valid;
  reg                 out_ready;
  wire [31:0]          alpha_fp32;
  wire [OUT_W-1:0]     out_vec_dbg;

  attn_m3_m4_top #(
    .TOKENS      (TOKENS),
    .D           (D),
    .M3_PIPE_STG (M3_PIPE_STG),
    .M4_PIPE_STG (M4_PIPE_STG),
    .ALPHA_MODE  (ALPHA_MODE),
    .ALPHA_SCALE (ALPHA_SCALE),
    .ALPHA_BIAS  (ALPHA_BIAS),
    .ALPHA_SIG_A (ALPHA_SIG_A)
  ) dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (in_valid),
    .in_ready   (in_ready),
    .w_flat     (w_flat),
    .v_vecs     (v_vecs),
    .out_valid  (out_valid),
    .out_ready  (out_ready),
    .alpha_fp32 (alpha_fp32),
    .out_vec_dbg(out_vec_dbg)
  );

  // ----------------------------------------------------------
  // pow2i + FP32 pack/unpack (IVERILOG-SAFE)
  // ----------------------------------------------------------
  function real pow2i(input integer e);
    integer i; real v;
    begin
      v = 1.0;
      if (e >= 0) for (i=0;i<e;i=i+1) v = v * 2.0;
      else        for (i=0;i<-e;i=i+1) v = v / 2.0;
      pow2i = v;
    end
  endfunction

  function real fp32_to_real(input [31:0] f);
    reg sign; integer exp, mant; real frac, val;
    begin
      sign = f[31]; exp = f[30:23]; mant = f[22:0];
      if (exp == 0) begin
        if (mant == 0) val = 0.0;
        else begin
          frac = mant / 8388608.0;
          val  = frac * pow2i(-126);
        end
      end else if (exp == 255) begin
        val = (mant==0) ? 1.0e30 : 0.0;
      end else begin
        frac = 1.0 + (mant / 8388608.0);
        val  = frac * pow2i(exp - 127);
      end
      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign; real a,v; integer e, exp_i, mant_i; real frac; reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;
      out  = 32'h0;

      if (a == 0.0) begin
        out = 32'h0;
      end else begin
        v=a; e=0;
        while (v >= 2.0) begin v=v/2.0; e=e+1; end
        while (v <  1.0) begin v=v*2.0; e=e-1; end
        exp_i = e + 127;
        frac  = v - 1.0;
        mant_i = integer'(frac * 8388608.0 + 0.5);
        if (mant_i >= 8388608) begin mant_i = 0; exp_i = exp_i + 1; end
        out = {sign, exp_i[7:0], mant_i[22:0]};
      end
      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // Random generators
  // ----------------------------------------------------------
  function [31:0] rand_fp32_small();
    integer r; real x;
    begin
      r = ($urandom() % 401) - 200; // [-200..200]
      x = r / 100.0;                // [-2..2]
      rand_fp32_small = real_to_fp32(x);
    end
  endfunction

  function [31:0] rand_out0();
    integer r; real x;
    begin
      r = ($urandom() % 2401) - 1200; // [-1200..1200]
      x = r / 200.0;                  // [-6..6]
      rand_out0 = real_to_fp32(x);
    end
  endfunction

  // ----------------------------------------------------------
  // Golden: out_vec[d] = sum_t w[t] * v[t][d]
  // ----------------------------------------------------------
  function [OUT_W-1:0] golden_out_vec(input [W_W-1:0] w_in, input [V_W-1:0] v_in);
    integer d, t;
    real acc, wr, vr;
    reg [OUT_W-1:0] tmp;
    begin
      tmp = '0;
      for (d = 0; d < D; d = d + 1) begin
        acc = 0.0;
        for (t = 0; t < TOKENS; t = t + 1) begin
          wr = fp32_to_real(w_in[t*32 +: 32]);
          vr = fp32_to_real(v_in[(t*D + d)*32 +: 32]);
          acc = acc + (wr * vr);
        end
        tmp[d*32 +: 32] = real_to_fp32(acc);
      end
      golden_out_vec = tmp;
    end
  endfunction

  // ----------------------------------------------------------
  // Golden alpha (same approx as DUT)
  // ----------------------------------------------------------
  function real clamp01(input real x);
    begin
      if (x < 0.0) clamp01 = 0.0;
      else if (x > 1.0) clamp01 = 1.0;
      else clamp01 = x;
    end
  endfunction

  localparam real LN2   = 0.6931471805599453;
  localparam real LOG2E = 1.4426950408889634;

  function real exp2_frac(input real F);
    real z,z2,z3;
    begin
      z=F*LN2; z2=z*z; z3=z2*z;
      exp2_frac = 1.0 + z + (z2*0.5) + (z3*(1.0/6.0));
    end
  endfunction

  function real exp2_approx(input real y);
    integer it, ifloor; real F;
    begin
      it = integer'(y);
      if ((y < 0.0) && (y != it)) ifloor = it - 1;
      else                        ifloor = it;
      F = y - ifloor;
      exp2_approx = pow2i(ifloor) * exp2_frac(F);
    end
  endfunction

  function real sigmoid_approx(input real x);
    real z, e;
    begin
      if (x > 8.0) sigmoid_approx = 0.999664;
      else if (x < -8.0) sigmoid_approx = 0.000335;
      else begin
        z = (-ALPHA_SIG_A * x) * LOG2E;
        e = exp2_approx(z);
        sigmoid_approx = 1.0 / (1.0 + e);
      end
    end
  endfunction

  function [31:0] golden_alpha(input [31:0] out0_bits);
    real x, a;
    begin
      x = fp32_to_real(out0_bits);
      if (ALPHA_MODE == 0)      a = sigmoid_approx(x);
      else if (ALPHA_MODE == 1) a = clamp01(x);
      else                      a = clamp01(ALPHA_SCALE*x + ALPHA_BIAS);
      golden_alpha = real_to_fp32(a);
    end
  endfunction

  // ----------------------------------------------------------
  // Scoreboard (expected out_vec + alpha)
  // ----------------------------------------------------------
  localparam int MAXQ = 1024;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  reg [31:0]      exp_a_q   [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [OUT_W-1:0] o, input [31:0] a);
    begin
      exp_out_q[sb_wr] = o;
      exp_a_q[sb_wr]   = a;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o, output [31:0] a);
    begin
      o = exp_out_q[sb_rd];
      a = exp_a_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // ----------------------------------------------------------
  // Hold-under-stall check
  // ----------------------------------------------------------
  reg [OUT_W-1:0] hold_out_prev;
  reg [31:0]      hold_a_prev;
  reg             hold_v;

  // ----------------------------------------------------------
  // Driver temps (avoid real arrays)
  // ----------------------------------------------------------
  integer w_int [0:TOKENS-1];
  real    sum_w;
  real    wi;
  reg [OUT_W-1:0] exp_out_tmp;
  reg [31:0]      exp_a_tmp;

  integer tx, rx, idle;

  // ----------------------------------------------------------
  // Main
  // ----------------------------------------------------------
  initial begin
    in_valid  = 1'b0;
    out_ready = 1'b1;
    w_flat    = '0;
    v_vecs    = '0;

    sb_wr=0; sb_rd=0; sb_cnt=0;
    tx=0; rx=0; idle=0;
    hold_out_prev='0; hold_a_prev=32'h0; hold_v=1'b0;

    $dumpfile("tb_attn_m3_m4_top.vcd");
    $dumpvars(0, tb_attn_m3_m4_top);

    @(posedge rst_n);
    repeat (3) @(posedge clk);

    fork
      drive_in();
      drive_out_ready();
      monitor();
    join_any
    disable fork;

    $display("[TB] DONE");
    $finish;
  end

  // ----------------------------------------------------------
  // Drive input (respect in_ready; hold when stalled)
  // - weights: non-negative ints then normalize to sum=1
  // - v_vecs : random small, but force out_vec[0] to vary
  // ----------------------------------------------------------
  task drive_in();
    integer t, d;
    begin
      while (tx < TX_GOAL) begin
        @(negedge clk);

        // if stalled, hold w_flat/v_vecs/in_valid
        if (!(in_valid && !in_ready)) begin
          if (in_ready && (($urandom()%100) < 70)) begin
            // random V
            for (t = 0; t < TOKENS; t = t + 1) begin
              // make V[t][0] a bit wider dynamic range sometimes
              v_vecs[(t*D + 0)*32 +: 32] <= rand_out0();
              for (d = 1; d < D; d = d + 1)
                v_vecs[(t*D + d)*32 +: 32] <= rand_fp32_small();
            end

            // random non-negative weights -> normalize
            sum_w = 0.0;
            for (t = 0; t < TOKENS; t = t + 1) begin
              w_int[t] = ($urandom() % 1000); // 0..999
              sum_w = sum_w + w_int[t];
            end
            if (sum_w == 0.0) sum_w = 1.0;

            for (t = 0; t < TOKENS; t = t + 1) begin
              wi = w_int[t] / sum_w;
              w_flat[t*32 +: 32] <= real_to_fp32(wi);
            end

            in_valid <= 1'b1;
          end else begin
            in_valid <= 1'b0;
          end
        end

        @(posedge clk);
        if (rst_n && in_valid && in_ready) begin
          exp_out_tmp = golden_out_vec(w_flat, v_vecs);
          exp_a_tmp   = golden_alpha(exp_out_tmp[0*32 +: 32]);
          sb_push(exp_out_tmp, exp_a_tmp);
          tx = tx + 1;
        end
      end

      // drain input
      while (in_valid && !in_ready) @(posedge clk);
      in_valid <= 1'b0;
    end
  endtask

  // ----------------------------------------------------------
  // Random backpressure
  // ----------------------------------------------------------
  task drive_out_ready();
    begin
      forever begin
        @(posedge clk);
        if (!rst_n) out_ready <= 1'b0;
        else       out_ready <= (($urandom()%100) < 70);
      end
    end
  endtask

  // ----------------------------------------------------------
  // Monitor/check
  // ----------------------------------------------------------
  task monitor();
    reg [OUT_W-1:0] exp_out;
    reg [31:0]      exp_a;
    integer d;
    real ar, er;
    begin
      forever begin
        @(posedge clk);

        if (!rst_n) begin
          hold_v <= 1'b0;
          idle   <= 0;
        end else begin
          // hold check
          if (out_valid && !out_ready) begin
            if (hold_v) begin
              if (out_vec_dbg !== hold_out_prev) begin
                $display("[TB][FAIL] out_vec_dbg changed under stall!");
                $display("  prev=%h", hold_out_prev);
                $display("  now =%h", out_vec_dbg);
                $finish;
              end
              if (alpha_fp32 !== hold_a_prev) begin
                $display("[TB][FAIL] alpha_fp32 changed under stall!");
                $display("  prev=%h now=%h", hold_a_prev, alpha_fp32);
                $finish;
              end
            end
            hold_out_prev <= out_vec_dbg;
            hold_a_prev   <= alpha_fp32;
            hold_v        <= 1'b1;
          end else begin
            hold_v <= 1'b0;
          end

          // consume
          if (out_valid && out_ready) begin
            sb_pop(exp_out, exp_a);

            if (out_vec_dbg !== exp_out) begin
              $display("[TB][FAIL] out_vec mismatch!");
              for (d = 0; d < D; d = d + 1) begin
                $display("  d=%0d exp=%h got=%h exp_r=%f got_r=%f",
                  d,
                  exp_out[d*32 +: 32],
                  out_vec_dbg[d*32 +: 32],
                  fp32_to_real(exp_out[d*32 +: 32]),
                  fp32_to_real(out_vec_dbg[d*32 +: 32])
                );
              end
              $finish;
            end

            if (alpha_fp32 !== exp_a) begin
              er = fp32_to_real(exp_a);
              ar = fp32_to_real(alpha_fp32);
              $display("[TB][FAIL] alpha mismatch! exp=%h (%f) got=%h (%f)",
                       exp_a, er, alpha_fp32, ar);
              $finish;
            end

            if (rx < PRINT_N) begin
              $display("------------------------------------------------------------");
              $display("[TB] RX #%0d time=%0t", rx, $time);
              $display("  alpha=%f (bits=%h)", fp32_to_real(alpha_fp32), alpha_fp32);
              for (d = 0; d < D; d = d + 1)
                $display("  out[%0d]=%f", d, fp32_to_real(out_vec_dbg[d*32 +: 32]));
            end

            rx = rx + 1;
          end

          // stop when drained
          if ((tx >= TX_GOAL) && (sb_cnt == 0) && !(in_valid && !in_ready))
            idle = idle + 1;
          else
            idle = 0;

          if (idle >= 40) begin
            $display("[TB] DONE");
            $finish;
          end
        end
      end
    end
  endtask

endmodule

`default_nettype wire
