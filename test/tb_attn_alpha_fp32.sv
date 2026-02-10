// ============================================================
// tb_attn_alpha_fp32.sv  (IVERILOG-SAFE / NO real arrays)
// - Test attn_alpha_fp32
// - Golden depends on MODE:
//   MODE=0: sigmoid_approx(out_vec[0])   (same approx as DUT)
//   MODE=1: clamp(out_vec[0], 0..1)
//   MODE=2: clamp(SCALE*out_vec[0]+BIAS, 0..1)
// - Checks:
//   1) alpha_fp32 matches golden
//   2) hold under stall: out_valid && !out_ready => alpha_fp32 stable
// ============================================================
/*
iverilog -g2012 -Wall \
  ./test/tb_attn_alpha_fp32.sv \
  -o ./vvp/tb_m4

vvp ./vvp/tb_m4
*/
`include "./src/AMOLED/atten_core/attn_alpha_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attn_alpha_fp32;

  // ----------------------------------------------------------
  // Parameters (edit here)
  // ----------------------------------------------------------
  localparam int unsigned D           = 8;
  localparam int unsigned PIPE_STAGES = 1;

  // pick one: 0=sigmoid, 1=clamp, 2=linear+clamp
  localparam int unsigned MODE  = 1;

  // used only in MODE=2
  localparam real SCALE = 1.0;
  localparam real BIAS  = 0.0;

  // used only in MODE=0
  localparam real SIG_A = 1.0;

  localparam int TX_GOAL = 200;
  localparam int PRINT_N = 8;

  // ----------------------------------------------------------
  // Clock / Reset
  // ----------------------------------------------------------
  reg clk, rst_n;
  initial begin clk=0; forever #5 clk=~clk; end
  initial begin rst_n=0; repeat(6) @(posedge clk); rst_n=1; end

  // ----------------------------------------------------------
  // DUT I/O
  // ----------------------------------------------------------
  reg                  in_valid;
  wire                 in_ready;
  reg  [D*32-1:0]       out_vec;

  wire                 out_valid;
  reg                  out_ready;
  wire [31:0]           alpha_fp32;

  // <<< compile these RTLs in your iverilog command >>>
  attn_alpha_fp32 #(
    .D(D),
    .PIPE_STAGES(PIPE_STAGES),
    .MODE(MODE),
    .SCALE(SCALE),
    .BIAS(BIAS),
    .SIG_A(SIG_A)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .in_ready(in_ready),
    .out_vec(out_vec),
    .out_valid(out_valid),
    .out_ready(out_ready),
    .alpha_fp32(alpha_fp32)
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
      sign=f[31]; exp=f[30:23]; mant=f[22:0];
      if (exp==0) begin
        if (mant==0) val=0.0;
        else val=(mant/8388608.0)*pow2i(-126);
      end else if (exp==255) begin
        val=(mant==0)?1.0e30:0.0;
      end else begin
        frac=1.0+(mant/8388608.0);
        val=frac*pow2i(exp-127);
      end
      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign; real a,v; integer e, exp_i, mant_i; real frac; reg [31:0] out;
    begin
      sign=(r<0.0);
      a=sign?-r:r;
      out=32'h0;
      if (a==0.0) out=32'h0;
      else begin
        v=a; e=0;
        while (v>=2.0) begin v=v/2.0; e=e+1; end
        while (v< 1.0) begin v=v*2.0; e=e-1; end
        exp_i=e+127;
        frac=v-1.0;
        mant_i=integer'(frac*8388608.0+0.5);
        if (mant_i>=8388608) begin mant_i=0; exp_i=exp_i+1; end
        out={sign,exp_i[7:0],mant_i[22:0]};
      end
      real_to_fp32=out;
    end
  endfunction

  // ----------------------------------------------------------
  // clamp / exp2 approx / sigmoid approx (same as DUT)
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
        z = (-SIG_A * x) * LOG2E;
        e = exp2_approx(z);
        sigmoid_approx = 1.0 / (1.0 + e);
      end
    end
  endfunction

  // ----------------------------------------------------------
  // Golden alpha (based on MODE)
  // ----------------------------------------------------------
  function [31:0] golden_alpha(input [31:0] out0_bits);
    real x, a;
    begin
      x = fp32_to_real(out0_bits);
      if (MODE == 0)      a = sigmoid_approx(x);
      else if (MODE == 1) a = clamp01(x);
      else                a = clamp01(SCALE*x + BIAS);
      golden_alpha = real_to_fp32(a);
    end
  endfunction

  // ----------------------------------------------------------
  // Scoreboard (expected alpha)
  // ----------------------------------------------------------
  localparam int MAXQ = 1024;
  reg [31:0] exp_a_q [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [31:0] a);
    begin
      exp_a_q[sb_wr] = a;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [31:0] a);
    begin
      a = exp_a_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // ----------------------------------------------------------
  // Hold-under-stall check
  // ----------------------------------------------------------
  reg [31:0] hold_prev;
  reg        hold_v;

  // ----------------------------------------------------------
  // Run control
  // ----------------------------------------------------------
  integer tx, rx, idle;
  reg [31:0] exp_tmp;

  // random out0 generator (covers negatives / >1)
  function [31:0] rand_out0();
    integer r;
    real x;
    begin
      r = ($urandom()%2401) - 1200;  // [-1200..1200]
      x = r / 200.0;                 // [-6.0..6.0]
      rand_out0 = real_to_fp32(x);
    end
  endfunction

  function [31:0] rand_fp32_small();
    integer r; real x;
    begin
      r = ($urandom()%401) - 200;    // [-200..200]
      x = r / 100.0;                 // [-2..2]
      rand_fp32_small = real_to_fp32(x);
    end
  endfunction

  initial begin
    in_valid  = 1'b0;
    out_ready = 1'b1;
    out_vec   = '0;

    sb_wr=0; sb_rd=0; sb_cnt=0;
    tx=0; rx=0; idle=0;
    hold_prev=32'h0; hold_v=1'b0;

    $dumpfile("tb_attn_alpha_fp32.vcd");
    $dumpvars(0, tb_attn_alpha_fp32);

    @(posedge rst_n);
    repeat(3) @(posedge clk);

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
  // ----------------------------------------------------------
  task drive_in();
    integer d;
    begin
      while (tx < TX_GOAL) begin
        @(negedge clk);

        // if stalled on input, hold out_vec + in_valid
        if (!(in_valid && !in_ready)) begin
          if (in_ready && (($urandom()%100) < 70)) begin
            out_vec[0*32 +: 32] <= rand_out0();
            for (d=1; d<D; d=d+1)
              out_vec[d*32 +: 32] <= rand_fp32_small();
            in_valid <= 1'b1;
          end else begin
            in_valid <= 1'b0;
          end
        end

        @(posedge clk);
        if (rst_n && in_valid && in_ready) begin
          exp_tmp = golden_alpha(out_vec[0*32 +: 32]);
          sb_push(exp_tmp);
          tx = tx + 1;
        end
      end

      // drain
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
    reg [31:0] exp_a;
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
            if (hold_v && (alpha_fp32 !== hold_prev)) begin
              $display("[TB][FAIL] alpha changed under stall!");
              $display("  prev=%h now=%h", hold_prev, alpha_fp32);
              $finish;
            end
            hold_prev <= alpha_fp32;
            hold_v    <= 1'b1;
          end else begin
            hold_v <= 1'b0;
          end

          // consume
          if (out_valid && out_ready) begin
            sb_pop(exp_a);

            if (alpha_fp32 !== exp_a) begin
              er = fp32_to_real(exp_a);
              ar = fp32_to_real(alpha_fp32);
              $display("[TB][FAIL] alpha mismatch! exp=%h (%f) got=%h (%f)",
                       exp_a, er, alpha_fp32, ar);
              $finish;
            end

            if (rx < PRINT_N) begin
              ar = fp32_to_real(alpha_fp32);
              $display("RX %0d: alpha=%f (bits=%h)", rx, ar, alpha_fp32);
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
