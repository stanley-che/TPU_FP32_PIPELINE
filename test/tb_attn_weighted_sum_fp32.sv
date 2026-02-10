// ============================================================
// tb_attn_weighted_sum_fp32.sv  (IVERILOG-SAFE / NO real arrays)
// - Test attn_weighted_sum_fp32
// - Golden: out[d] = sum_t w[t] * V[t][d]
// - Checks:
//   1) out_vec matches golden
//   2) hold under stall
// ============================================================
/*
iverilog -g2012 -Wall \
  ./test/tb_attn_weighted_sum_fp32.sv \
  -o ./vvp/tb_m3

vvp ./vvp/tb_m3
gtkwave tb_attn_weighted_sum_fp32.vcd

*/
`include "./src/AMOLED/atten_core/attn_weighted_sum_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attn_weighted_sum_fp32;

  // ----------------------------------------------------------
  // Parameters
  // ----------------------------------------------------------
  localparam int unsigned TOKENS = 9;
  localparam int unsigned D      = 8;
  localparam int unsigned PIPE_STAGES = 1;

  localparam int unsigned W_W   = TOKENS*32;
  localparam int unsigned V_W   = TOKENS*D*32;
  localparam int unsigned OUT_W = D*32;

  localparam int TX_GOAL = 200;
  localparam int PRINT_N = 5;

  // ----------------------------------------------------------
  // Clock / Reset
  // ----------------------------------------------------------
  reg clk, rst_n;
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

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
  wire [OUT_W-1:0]     out_vec;

  // <<< 這裡假設你 RTL 檔是已經另外編譯進來 >>>
  // module name: attn_weighted_sum_fp32
  attn_weighted_sum_fp32 #(
    .TOKENS      (TOKENS),
    .D           (D),
    .PIPE_STAGES (PIPE_STAGES)
  ) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .in_ready (in_ready),
    .w_flat   (w_flat),
    .v_vecs   (v_vecs),
    .out_valid(out_valid),
    .out_ready(out_ready),
    .out_vec  (out_vec)
  );

  // ----------------------------------------------------------
  // pow2i + FP32 pack/unpack (IVERILOG-SAFE)
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
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end

        exp_i = e + 127;

        frac   = v - 1.0;
        mant_i = integer'(frac * 8388608.0 + 0.5);
        if (mant_i >= 8388608) begin
          mant_i = 0;
          exp_i  = exp_i + 1;
        end
        out = {sign, exp_i[7:0], mant_i[22:0]};
      end

      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // Random small fp32
  // ----------------------------------------------------------
  function [31:0] rand_fp32_small();
    integer r;
    real x;
    begin
      r = ($urandom() % 401) - 200;   // [-200..200]
      x = r / 100.0;                  // [-2.0..2.0]
      rand_fp32_small = real_to_fp32(x);
    end
  endfunction

  // ----------------------------------------------------------
  // Golden: out[d] = sum_t w[t] * v[t][d]
  // ----------------------------------------------------------
  function [OUT_W-1:0] golden_out(input [W_W-1:0] w_in, input [V_W-1:0] v_in);
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
      golden_out = tmp;
    end
  endfunction

  // ----------------------------------------------------------
  // Scoreboard (expected out_vec)
  // ----------------------------------------------------------
  localparam int MAXQ = 1024;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [OUT_W-1:0] o);
    begin
      exp_out_q[sb_wr] = o;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o);
    begin
      o = exp_out_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // ----------------------------------------------------------
  // Hold-under-stall check
  // ----------------------------------------------------------
  reg [OUT_W-1:0] hold_prev;
  reg             hold_v;

  // ----------------------------------------------------------
  // Shared temps (avoid task-local arrays)
  // ----------------------------------------------------------
  integer w_int [0:TOKENS-1];   // integer array (Icarus OK)
  real    sum_w;
  real    wi;
  reg [OUT_W-1:0] exp_tmp;

  // ----------------------------------------------------------
  // Run control
  // ----------------------------------------------------------
  integer tx, rx, idle;

  initial begin
    in_valid   = 1'b0;
    out_ready  = 1'b1;
    w_flat     = '0;
    v_vecs     = '0;

    sb_wr=0; sb_rd=0; sb_cnt=0;
    tx=0; rx=0; idle=0;
    hold_prev='0; hold_v=1'b0;

    $dumpfile("tb_attn_weighted_sum_fp32.vcd");
    $dumpvars(0, tb_attn_weighted_sum_fp32);

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
  // Drive input
  // - weights: generate non-negative integers, normalize with real
  // ----------------------------------------------------------
  task drive_in();
    integer t, d;
    begin
      while (tx < TX_GOAL) begin
        @(negedge clk);

        if (in_ready && (($urandom()%100) < 70)) begin
          // random V
          for (t = 0; t < TOKENS; t = t + 1)
            for (d = 0; d < D; d = d + 1)
              v_vecs[(t*D + d)*32 +: 32] <= rand_fp32_small();

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

        @(posedge clk);
        if (rst_n && in_valid && in_ready) begin
          exp_tmp = golden_out(w_flat, v_vecs);
          sb_push(exp_tmp);
          tx = tx + 1;
        end
      end

      in_valid <= 1'b0;
    end
  endtask

  // ----------------------------------------------------------
  // Random backpressure on output
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
    integer d;
    begin
      forever begin
        @(posedge clk);

        if (!rst_n) begin
          hold_v <= 1'b0;
          idle   <= 0;
        end else begin
          // hold check
          if (out_valid && !out_ready) begin
            if (hold_v && (out_vec !== hold_prev)) begin
              $display("[TB][FAIL] out_vec changed under stall!");
              $display("  prev=%h", hold_prev);
              $display("  now =%h", out_vec);
              $finish;
            end
            hold_prev <= out_vec;
            hold_v    <= 1'b1;
          end else begin
            hold_v <= 1'b0;
          end

          // consume
          if (out_valid && out_ready) begin
            sb_pop(exp_out);

            if (out_vec !== exp_out) begin
              $display("[TB][FAIL] out_vec mismatch!");
              for (d = 0; d < D; d = d + 1) begin
                $display("  d=%0d exp=%h got=%h exp_r=%f got_r=%f",
                  d,
                  exp_out[d*32 +: 32],
                  out_vec[d*32 +: 32],
                  fp32_to_real(exp_out[d*32 +: 32]),
                  fp32_to_real(out_vec[d*32 +: 32])
                );
              end
              $finish;
            end

            if (rx < PRINT_N) begin
              $display("------------------------------------------------------------");
              $display("[TB] RX #%0d time=%0t", rx, $time);
              for (d = 0; d < D; d = d + 1)
                $display("  out[%0d]=%f", d, fp32_to_real(out_vec[d*32 +: 32]));
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
