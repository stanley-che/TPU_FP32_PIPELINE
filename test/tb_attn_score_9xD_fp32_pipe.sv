// ============================================================
// tb_attn_score_9xD_fp32_pipe.sv  (IVERILOG-SAFE)
// - No shortreal
// - Golden uses real + local fp32 pack/unpack (same style as DUT)
// - Random backpressure on out_ready
// - Scoreboard queue for expected outputs (accounts for elastic latency)
// - Checks:
//   1) Numerical match (fp32 bits) for each token score
//   2) Out hold under stall: when out_valid && !out_ready, score_flat stable
// ============================================================
`include "./src/AMOLED/atten_core/attn_score_9xD_fp32_pipe.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attn_score_9xD_fp32_pipe;

  // ----------------------------
  // Parameters (match DUT)
  // ----------------------------
  localparam int unsigned TOKENS      = 9;
  localparam int unsigned D           = 8;
  localparam int unsigned PIPE_STAGES = 2;  // >=1
  localparam int unsigned SCORE_W     = TOKENS*32;

  // ----------------------------
  // Clock/Reset
  // ----------------------------
  reg clk;
  reg rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100MHz
  end

  initial begin
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                    in_valid;
  wire                   in_ready;
  reg  [D*32-1:0]         q_vec;
  reg  [TOKENS*D*32-1:0]  k_vecs;

  wire                   out_valid;
  reg                    out_ready;
  wire [SCORE_W-1:0]      score_flat;

  // DUT
  attn_score_9xD_fp32 #(
    .TOKENS(TOKENS),
    .D(D),
    .PIPE_STAGES(PIPE_STAGES)
  ) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .in_valid  (in_valid),
    .in_ready  (in_ready),
    .q_vec     (q_vec),
    .k_vecs    (k_vecs),
    .out_valid (out_valid),
    .out_ready (out_ready),
    .score_flat(score_flat)
  );

  // ----------------------------
  // pow2i (iverilog-safe)
  // ----------------------------
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

  // ----------------------------
  // FP32 <-> real (same style)
  // ----------------------------
  function real fp32_to_real(input [31:0] f);
    reg sign;
    integer exp;
    integer mant;
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
        val = (mant == 0) ? 1.0e30 : 0.0; // clamp (inf/nan)
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
    integer e;
    integer exp_i;
    integer mant_i;
    real frac;
    reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;

      out = 32'h0000_0000;

      if (a == 0.0) begin
        out = 32'h0000_0000;
      end else if (a > 3.4e38) begin
        out = {sign, 8'hFF, 23'h0}; // Inf
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end

        exp_i = e + 127;

        if (exp_i <= 0) begin
          // subnormal
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

  // ----------------------------
  // Helpers: pack random "reasonable" fp32
  //   - avoid NaN/Inf
  //   - generate roughly in [-2, 2)
  // ----------------------------
  function [31:0] rand_fp32();
    integer s;
    integer e;
    integer m;
    reg [31:0] f;
    begin
      s = ($urandom() & 1);
      // exponent around 127 (1.x) and 126 (0.5~1) etc.
      e = 126 + ($urandom() % 3); // {126,127,128} -> ~[0.5,2)
      m = $urandom() & 23'h7FFFFF;
      f = {s[0], e[7:0], m[22:0]};
      rand_fp32 = f;
    end
  endfunction

  function automatic [31:0] get_q(input int unsigned di);
    get_q = q_vec[di*32 +: 32];
  endfunction

  function automatic [31:0] get_k(input int unsigned ti, input int unsigned di);
    int unsigned idx;
    begin
      idx = (ti*D + di);
      get_k = k_vecs[idx*32 +: 32];
    end
  endfunction

  // ----------------------------
  // Golden compute: score_flat expected
  // ----------------------------
  function automatic [SCORE_W-1:0] golden_score(
    input [D*32-1:0] q,
    input [TOKENS*D*32-1:0] k
  );
    integer t, d;
    real acc;
    reg [SCORE_W-1:0] tmp;
    reg [31:0] qf, kf;
    begin
      tmp = '0;
      for (t = 0; t < TOKENS; t = t + 1) begin
        acc = 0.0;
        for (d = 0; d < D; d = d + 1) begin
          qf = q[d*32 +: 32];
          kf = k[(t*D + d)*32 +: 32];
          acc = acc + fp32_to_real(qf) * fp32_to_real(kf);
        end
        tmp[t*32 +: 32] = real_to_fp32(acc);
      end
      golden_score = tmp;
    end
  endfunction

  // ----------------------------
  // Scoreboard queue
  // ----------------------------
  localparam int MAXQ = 1024;
  reg [SCORE_W-1:0] exp_q [0:MAXQ-1];
  integer q_wr, q_rd, q_count;

  task sb_push(input [SCORE_W-1:0] v);
    begin
      if (q_count >= MAXQ) begin
        $display("[TB][FATAL] Scoreboard overflow");
        $finish;
      end
      exp_q[q_wr] = v;
      q_wr = (q_wr + 1) % MAXQ;
      q_count = q_count + 1;
    end
  endtask

  task sb_pop(output [SCORE_W-1:0] v);
    begin
      if (q_count <= 0) begin
        $display("[TB][FATAL] Scoreboard underflow");
        $finish;
      end
      v = exp_q[q_rd];
      q_rd = (q_rd + 1) % MAXQ;
      q_count = q_count - 1;
    end
  endtask

  // ----------------------------
  // Hold-under-stall check
  // ----------------------------
  reg [SCORE_W-1:0] hold_prev;
  reg              hold_prev_valid;

  // ----------------------------
  // Drive stimulus
  // ----------------------------
  integer tx_sent, tx_goal;
  integer seed;
  reg done;
integer rx_cnt;
localparam int PRINT_N = 5;

  initial begin
    seed = 32'hC0FFEE11;
    void'($urandom(seed));

    in_valid = 1'b0;
    q_vec    = '0;
    k_vecs   = '0;
    done = 1'b0;
rx_cnt = 0;

    out_ready = 1'b1;

    q_wr = 0; q_rd = 0; q_count = 0;
    hold_prev = '0;
    hold_prev_valid = 1'b0;

    tx_sent = 0;
    tx_goal = 200;

    // VCD
    $dumpfile("tb_attn_score_9xD_fp32_pipe.vcd");
    $dumpvars(0, tb_attn_score_9xD_fp32_pipe);

    // wait reset release
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    // main run
    fork
  drive_in();
  drive_out_ready();
  monitor_and_check();
join_any

// 有一個 thread 結束就到這裡，直接砍掉其他 thread
disable fork;

$display("[TB] DONE");
$finish;

  end

 task automatic drive_in();
  integer t, d;
  reg [SCORE_W-1:0] exp;
  begin
    // reset 期間：確保不送
    in_valid <= 1'b0;

    while (tx_sent < tx_goal) begin
      // ============================
      // 1) 在 negedge 先準備下一筆輸入（避免 posedge race）
      // ============================
      @(negedge clk);

      if (!rst_n) begin
        in_valid <= 1'b0;
      end else begin
        // 如果上一筆還卡在 in_valid && !in_ready，就不要改資料（保持 upstream 合規）
        if (!(in_valid && !in_ready)) begin
          if (($urandom() % 100) < 60) begin
            // prepare a new transaction
            for (d = 0; d < D; d = d + 1) begin
              q_vec[d*32 +: 32] <= rand_fp32();
            end
            for (t = 0; t < TOKENS; t = t + 1) begin
              for (d = 0; d < D; d = d + 1) begin
                k_vecs[(t*D + d)*32 +: 32] <= rand_fp32();
              end
            end
            in_valid <= 1'b1;
          end else begin
            in_valid <= 1'b0;
          end
        end
      end

      // ============================
      // 2) 在 posedge 判斷 handshake，並用「穩定後的 q/k」算 golden 入隊
      // ============================
      @(posedge clk);
      if (!rst_n) begin
        // no-op
      end else if (in_valid && in_ready) begin
        exp = golden_score(q_vec, k_vecs);
        sb_push(exp);
        tx_sent = tx_sent + 1;
      end
    end

    // 收尾：如果還有 pending valid 被 backpressure，等它被吃掉
    while (in_valid && !in_ready) @(posedge clk);
    in_valid <= 1'b0;
  end
endtask


  task automatic drive_out_ready();
    begin
      forever begin
        @(posedge clk);
        if (!rst_n) begin
          out_ready <= 1'b0;
        end else begin
          // Random backpressure: 70% ready, 30% stall
          out_ready <= (($urandom() % 100) < 70);
        end
      end
    end
  endtask

  task automatic monitor_and_check();
    reg [SCORE_W-1:0] exp;
    integer t;
    begin
      forever begin
        @(posedge clk);
        if (!rst_n) begin
          hold_prev_valid <= 1'b0;
        end else begin
          // hold check: if currently stalling output, must be stable
          if (out_valid && !out_ready) begin
            if (hold_prev_valid) begin
              if (score_flat !== hold_prev) begin
                $display("[TB][FAIL] Output changed under stall!");
                $display("  prev=%h", hold_prev);
                $display("  now =%h", score_flat);
                $finish;
              end
            end
            hold_prev <= score_flat;
            hold_prev_valid <= 1'b1;
          end else begin
            hold_prev_valid <= 1'b0;
          end

          // consume + compare when handshake
          if (out_valid && out_ready) begin
  sb_pop(exp);

  // --- PRINT ---
  if (rx_cnt < PRINT_N) begin
    integer tt;
    $display("------------------------------------------------------------");
    $display("[TB] RX #%0d  (time=%0t)", rx_cnt, $time);

    // 印 Q 向量（bit + real）
    $display("[TB] Q:");
    for (tt = 0; tt < D; tt = tt + 1) begin
      $display("  q[%0d]=%h  (%f)", tt, q_vec[tt*32 +: 32], fp32_to_real(q_vec[tt*32 +: 32]));
    end

    // 印 K 向量（每個 token 的 D 維）
    $display("[TB] K:");
    for (tt = 0; tt < TOKENS; tt = tt + 1) begin
      integer dd;
      $display("  token %0d:", tt);
      for (dd = 0; dd < D; dd = dd + 1) begin
        $display("    k[%0d][%0d]=%h  (%f)",
          tt, dd,
          k_vecs[(tt*D + dd)*32 +: 32],
          fp32_to_real(k_vecs[(tt*D + dd)*32 +: 32])
        );
      end
    end

    // 印 score（golden vs dut）
    $display("[TB] SCORE:");
    for (tt = 0; tt < TOKENS; tt = tt + 1) begin
      $display("  t=%0d  exp=%h (%f)   got=%h (%f)",
        tt,
        exp[tt*32 +: 32],
        fp32_to_real(exp[tt*32 +: 32]),
        score_flat[tt*32 +: 32],
        fp32_to_real(score_flat[tt*32 +: 32])
      );
    end
  end

  rx_cnt = rx_cnt + 1;

  // --- CHECK ---
  if (score_flat !== exp) begin
    integer t;
    $display("[TB][FAIL] Mismatch at consume!");
    for (t = 0; t < TOKENS; t = t + 1) begin
      $display("  t=%0d exp=%h got=%h exp_r=%f got_r=%f",
        t,
        exp[t*32 +: 32],
        score_flat[t*32 +: 32],
        fp32_to_real(exp[t*32 +: 32]),
        fp32_to_real(score_flat[t*32 +: 32])
      );
    end
    $finish;
  end
end


          // termination condition: sent enough and drained scoreboard
          if ((tx_sent >= tx_goal) && (q_count == 0) && !(in_valid && !in_ready)) begin
            // give a couple cycles to settle then exit
            repeat (10) @(posedge clk);
            disable monitor_and_check;
          end
        end
      end
    end
  endtask

endmodule

`default_nettype wire
