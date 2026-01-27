// ============================================================================
// tb_att_score_dump_matrix.sv
// 目的：
//  1) 寫入 Q/K
//  2) 啟動 attention_score_top_with_fsm (經由 attention_presoft_submax_top wrapper)
//  3) 掃描 score SRAM 讀出整個 T×T score matrix
//  4) 以 hex + real 形式印出矩陣
//
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_dump.vvp ./test/tb_submax_core_top.sv
//   vvp ./vvp/tb_att_dump.vvp
// ============================================================================
`include "./src/EPU/attention_score/submax_core_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attention_presoft_submax_top;

  // ----------------------------
  // Small debug params
  // ----------------------------
  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned TR_M   = 2;
  localparam int unsigned CONFLICT_POLICY_C = 1;

  localparam int unsigned T_W = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W = (DMAX<=1)?1:$clog2(DMAX);

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  logic clk;
  logic rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic        fsm_start;
  logic [15:0] D_len;
  logic        fsm_busy;
  logic        fsm_done;

  // Q write
  logic              cpu_q_we;
  logic [T_W-1:0]    cpu_q_t;
  logic [D_W-1:0]    cpu_q_d;
  logic [DATA_W-1:0] cpu_q_wdata;
  logic [BYTE_W-1:0] cpu_q_wmask;

  // K write
  logic              cpu_k_we;
  logic [31:0]       cpu_k_t;
  logic [31:0]       cpu_k_d;
  logic [DATA_W-1:0] cpu_k_wdata;

  // score read ports (TB debug read, DUT sweep時會覆蓋)
  logic              score_re;
  logic [T_W-1:0]    score_tq;
  logic [T_W-1:0]    score_tk;
  logic [DATA_W-1:0] score_rdata;
  logic              score_rvalid;

  // extra adder passthrough (unused here)
  logic        add_start;
  logic [31:0] add_a_bits, add_b_bits;
  logic        add_busy, add_done;
  logic [31:0] add_z_bits;

  // Y read
  logic              y_re;
  logic [T_W-1:0]    y_tq;
  logic [T_W-1:0]    y_tk;
  logic [DATA_W-1:0] y_rdata;
  logic              y_rvalid;

  logic              busy;
  logic              done;

  // ----------------------------
  // Instantiate DUT
  // ----------------------------
  attention_presoft_submax_top #(
    .T(T),
    .DMAX(DMAX),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .NB(NB),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .fsm_start(fsm_start),
    .D_len(D_len),
    .fsm_busy(fsm_busy),
    .fsm_done(fsm_done),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .score_re(score_re),
    .score_tq(score_tq),
    .score_tk(score_tk),
    .score_rdata(score_rdata),
    .score_rvalid(score_rvalid),

    .add_start(add_start),
    .add_a_bits(add_a_bits),
    .add_b_bits(add_b_bits),
    .add_busy(add_busy),
    .add_done(add_done),
    .add_z_bits(add_z_bits),

    .y_re(y_re),
    .y_tq(y_tq),
    .y_tk(y_tk),
    .y_rdata(y_rdata),
    .y_rvalid(y_rvalid),

    .busy(busy),
    .done(done)
  );

  // ==========================================================================
  // FP32 -> real conversion (your function)
  // ==========================================================================
  function automatic real fp32_to_real(input logic [31:0] b);
    int  s;
    int  e;
    int  frac;
    real mant;
    real val;
    begin
      s    = b[31];
      e    = b[30:23];
      frac = b[22:0];

      if (e == 0) begin
        if (frac == 0) begin
          val = 0.0;
        end else begin
          mant = frac / 8388608.0;
          val  = mant * (2.0 ** (-126));
        end
      end else if (e == 255) begin
        if (frac == 0) val = 1.0/0.0;
        else           val = 0.0/0.0;
      end else begin
        mant = 1.0 + (frac / 8388608.0);
        val  = mant * (2.0 ** (e - 127));
      end

      fp32_to_real = s ? -val : val;
    end
  endfunction

  // ordering key for fp32 compare
  function automatic logic [31:0] fp32_key(input logic [31:0] a);
    fp32_key = a[31] ? ~a : (a ^ 32'h8000_0000);
  endfunction

  function automatic bit fp32_gt(input logic [31:0] a, input logic [31:0] b);
    fp32_gt = (fp32_key(a) > fp32_key(b));
  endfunction

  function automatic bit fp32_le(input logic [31:0] a, input logic [31:0] b);
    fp32_le = (fp32_key(a) <= fp32_key(b));
  endfunction

  // ==========================================================================
  // Tasks
  // ==========================================================================
  task automatic write_q(input int tq, input int dd, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_q_we    <= 1'b1;
      cpu_q_t     <= tq[T_W-1:0];
      cpu_q_d     <= dd[D_W-1:0];
      cpu_q_wdata <= bits;
      cpu_q_wmask <= {BYTE_W{1'b1}};
      @(posedge clk);
      cpu_q_we    <= 1'b0;
    end
  endtask

  task automatic write_k(input int tk, input int dd, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_k_we    <= 1'b1;
      cpu_k_t     <= tk[31:0];
      cpu_k_d     <= dd[31:0];
      cpu_k_wdata <= bits;
      @(posedge clk);
      cpu_k_we    <= 1'b0;
    end
  endtask

  task automatic pulse_fsm_start;
    begin
      @(posedge clk);
      fsm_start <= 1'b1;
      @(posedge clk);
      fsm_start <= 1'b0;
    end
  endtask

  // TB debug read score (only use AFTER done, to avoid sweep override)
  task automatic read_score(input int tq, input int tk, output logic [31:0] bits);
    begin
      @(posedge clk);
      score_re <= 1'b1;
      score_tq <= tq[T_W-1:0];
      score_tk <= tk[T_W-1:0];
      @(posedge clk);
      score_re <= 1'b0;
      while (!score_rvalid) @(posedge clk);
      bits = score_rdata;
    end
  endtask

  // read Y (1-cycle latency)
  task automatic read_y(input int tq, input int tk, output logic [31:0] bits);
    begin
      @(posedge clk);
      y_re <= 1'b1;
      y_tq <= tq[T_W-1:0];
      y_tk <= tk[T_W-1:0];
      @(posedge clk);
      y_re <= 1'b0;
      while (!y_rvalid) @(posedge clk);
      bits = y_rdata;
    end
  endtask

  // ==========================================================================
  // Storage
  // ==========================================================================
  logic [31:0] X [0:T-1][0:T-1];
  logic [31:0] Y [0:T-1][0:T-1];
  logic [31:0] RowMaxTB [0:T-1];

  // ==========================================================================
  // Main
  // ==========================================================================
  bit pass;

  initial begin
    // defaults
    fsm_start   = 1'b0;
    D_len       = 16'd4;

    cpu_q_we    = 1'b0;
    cpu_q_t     = '0;
    cpu_q_d     = '0;
    cpu_q_wdata = '0;
    cpu_q_wmask = '0;

    cpu_k_we    = 1'b0;
    cpu_k_t     = '0;
    cpu_k_d     = '0;
    cpu_k_wdata = '0;

    score_re    = 1'b0;
    score_tq    = '0;
    score_tk    = '0;

    add_start   = 1'b0;
    add_a_bits  = '0;
    add_b_bits  = '0;

    y_re        = 1'b0;
    y_tq        = '0;
    y_tk        = '0;

    pass = 1'b1;

    wait (rst_n === 1'b1);
    repeat (5) @(posedge clk);

    // ------------------------------------------------------------
    // Load Q & K (deterministic pattern)
    // ------------------------------------------------------------
    for (int t=0; t<T; t++) begin
      for (int d=0; d<4; d++) begin
        logic [31:0] qbits;
        case (t)
          0: qbits = 32'h3F800000; // 1.0
          1: qbits = 32'h40000000; // 2.0
          2: qbits = 32'h40400000; // 3.0
          default: qbits = 32'h40800000; // 4.0
        endcase
        qbits = qbits + (d * 32'h00010000);
        write_q(t, d, qbits);
      end
    end

    for (int t=0; t<T; t++) begin
      for (int d=0; d<4; d++) begin
        logic [31:0] kbits;
        case (t)
          0: kbits = 32'h3F000000; // 0.5
          1: kbits = 32'h3F800000; // 1.0
          2: kbits = 32'h40000000; // 2.0
          default: kbits = 32'h40400000; // 3.0
        endcase
        kbits = kbits + (d * 32'h00008000);
        write_k(t, d, kbits);
      end
    end

    // ------------------------------------------------------------
    // Run
    // ------------------------------------------------------------
    $display("[TB] pulse fsm_start...");
    pulse_fsm_start();

    while (!fsm_done) @(posedge clk);
    $display("[TB] fsm_done asserted (QKT done).");

    while (!done) @(posedge clk);
    $display("[TB] done asserted (submax done).");

    // ------------------------------------------------------------
    // Read X (score) and Y (submax)
    // Read X only after done to avoid sweep overriding score ports
    // ------------------------------------------------------------
    for (int tq=0; tq<T; tq++) begin
      for (int tk=0; tk<T; tk++) begin
        logic [31:0] xb, yb;
        read_score(tq, tk, xb);
        read_y(tq, tk, yb);
        X[tq][tk] = xb;
        Y[tq][tk] = yb;
      end
    end

    // ------------------------------------------------------------
    // Compute RowMaxTB from X
    // ------------------------------------------------------------
    for (int tq=0; tq<T; tq++) begin
      logic [31:0] m;
      m = X[tq][0];
      for (int tk=1; tk<T; tk++) begin
        if (fp32_gt(X[tq][tk], m)) m = X[tq][tk];
      end
      RowMaxTB[tq] = m;
    end

    // ------------------------------------------------------------
    // Print matrices
    // ------------------------------------------------------------
    $display("\n==== SCORE MATRIX X (hex) ====");
    for (int tq=0; tq<T; tq++) begin
      $write("row %0d :", tq);
      for (int tk=0; tk<T; tk++) $write(" %08x", X[tq][tk]);
      $write("\n");
    end

    $display("\n==== SCORE MATRIX X (real) ====");
    for (int tq=0; tq<T; tq++) begin
      $write("row %0d :", tq);
      for (int tk=0; tk<T; tk++) $write(" %0.6f", fp32_to_real(X[tq][tk]));
      $write("\n");
    end

    $display("\n==== RowMax (from X) ====");
    for (int tq=0; tq<T; tq++) begin
      $display("row %0d max = %0.6f (hex=%08x)", tq, fp32_to_real(RowMaxTB[tq]), RowMaxTB[tq]);
    end

    $display("\n==== SUBMAX MATRIX Y = X - RowMax(row) (hex) ====");
    for (int tq=0; tq<T; tq++) begin
      $write("row %0d :", tq);
      for (int tk=0; tk<T; tk++) $write(" %08x", Y[tq][tk]);
      $write("\n");
    end

    $display("\n==== SUBMAX MATRIX Y (real) ====");
    for (int tq=0; tq<T; tq++) begin
      $write("row %0d :", tq);
      for (int tk=0; tk<T; tk++) $write(" %0.6f", fp32_to_real(Y[tq][tk]));
      $write("\n");
    end

    // ------------------------------------------------------------
    // Checks:
    //  1) max(Y[row][:]) == 0 (+0 or -0)
    //  2) all Y <= 0
    // ------------------------------------------------------------
    for (int tq=0; tq<T; tq++) begin
      logic [31:0] row_max_y;
      row_max_y = Y[tq][0];
      for (int tk=1; tk<T; tk++) begin
        if (fp32_gt(Y[tq][tk], row_max_y)) row_max_y = Y[tq][tk];
      end

      if (!((row_max_y == 32'h00000000) || (row_max_y == 32'h80000000))) begin
        $display("[FAIL] row %0d: max(Y) != 0 : %08x (%0.6f)",
                 tq, row_max_y, fp32_to_real(row_max_y));
        pass = 1'b0;
      end else begin
        $display("[OK] row %0d: max(Y) == 0", tq);
      end

      for (int tk=0; tk<T; tk++) begin
        if (!fp32_le(Y[tq][tk], 32'h00000000)) begin
          $display("[FAIL] Y[%0d][%0d] > 0 : %08x (%0.6f)",
                   tq, tk, Y[tq][tk], fp32_to_real(Y[tq][tk]));
          pass = 1'b0;
        end
      end
    end

    if (pass) begin
      $display("\n==============================");
      $display("           PASS               ");
      $display("==============================\n");
    end else begin
      $display("\n==============================");
      $display("           FAIL               ");
      $display("==============================\n");
    end

    repeat (20) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
