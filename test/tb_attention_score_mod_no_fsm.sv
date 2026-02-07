// tb_attention_score_mod_no_fsm_case2.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_att_mod_case2.vvp \
  ./test/tb_attention_score_mod_no_fsm.sv

vvp ./vvp/tb_att_mod_case2.vvp
gtkwave ./vvp/tb_att_mod_case2.vcd
*/

`include "./src/EPU/attention_score/attention_score_Mod.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attention_score_mod_no_fsm_case2;

  // ----------------------------
  // Params
  // ----------------------------
  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned D_LEN  = 4;   // <<< CASE2 uses D=4
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned TR_M   = 2;

  localparam int unsigned T_W = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W = (DMAX<=1)?1:$clog2(DMAX);

  // ----------------------------
  // clock / reset
  // ----------------------------
  logic clk;
  logic rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (6) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic        tr_start;
  logic        tr_busy, tr_done;

  logic        gemm_start;
  logic        gemm_busy, gemm_done;

  logic [15:0] D_len;

  logic        busy, done, C_valid;

  // Q -> W
  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  // K -> transpose A
  logic                 cpu_k_we;
  logic [31:0]          cpu_k_t;
  logic [31:0]          cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  // read K^T from transpose B
  logic                 tr_b_re;
  logic [31:0]          tr_b_row;
  logic [31:0]          tr_b_col;
  logic [DATA_W-1:0]    tr_b_rdata;
  logic                 tr_b_rvalid;

  // X write (load K^T)
  logic                 cpu_x_we;
  logic [D_W-1:0]       cpu_x_k;
  logic [T_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  // Score read (C SRAM)
  logic                 score_re;
  logic [T_W-1:0]       score_tq;
  logic [T_W-1:0]       score_tk;
  logic [DATA_W-1:0]    score_rdata;
  logic                 score_rvalid;

  // ----------------------------
  // DUT
  // ----------------------------
  attention_score_Mod #(
    .T(T),
    .DMAX(DMAX),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .NB(NB),
    .BYTE_W(BYTE_W),
    .TR_M(TR_M)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .tr_start(tr_start),
    .tr_busy(tr_busy),
    .tr_done(tr_done),

    .gemm_start(gemm_start),
    .gemm_busy(gemm_busy),
    .gemm_done(gemm_done),

    .D_len(D_len),

    .busy(busy),
    .done(done),
    .C_valid(C_valid),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .tr_b_re(tr_b_re),
    .tr_b_row(tr_b_row),
    .tr_b_col(tr_b_col),
    .tr_b_rdata(tr_b_rdata),
    .tr_b_rvalid(tr_b_rvalid),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .score_re(score_re),
    .score_tq(score_tq),
    .score_tk(score_tk),
    .score_rdata(score_rdata),
    .score_rvalid(score_rvalid)
  );

  // ----------------------------
  // FP32 bit matrices
  // ----------------------------
  logic [31:0] Q_fp  [0:T-1][0:DMAX-1]; // Q[t][d]
  logic [31:0] K_fp  [0:T-1][0:DMAX-1]; // K[t][d]
  logic [31:0] KT_fp [0:DMAX-1][0:T-1]; // K^T[d][t] readback
  logic [31:0] S_rd  [0:T-1][0:T-1];    // Score[tq][tk] readback

  // ----------------------------
  // init signals
  // ----------------------------
  task automatic init_signals();
    tr_start   = 1'b0;
    gemm_start = 1'b0;
    D_len      = D_LEN[15:0];

    cpu_q_we    = 1'b0;
    cpu_q_t     = '0;
    cpu_q_d     = '0;
    cpu_q_wdata = '0;
    cpu_q_wmask = '0;

    cpu_k_we    = 1'b0;
    cpu_k_t     = '0;
    cpu_k_d     = '0;
    cpu_k_wdata = '0;

    tr_b_re     = 1'b0;
    tr_b_row    = '0;
    tr_b_col    = '0;

    cpu_x_we    = 1'b0;
    cpu_x_k     = '0;
    cpu_x_n     = '0;
    cpu_x_wdata = '0;
    cpu_x_wmask = '0;

    score_re    = 1'b0;
    score_tq    = '0;
    score_tk    = '0;
  endtask

  // ----------------------------
  // write Q (W SRAM)
  // ----------------------------
  task automatic write_Q(input int t, input int d, input logic [31:0] bits);
    @(posedge clk);
    cpu_q_we    <= 1'b1;
    cpu_q_t     <= t[T_W-1:0];
    cpu_q_d     <= d[D_W-1:0];
    cpu_q_wdata <= bits;
    cpu_q_wmask <= 4'b1111;
    @(posedge clk);
    cpu_q_we    <= 1'b0;
    cpu_q_wmask <= '0;
  endtask

  // ----------------------------
  // write K into transpose A
  // ----------------------------
  task automatic write_K(input int t, input int d, input logic [31:0] bits);
    @(posedge clk);
    cpu_k_we    <= 1'b1;
    cpu_k_t     <= t[31:0];
    cpu_k_d     <= d[31:0];
    cpu_k_wdata <= bits;
    @(posedge clk);
    cpu_k_we    <= 1'b0;
  endtask

  // ----------------------------
  // run transpose
  // ----------------------------
  task automatic run_transpose();
    @(posedge clk);
    tr_start <= 1'b1;
    @(posedge clk);
    tr_start <= 1'b0;

    wait (tr_busy == 1'b1);
    wait (tr_done == 1'b1);
    @(posedge clk);
  endtask

  // ----------------------------
  // read K^T from transpose B: row=d, col=t
  // ----------------------------
  task automatic read_KT_bits(input int d, input int t, output logic [31:0] bits);
    @(posedge clk);
    tr_b_re  <= 1'b1;
    tr_b_row <= d[31:0];
    tr_b_col <= t[31:0];
    @(posedge clk);
    tr_b_re  <= 1'b0;

    wait (tr_b_rvalid == 1'b1);
    bits = tr_b_rdata;
    @(posedge clk);
  endtask

  // ----------------------------
  // write X[k][n] = K^T[k][n]
  // ----------------------------
  task automatic write_X_bits(input int k, input int n, input logic [31:0] bits);
    @(posedge clk);
    cpu_x_we    <= 1'b1;
    cpu_x_k     <= k[D_W-1:0];
    cpu_x_n     <= n[T_W-1:0];
    cpu_x_wdata <= bits;
    cpu_x_wmask <= 4'b1111;
    @(posedge clk);
    cpu_x_we    <= 1'b0;
    cpu_x_wmask <= '0;
  endtask

  // ----------------------------
  // run gemm
  // ----------------------------
  task automatic run_gemm();
    @(posedge clk);
    gemm_start <= 1'b1;

    while (gemm_busy !== 1'b1) begin
      @(posedge clk);
    end

    gemm_start <= 1'b0;

    while (gemm_done !== 1'b1) begin
      @(posedge clk);
    end

    repeat (2) @(posedge clk);
  endtask

  // ----------------------------
  // read Score[tq][tk]
  // ----------------------------
  task automatic read_S_bits(input int tq, input int tk, output logic [31:0] bits);
    @(posedge clk);
    score_re <= 1'b1;
    score_tq <= tq[T_W-1:0];
    score_tk <= tk[T_W-1:0];
    @(posedge clk);
    score_re <= 1'b0;

    wait (score_rvalid == 1'b1);
    bits = score_rdata;
    @(posedge clk);
  endtask

  // ------------------------------------------------------------
  // FP32 (IEEE754) bits -> real   (iverilog-safe, no shortreal)
  // ------------------------------------------------------------
  function automatic real fp32_to_real(input logic [31:0] b);
    int s;
    int e;
    int frac;
    real mant;
    real val;
    begin
      s    = b[31];
      e    = b[30:23];
      frac = b[22:0];

      if (e == 0) begin
        if (frac == 0) val = 0.0;
        else begin
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

      if (s) fp32_to_real = -val;
      else   fp32_to_real =  val;
    end
  endfunction

  task automatic print_Q_hex();
    int i,j;
    real f;
    $display("\n==== Q (hex/float) T x D_LEN=%0d ====", D_LEN);
    for (i=0;i<T;i++) begin
      $write("Q[%0d]: ", i);
      for (j=0;j<D_LEN;j++) begin
        f = fp32_to_real(Q_fp[i][j]);
        $write("%08x(%0.4f) ", Q_fp[i][j], f);
      end
      $write("\n");
    end
  endtask

  task automatic print_K_hex();
    int i,j;
    real f;
    $display("\n==== K (hex/float) T x D_LEN=%0d ====", D_LEN);
    for (i=0;i<T;i++) begin
      $write("K[%0d]: ", i);
      for (j=0;j<D_LEN;j++) begin
        f = fp32_to_real(K_fp[i][j]);
        $write("%08x(%0.4f) ", K_fp[i][j], f);
      end
      $write("\n");
    end
  endtask

  task automatic print_KT_hex();
    int i,j;
    real f;
    $display("\n==== K^T (hex/float) D_LEN x T ====");
    for (i=0;i<D_LEN;i++) begin
      $write("KT[%0d]: ", i);
      for (j=0;j<T;j++) begin
        f = fp32_to_real(KT_fp[i][j]);
        $write("%08x(%0.4f) ", KT_fp[i][j], f);
      end
      $write("\n");
    end
  endtask

  task automatic print_S_hex();
    int i,j;
    real f;
    $display("\n==== Score (hex/float) T x T ====");
    for (i=0;i<T;i++) begin
      $write("S[%0d]: ", i);
      for (j=0;j<T;j++) begin
        f = fp32_to_real(S_rd[i][j]);
        $write("%08x(%0.4f) ", S_rd[i][j], f);
      end
      $write("\n");
    end
  endtask

  // ----------------------------
  // main
  // ----------------------------
  int t,d,k,tk,tq,col;
  logic [31:0] tmp_bits;

  initial begin
    $dumpfile("./vvp/tb_att_mod_case2.vcd");
    $dumpvars(0, tb_attention_score_mod_no_fsm_case2);

    init_signals();
    wait (rst_n == 1'b1);
    repeat (2) @(posedge clk);

    // ----------------------------
    // CASE2 vectors (T=4, D_LEN=4)
    // Q:
    // row0: -2  -1.5 -1  -0.5
    // row1:  0   0.5  1   1.5
    // row2:  2  -2   -1.5 -1
    // row3: -0.5 0    0.5  1
    //
    // K:
    // row0: -1.5  0    1.5 -1.5
    // row1:  2   -1    0.5  2
    // row2:  1   -2   -0.5  1
    // row3:  0    1.5 -1.5  0
    // ----------------------------
    for (t=0;t<T;t++) for (d=0;d<DMAX;d++) begin
      Q_fp[t][d] = 32'h00000000;
      K_fp[t][d] = 32'h00000000;
    end

    // Q
    Q_fp[0][0]=32'hc0000000; Q_fp[0][1]=32'hbfc00000; Q_fp[0][2]=32'hbf800000; Q_fp[0][3]=32'hbf000000;
    Q_fp[1][0]=32'h00000000; Q_fp[1][1]=32'h3f000000; Q_fp[1][2]=32'h3f800000; Q_fp[1][3]=32'h3fc00000;
    Q_fp[2][0]=32'h40000000; Q_fp[2][1]=32'hc0000000; Q_fp[2][2]=32'hbfc00000; Q_fp[2][3]=32'hbf800000;
    Q_fp[3][0]=32'hbf000000; Q_fp[3][1]=32'h00000000; Q_fp[3][2]=32'h3f000000; Q_fp[3][3]=32'h3f800000;

    // K
    K_fp[0][0]=32'hbfc00000; K_fp[0][1]=32'h00000000; K_fp[0][2]=32'h3fc00000; K_fp[0][3]=32'hbfc00000;
    K_fp[1][0]=32'h40000000; K_fp[1][1]=32'hbf800000; K_fp[1][2]=32'h3f000000; K_fp[1][3]=32'h40000000;
    K_fp[2][0]=32'h3f800000; K_fp[2][1]=32'hc0000000; K_fp[2][2]=32'hbf000000; K_fp[2][3]=32'hbfc00000;
    K_fp[3][0]=32'hbfc00000; K_fp[3][1]=32'h40000000; K_fp[3][2]=32'hbfc00000; K_fp[3][3]=32'h00000000;

    print_Q_hex();
    print_K_hex();

    // ----------------------------
    // Write Q -> W
    // ----------------------------
    $display("\n[TB] Write Q -> W SRAM ...");
    for (t=0;t<T;t++) begin
      for (d=0;d<D_LEN;d++) begin
        write_Q(t, d, Q_fp[t][d]);
      end
    end

    // ----------------------------
    // Write K -> transpose A
    // ----------------------------
    $display("\n[TB] Write K -> Transpose A ...");
    for (t=0;t<T;t++) begin
      for (d=0;d<D_LEN;d++) begin
        write_K(t, d, K_fp[t][d]);
      end
    end

    // ----------------------------
    // Run transpose
    // ----------------------------
    $display("\n[TB] Run transpose ...");
    run_transpose();
    $display("[TB] Transpose done.");

    // ----------------------------
    // Read K^T and load X
    // X[k][tk] = K^T[k][tk]
    // ----------------------------
    $display("\n[TB] Read K^T from transpose B and load X ...");
    for (k=0;k<D_LEN;k++) begin
      for (tk=0;tk<T;tk++) begin
        read_KT_bits(k, tk, tmp_bits);
        KT_fp[k][tk] = tmp_bits;
        write_X_bits(k, tk, tmp_bits);
      end
    end
    print_KT_hex();

    // ----------------------------
    // Run GEMM
    // ----------------------------
    $display("\n[TB] Run GEMM ...");
    run_gemm();
    $display("[TB] GEMM done. C_valid=%0d", C_valid);

    while (C_valid !== 1'b1) begin
      @(posedge clk);
    end
    $display("[TB] C_valid asserted, start reading Score");

    // ----------------------------
    // Read Score
    // ----------------------------
    $display("\n[TB] Read Score ...");
    for (tq=0;tq<T;tq++) begin
      for (col=0;col<T;col++) begin
        read_S_bits(tq, col, tmp_bits);
        S_rd[tq][col] = tmp_bits;
      end
    end
    print_S_hex();

    // ----------------------------
    // Expected check (case2)
    // S =
    // [  2.25  -4.00   2.25  -1.00
    //   -0.75   3.00  -3.75   2.00
    //   -3.75   3.25   8.25  -8.50
    //    0.00   1.25  -2.25   1.25 ]
    // ----------------------------
    begin : CHECK_AUTO
  real expS [0:T-1][0:T-1];
  real got, diff;
  real eps;
  int tq_i, tk_i, d_i;

  eps = 1e-3;

  // auto expected: S[tq][tk] = sum_d Q[tq][d] * K[tk][d]
  for (tq_i=0; tq_i<T; tq_i++) begin
    for (tk_i=0; tk_i<T; tk_i++) begin
      expS[tq_i][tk_i] = 0.0;
      for (d_i=0; d_i<D_LEN; d_i++) begin
        expS[tq_i][tk_i] += fp32_to_real(Q_fp[tq_i][d_i]) * fp32_to_real(K_fp[tk_i][d_i]);
      end
    end
  end

  $display("\n==== CHECK auto expected S (eps=%0.6f) ====", eps);
  for (tq_i=0; tq_i<T; tq_i++) begin
    for (tk_i=0; tk_i<T; tk_i++) begin
      got  = fp32_to_real(S_rd[tq_i][tk_i]);
      diff = got - expS[tq_i][tk_i];
      if (diff < 0) diff = -diff;
      if (diff > eps) begin
        $display("[TB][FAIL] S[%0d][%0d] got=%0.6f exp=%0.6f (hex=%08x)",
                 tq_i, tk_i, got, expS[tq_i][tk_i], S_rd[tq_i][tk_i]);
        $finish;
      end
    end
  end
  $display("[TB][PASS] auto-check OK!");
end


    $display("\n[TB] DONE.");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
