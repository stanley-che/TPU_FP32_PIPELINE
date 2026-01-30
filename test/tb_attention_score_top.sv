// tb_attention_score_top_case2.sv
// ------------------------------------------------------------
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_case2.vvp ./test/tb_attention_score_top.sv
// vvp ./vvp/tb_att_case2.vvp
// ------------------------------------------------------------

`include "./src/EPU/attention_score/attention_score_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attention_score_with_fsm;

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

  logic clk, rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (6) @(posedge clk);
    rst_n = 1'b1;
  end

  // FSM control
  logic        fsm_start;
  logic [15:0] D_len;
  logic        fsm_busy, fsm_done;

  // CPU Q write
  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  // CPU K write
  logic                 cpu_k_we;
  logic [31:0]          cpu_k_t;
  logic [31:0]          cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  // CPU Score read
  logic                 score_re;
  logic [T_W-1:0]       score_tq;
  logic [T_W-1:0]       score_tk;
  logic [DATA_W-1:0]    score_rdata;
  logic                 score_rvalid;

  // Matrices
  logic [31:0] Q_fp [0:T-1][0:DMAX-1];
  logic [31:0] K_fp [0:T-1][0:DMAX-1];
  logic [31:0] S_rd [0:T-1][0:T-1];

  // DUT
  attention_score_top_with_fsm #(
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
    .score_rvalid(score_rvalid)
  );

  // init
  task automatic init_sig();
    fsm_start   = 1'b0;
    D_len       = D_LEN[15:0];

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
  endtask

  // write Q
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

  // write K
  task automatic write_K(input int t, input int d, input logic [31:0] bits);
    @(posedge clk);
    cpu_k_we    <= 1'b1;
    cpu_k_t     <= t[31:0];
    cpu_k_d     <= d[31:0];
    cpu_k_wdata <= bits;
    @(posedge clk);
    cpu_k_we    <= 1'b0;
  endtask

  // read score (with timeout) - already waits rvalid (good)
  task automatic read_S(input int tq, input int tk, output logic [31:0] bits);
    int timeout;
    timeout = 0;

    @(posedge clk);
    score_re <= 1'b1;
    score_tq <= tq[T_W-1:0];
    score_tk <= tk[T_W-1:0];
    @(posedge clk);
    score_re <= 1'b0;

    while (score_rvalid !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 2000) begin
        $display("[TB][ERR] score_rvalid timeout tq=%0d tk=%0d rdata=0x%08x",
                 tq, tk, score_rdata);
        $finish;
      end
    end

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

      if (s) fp32_to_real = -val;
      else   fp32_to_real =  val;
    end
  endfunction

  task automatic print_Q_dec();
    int i,j;
    real f;
    $display("\n==== Q (float) T x D_LEN=%0d ====", D_LEN);
    for (i=0;i<T;i++) begin
      $write("Q[%0d]: ", i);
      for (j=0;j<D_LEN;j++) begin
        f = fp32_to_real(Q_fp[i][j]);
        $write("%10.4f ", f);
      end
      $write("\n");
    end
  endtask

  task automatic print_K_dec();
    int i,j;
    real f;
    $display("\n==== K (float) T x D_LEN=%0d ====", D_LEN);
    for (i=0;i<T;i++) begin
      $write("K[%0d]: ", i);
      for (j=0;j<D_LEN;j++) begin
        f = fp32_to_real(K_fp[i][j]);
        $write("%10.4f ", f);
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

  int t,d,tq,tk;
  logic [31:0] tmp;

  initial begin
    $dumpfile("./vvp/tb_att_case2.vcd");
    $dumpvars(0, tb_attention_score_with_fsm);

    init_sig();
    wait (rst_n == 1'b1);
    repeat (2) @(posedge clk);

    // init matrices
    for (t=0;t<T;t++) for (d=0;d<DMAX;d++) begin
      Q_fp[t][d] = 32'h00000000;
      K_fp[t][d] = 32'h00000000;
    end

    // ===================== CASE2 vectors =====================

    // Q
    Q_fp[0][0]=32'hc0000000; Q_fp[0][1]=32'hbfc00000; Q_fp[0][2]=32'hbf800000; Q_fp[0][3]=32'hbf000000;
    Q_fp[1][0]=32'h00000000; Q_fp[1][1]=32'h3f000000; Q_fp[1][2]=32'h3f800000; Q_fp[1][3]=32'h3fc00000;
    Q_fp[2][0]=32'h40000000; Q_fp[2][1]=32'hc0000000; Q_fp[2][2]=32'hbfc00000; Q_fp[2][3]=32'hbf800000;
    Q_fp[3][0]=32'hbf000000; Q_fp[3][1]=32'h00000000; Q_fp[3][2]=32'h3f000000; Q_fp[3][3]=32'h3f800000;

    // K
    K_fp[0][0]=32'hbfc00000; K_fp[0][1]=32'h00000000; K_fp[0][2]=32'h3fc00000; K_fp[0][3]=32'hbfc00000;
    K_fp[1][0]=32'h40000000; K_fp[1][1]=32'hbf800000; K_fp[1][2]=32'h3f000000; K_fp[1][3]=32'h40000000;
    K_fp[2][0]=32'h3f800000; K_fp[2][1]=32'hc0000000; K_fp[2][2]=32'hbf000000; K_fp[2][3]=32'h3f800000;
    K_fp[3][0]=32'h00000000; K_fp[3][1]=32'h3fc00000; K_fp[3][2]=32'hbfc00000; K_fp[3][3]=32'h00000000;

    // write Q
    $display("[TB] Write Q (case2) ...");
    for (t=0;t<T;t++) for (d=0; d<D_LEN; d++) begin
      write_Q(t,d,Q_fp[t][d]);
    end

    // write K
    $display("[TB] Write K (case2) ...");
    for (t=0;t<T;t++) for (d=0; d<D_LEN; d++) begin
      write_K(t,d,K_fp[t][d]);
    end

    print_Q_dec();
    print_K_dec();

    // start FSM
    $display("[TB] FSM start ...");
    @(posedge clk);
    fsm_start <= 1'b1;
    @(posedge clk);
    fsm_start <= 1'b0;

    // wait done
    while (fsm_done !== 1'b1) @(posedge clk);
    $display("[TB] FSM done.");

    // read score
    $display("[TB] Read Score ...");
    for (tq=0;tq<T;tq++) begin
      for (tk=0;tk<T;tk++) begin
        read_S(tq,tk,tmp);
        S_rd[tq][tk] = tmp;
      end
    end
    print_S_hex();

    // ===================== expected check =====================
    // Expected S = Q * K^T
    // [  2.25  -4.00   2.25  -1.00
    //   -0.75   3.00  -3.75   2.00
    //   -3.75   3.25   8.25  -8.50
    //    0.00   1.25  -2.25   1.25 ]
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


    $display("[TB] DONE.");
    repeat (20) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
