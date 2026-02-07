// ============================================================================
// tb_attention_top_test2_case2_iverilog.sv
// - Deterministic CASE2 Q/K/V
// - Separate start for REF softmax and DUT attention_top (avoid contention)
// - Start handshake: hold-high until busy, then drop; wait done
// - Prints S / S_scaled / rowmax / S_sub (TB computed)
// - Reads full E/P from ref, prints + row sums
// - Reads full O from DUT, compares O ~= P*V
//
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_case2.vvp ./test/tb_attention_top_test2.sv
//   vvp ./vvp/tb_att_case2.vvp
// ============================================================================

`include "./src/EPU/attention_score/attention_top.sv"
`include "./src/EPU/attention_score/attention_softmax_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_top_test2_case2_iverilog;

  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned D_LEN  = 4;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned CONFLICT_POLICY_C = 1;
  localparam int unsigned TR_M   = 6;

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX);
  localparam int unsigned ROW_W = T_W;
  localparam int unsigned COL_W = T_W;

  // clock/reset
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
  // DUT: attention_top
  // ----------------------------
  logic        start_dut;
  logic [15:0] D_len;
  logic        dut_busy, dut_done;

  logic [T-1:0] pad_valid;
  logic         causal_en;

  // Q writes
  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  // K writes
  logic                 cpu_k_we;
  logic [T_W-1:0]       cpu_k_t;
  logic [D_W-1:0]       cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  // V writes
  logic                 cpu_v_we;
  logic [T_W-1:0]       cpu_v_tk;
  logic [D_W-1:0]       cpu_v_d;
  logic [DATA_W-1:0]    cpu_v_wdata;
  logic [BYTE_W-1:0]    cpu_v_wmask;

  // O read
  logic              o_re;
  logic              o_en;
  logic [T_W-1:0]    o_tq;
  logic [D_W-1:0]    o_d;
  logic [DATA_W-1:0] o_rdata;
  logic              o_rvalid;

  attention_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .start(start_dut),
    .D_len(D_len),
    .busy(dut_busy),
    .done(dut_done),

    .pad_valid(pad_valid),
    .causal_en(causal_en),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .cpu_v_we(cpu_v_we),
    .cpu_v_tk(cpu_v_tk),
    .cpu_v_d(cpu_v_d),
    .cpu_v_wdata(cpu_v_wdata),
    .cpu_v_wmask(cpu_v_wmask),

    .o_re(o_re),
    .o_en(o_en),
    .o_tq(o_tq),
    .o_d(o_d),
    .o_rdata(o_rdata),
    .o_rvalid(o_rvalid)
  );

  // ----------------------------
  // REF: attention_softmax_top (E/P)
  // ----------------------------
  logic start_ref;
  logic sm_busy, sm_done;

  logic              e_re_ref;
  logic [ROW_W-1:0]  e_tq_ref;
  logic [COL_W-1:0]  e_tk_ref;
  logic [31:0]       e_rdata_ref;
  logic              e_rvalid_ref;

  logic              p_re_ref;
  logic [ROW_W-1:0]  p_tq_ref;
  logic [COL_W-1:0]  p_tk_ref;
  logic [31:0]       p_rdata_ref;
  logic              p_rvalid_ref;

  attention_softmax_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_ref (
    .clk(clk),
    .rst_n(rst_n),

    .start(start_ref),
    .D_len(D_len),
    .busy(sm_busy),
    .done(sm_done),

    .pad_valid(pad_valid),
    .causal_en(causal_en),

    // share Q/K writes
    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .e_re(e_re_ref),
    .e_tq(e_tq_ref),
    .e_tk(e_tk_ref),
    .e_rdata(e_rdata_ref),
    .e_rvalid(e_rvalid_ref),

    .p_re(p_re_ref),
    .p_tq(p_tq_ref),
    .p_tk(p_tk_ref),
    .p_rdata(p_rdata_ref),
    .p_rvalid(p_rvalid_ref)
  );

  // ----------------------------
  // Storage
  // ----------------------------
  logic [31:0] Q_bits [0:T-1][0:D_LEN-1];
  logic [31:0] K_bits [0:T-1][0:D_LEN-1];
  logic [31:0] V_bits [0:T-1][0:D_LEN-1];

  logic [31:0] E_bits [0:T-1][0:T-1];
  logic [31:0] P_bits [0:T-1][0:T-1];
  logic [31:0] O_bits [0:T-1][0:D_LEN-1];

  real S    [0:T-1][0:T-1];
  real Ssc  [0:T-1][0:T-1];
  real rowm [0:T-1];
  real Ssub [0:T-1][0:T-1];

  // ----------------------------
  // FP32 -> real
  // ----------------------------
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

      fp32_to_real = s ? -val : val;
    end
  endfunction

  // ----------------------------
  // CPU writes
  // ----------------------------
  task automatic cpu_write_q(input int tq, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_q_we    <= 1'b1;
      cpu_q_t     <= tq[T_W-1:0];
      cpu_q_d     <= d[D_W-1:0];
      cpu_q_wdata <= bits;
      cpu_q_wmask <= {BYTE_W{1'b1}};
      @(posedge clk);
      cpu_q_we    <= 1'b0;
      cpu_q_wmask <= '0;
    end
  endtask

  task automatic cpu_write_k(input int tk, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_k_we    <= 1'b1;
      cpu_k_t     <= tk[T_W-1:0];
      cpu_k_d     <= d[D_W-1:0];
      cpu_k_wdata <= bits;
      @(posedge clk);
      cpu_k_we    <= 1'b0;
    end
  endtask

  task automatic cpu_write_v(input int tk, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_v_we    <= 1'b1;
      cpu_v_tk    <= tk[T_W-1:0];
      cpu_v_d     <= d[D_W-1:0];
      cpu_v_wdata <= bits;
      cpu_v_wmask <= {BYTE_W{1'b1}};
      @(posedge clk);
      cpu_v_we    <= 1'b0;
      cpu_v_wmask <= '0;
    end
  endtask

  // ----------------------------
  // Start handshake helpers
  // ----------------------------
  task automatic run_ref();
    begin
      @(posedge clk);
      start_ref <= 1'b1;
      while (sm_busy !== 1'b1) @(posedge clk);
      start_ref <= 1'b0;
      while (sm_done !== 1'b1) @(posedge clk);
      @(posedge clk);
    end
  endtask

  task automatic run_dut();
    begin
      @(posedge clk);
      start_dut <= 1'b1;
      while (dut_busy !== 1'b1) @(posedge clk);
      start_dut <= 1'b0;
      while (dut_done !== 1'b1) @(posedge clk);
      @(posedge clk);
    end
  endtask

  // ----------------------------
  // Read ref E/P
  // ----------------------------
  task automatic read_e_ref_cell(input int tq_i, input int tk_i, output logic [31:0] bits);
    int guard;
    begin
      @(posedge clk);
      e_re_ref <= 1'b1;
      e_tq_ref <= tq_i[ROW_W-1:0];
      e_tk_ref <= tk_i[COL_W-1:0];
      @(posedge clk);
      e_re_ref <= 1'b0;

      guard = 0;
      while (!e_rvalid_ref && guard < 4000) begin
        @(posedge clk);
        guard++;
      end
      if (!e_rvalid_ref) begin
        $display("[TB][E_RD_TIMEOUT] tq=%0d tk=%0d", tq_i, tk_i);
        $finish;
      end
      bits = e_rdata_ref;
      @(posedge clk);
    end
  endtask

  task automatic read_p_ref_cell(input int tq_i, input int tk_i, output logic [31:0] bits);
    int guard;
    begin
      @(posedge clk);
      p_re_ref <= 1'b1;
      p_tq_ref <= tq_i[ROW_W-1:0];
      p_tk_ref <= tk_i[COL_W-1:0];
      @(posedge clk);
      p_re_ref <= 1'b0;

      guard = 0;
      while (!p_rvalid_ref && guard < 4000) begin
        @(posedge clk);
        guard++;
      end
      if (!p_rvalid_ref) begin
        $display("[TB][P_RD_TIMEOUT] tq=%0d tk=%0d", tq_i, tk_i);
        $finish;
      end
      bits = p_rdata_ref;
      @(posedge clk);
    end
  endtask

  // ----------------------------
  // Read O
  // ----------------------------
  task automatic read_o_cell(input int tq_i, input int d_i, output logic [31:0] bits);
    int guard;
    begin
      @(posedge clk);
      o_en <= 1'b1;
      o_re <= 1'b1;
      o_tq <= tq_i[T_W-1:0];
      o_d  <= d_i[D_W-1:0];
      @(posedge clk);
      o_re <= 1'b0;

      guard = 0;
      while (!o_rvalid && guard < 4000) begin
        @(posedge clk);
        guard++;
      end
      if (!o_rvalid) begin
        $display("[TB][O_RD_TIMEOUT] tq=%0d d=%0d", tq_i, d_i);
        $finish;
      end
      bits = o_rdata;
      @(posedge clk);
    end
  endtask

  // ----------------------------
  // Print matrices (no array args)
  // ----------------------------
  task automatic print_QKV;
    begin
      $display("\n==== Q (T=%0d,D=%0d) ====", T, D_LEN);
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<D_LEN;c++) $write("%8.4f ", fp32_to_real(Q_bits[r][c]));
        $write("\n");
      end

      $display("\n==== K (T=%0d,D=%0d) ====", T, D_LEN);
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<D_LEN;c++) $write("%8.4f ", fp32_to_real(K_bits[r][c]));
        $write("\n");
      end

      $display("\n==== V (T=%0d,D=%0d) ====", T, D_LEN);
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<D_LEN;c++) $write("%8.4f ", fp32_to_real(V_bits[r][c]));
        $write("\n");
      end
    end
  endtask

  // ----------------------------
  // TB compute + print softmax pre-stages
  // ----------------------------
  task automatic build_and_print_softmax_pre;
    real alpha;
    begin
      alpha = 1.0 / ((D_LEN*1.0) ** 0.5); // 0.5

      for (int i=0;i<T;i++) begin
        for (int j=0;j<T;j++) begin
          real acc;
          acc = 0.0;
          for (int d=0; d<D_LEN; d++) begin
            acc += fp32_to_real(Q_bits[i][d]) * fp32_to_real(K_bits[j][d]);
          end
          S[i][j]   = acc;
          Ssc[i][j] = acc * alpha;
        end
      end

      for (int i=0;i<T;i++) begin
        real m;
        m = Ssc[i][0];
        for (int j=1;j<T;j++) if (Ssc[i][j] > m) m = Ssc[i][j];
        rowm[i] = m;
        for (int j=0;j<T;j++) Ssub[i][j] = Ssc[i][j] - m;
      end

      $display("\n==== S = Q*K^T (before scaling) ====");
      for (int i=0;i<T;i++) begin
        $write("row %0d : ", i);
        for (int j=0;j<T;j++) $write("%10.4f ", S[i][j]);
        $write("\n");
      end

      $display("\n(alpha = 1/sqrt(D_LEN) = %0.6f)", alpha);

      $display("\n==== S_scaled ====");
      for (int i=0;i<T;i++) begin
        $write("row %0d : ", i);
        for (int j=0;j<T;j++) $write("%10.4f ", Ssc[i][j]);
        $write("\n");
      end

      $display("\n==== rowmax(S_scaled) ====");
      for (int i=0;i<T;i++) $display("row %0d : %10.4f", i, rowm[i]);

      $display("\n==== S_sub = S_scaled - rowmax ====");
      for (int i=0;i<T;i++) begin
        $write("row %0d : ", i);
        for (int j=0;j<T;j++) $write("%10.4f ", Ssub[i][j]);
        $write("\n");
      end
    end
  endtask

  task automatic print_E_P_and_sums;
    begin
      $display("\n==== E = exp(S_sub) (ref) ====");
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<T;c++) $write("%10.6f ", fp32_to_real(E_bits[r][c]));
        $write("\n");
      end

      $display("\n==== row sum(E) ====");
      for (int r=0;r<T;r++) begin
        real s;
        s = 0.0;
        for (int c=0;c<T;c++) s += fp32_to_real(E_bits[r][c]);
        $display("row %0d sum(E)= %0.6f", r, s);
      end

      $display("\n==== P (softmax) (ref) ====");
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<T;c++) $write("%10.6f ", fp32_to_real(P_bits[r][c]));
        $write("\n");
      end

      $display("\n==== row sum(P) ====");
      for (int r=0;r<T;r++) begin
        real s;
        s = 0.0;
        for (int c=0;c<T;c++) s += fp32_to_real(P_bits[r][c]);
        $display("row %0d sum(P)= %0.6f", r, s);
      end
    end
  endtask

  task automatic print_O;
    begin
      $display("\n==== O (DUT) ====");
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<D_LEN;c++) $write("%10.6f ", fp32_to_real(O_bits[r][c]));
        $write("\n");
      end
    end
  endtask

  // watchdog
  initial begin
    int cyc = 0;
    while (cyc < 800000) begin
      @(posedge clk);
      cyc++;
    end
    $display("[TB] TIMEOUT dut_busy=%0d dut_done=%0d sm_busy=%0d sm_done=%0d",
             dut_busy, dut_done, sm_busy, sm_done);
    $finish;
  end

  // ----------------------------
  // Main
  // ----------------------------
  int err;
    real tol;
  initial begin
    start_dut = 1'b0;
    start_ref = 1'b0;
    D_len     = D_LEN[15:0];
    pad_valid = {T{1'b1}};
    causal_en = 1'b0;

    cpu_q_we = 0; cpu_k_we = 0; cpu_v_we = 0;
    cpu_q_t = '0; cpu_q_d = '0; cpu_q_wdata = '0; cpu_q_wmask = '0;
    cpu_k_t = '0; cpu_k_d = '0; cpu_k_wdata = '0;
    cpu_v_tk = '0; cpu_v_d = '0; cpu_v_wdata = '0; cpu_v_wmask = '0;

    e_re_ref = 0; e_tq_ref = '0; e_tk_ref = '0;
    p_re_ref = 0; p_tq_ref = '0; p_tk_ref = '0;

    o_en = 1'b1; o_re = 1'b0; o_tq = '0; o_d = '0;

    $dumpfile("./vvp/tb_att_case2.vcd");
    $dumpvars(0, tb_attention_top_test2_case2_iverilog);

    wait (rst_n == 1'b1);
    repeat (5) @(posedge clk);

    // CASE2 Q
    Q_bits[0][0]=32'hc0000000; Q_bits[0][1]=32'hbfc00000; Q_bits[0][2]=32'hbf800000; Q_bits[0][3]=32'hbf000000;
    Q_bits[1][0]=32'h00000000; Q_bits[1][1]=32'h3f000000; Q_bits[1][2]=32'h3f800000; Q_bits[1][3]=32'h3fc00000;
    Q_bits[2][0]=32'h40000000; Q_bits[2][1]=32'hc0000000; Q_bits[2][2]=32'hbfc00000; Q_bits[2][3]=32'hbf800000;
    Q_bits[3][0]=32'hbf000000; Q_bits[3][1]=32'h00000000; Q_bits[3][2]=32'h3f000000; Q_bits[3][3]=32'h3f800000;

    // CASE2 K
    K_bits[0][0]=32'hbfc00000; K_bits[0][1]=32'h00000000; K_bits[0][2]=32'h3fc00000; K_bits[0][3]=32'hbfc00000;
    K_bits[1][0]=32'h40000000; K_bits[1][1]=32'hbf800000; K_bits[1][2]=32'h3f000000; K_bits[1][3]=32'h40000000;
    K_bits[2][0]=32'h3f800000; K_bits[2][1]=32'hc0000000; K_bits[2][2]=32'hbf000000; K_bits[2][3]=32'h3f800000;
    K_bits[3][0]=32'h00000000; K_bits[3][1]=32'h3fc00000; K_bits[3][2]=32'hbfc00000; K_bits[3][3]=32'h00000000;

    // V deterministic: set V = Q (easy to verify)
    for (int r=0;r<T;r++) begin
      for (int c=0;c<D_LEN;c++) begin
        V_bits[r][c] = Q_bits[r][c];
      end
    end

    // write Q/K/V
    $display("[TB] Write Q ...");
    for (int i=0;i<T;i++) for (int d=0; d<D_LEN; d++) cpu_write_q(i,d,Q_bits[i][d]);

    $display("[TB] Write K ...");
    for (int i=0;i<T;i++) for (int d=0; d<D_LEN; d++) cpu_write_k(i,d,K_bits[i][d]);

    $display("[TB] Write V ...");
    for (int i=0;i<T;i++) for (int d=0; d<D_LEN; d++) cpu_write_v(i,d,V_bits[i][d]);

    print_QKV();
    build_and_print_softmax_pre();

    // run REF softmax and read E/P
    $display("\n[TB] Run REF softmax ...");
    run_ref();

    for (int r=0;r<T;r++) begin
      for (int c=0;c<T;c++) begin
        read_e_ref_cell(r,c,E_bits[r][c]);
        read_p_ref_cell(r,c,P_bits[r][c]);
      end
    end
    print_E_P_and_sums();

    // run DUT and read O
    $display("\n[TB] Run DUT attention_top ...");
    run_dut();

    for (int r=0;r<T;r++) begin
      for (int d=0; d<D_LEN; d++) begin
        read_o_cell(r,d,O_bits[r][d]);
      end
    end
    print_O();

    // compare O vs P*V
    $display("\n==== COMPARE O vs P*V ====");
    
    err = 0;
    tol = 1e-2;

    for (int r=0;r<T;r++) begin
      for (int d=0;d<D_LEN;d++) begin
        real acc, got, diff;
        acc = 0.0;
        for (int k=0;k<T;k++) acc += fp32_to_real(P_bits[r][k]) * fp32_to_real(V_bits[k][d]);
        got  = fp32_to_real(O_bits[r][d]);
        diff = (got > acc) ? (got-acc) : (acc-got);
        if (diff > tol) begin
          $display("[FAIL] r=%0d d=%0d got=%0.6f exp=%0.6f diff=%0.6f", r, d, got, acc, diff);
          err++;
        end
      end
    end

    if (err == 0) $display("[PASS] O matches P*V (tol=%0.6f)", tol);
    else          $display("[FAIL] mismatches=%0d (tol=%0.6f)", err, tol);

    $display("[TB] DONE.");
    $finish;
  end

endmodule

`default_nettype wire
