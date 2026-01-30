// ============================================================================
// tb_attention_top.sv
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_top.vvp ./test/tb_attention_top.sv
//   vvp ./vvp/tb_att_top.vvp
// ============================================================================

`include "./src/EPU/attention_score/attention_top.sv"
`include "./src/EPU/attention_score/attention_softmax_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_top;

  // ----------------------------
  // Params (small debug)
  // ----------------------------
  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned D_LEN  = 4;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned CONFLICT_POLICY_C = 1;
  localparam int unsigned TR_M   = 2;

  localparam int unsigned T_W = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W = (DMAX<=1)?1:$clog2(DMAX);
  localparam int unsigned ROW_W = T_W;
  localparam int unsigned COL_W = T_W;

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
  // DUT I/O (attention_top)
  // ----------------------------
  logic        start;
  logic [15:0] D_len;
  logic        dut_busy, dut_done;

  logic [T-1:0] pad_valid;
  logic         causal_en;

  // Q
  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  // K
  logic                 cpu_k_we;
  logic [31:0]          cpu_k_t;
  logic [31:0]          cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  // V
  logic                 cpu_v_we;
  logic [31:0]          cpu_v_tk;
  logic [31:0]          cpu_v_d;
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

    .start(start),
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

  // --------------------------------------------------------------------------
  // Reference softmax block (for reading P)
  // --------------------------------------------------------------------------
  logic sm_busy, sm_done;

  // ref E debug unused
  logic              e_re_ref;
  logic [ROW_W-1:0]  e_tq_ref;
  logic [COL_W-1:0]  e_tk_ref;
  logic [31:0]       e_rdata_ref;
  logic              e_rvalid_ref;

  // ref P read
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
  ) u_ref_softmax (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(sm_busy),
    .done(sm_done),

    .pad_valid(pad_valid),
    .causal_en(causal_en),

    // share same Q/K writes as DUT
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

  // disable ref E reads
  initial begin
    e_re_ref = 1'b0;
    e_tq_ref = '0;
    e_tk_ref = '0;
  end

  // ----------------------------
  // Helpers: FP32 -> real
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

  // ----------------------------
  // CPU write tasks (Q/K/V)
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
      cpu_q_we <= 1'b0;
    end
  endtask

  task automatic cpu_write_k(input int tk, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_k_we    <= 1'b1;
      cpu_k_t     <= tk;
      cpu_k_d     <= d;
      cpu_k_wdata <= bits;
      @(posedge clk);
      cpu_k_we <= 1'b0;
    end
  endtask

  task automatic cpu_write_v(input int tk, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_v_we    <= 1'b1;
      cpu_v_tk    <= tk;
      cpu_v_d     <= d;
      cpu_v_wdata <= bits;
      cpu_v_wmask <= {BYTE_W{1'b1}};
      @(posedge clk);
      cpu_v_we <= 1'b0;
    end
  endtask

  // ----------------------------
  // Read P from reference softmax
  // ----------------------------
  task automatic read_p_ref(input int tq_i, input int tk_i, output logic [31:0] bits);
    begin
      @(posedge clk);
      p_re_ref <= 1'b1;
      p_tq_ref <= tq_i[ROW_W-1:0];
      p_tk_ref <= tk_i[COL_W-1:0];
      @(posedge clk);
      p_re_ref <= 1'b0;

      while (!p_rvalid_ref) @(posedge clk);
      bits = p_rdata_ref;
    end
  endtask

  // ----------------------------
  // Read O from DUT  (HOLD re until rvalid to avoid pulse-handshake issues)
  // ----------------------------
  task automatic read_o(input int tq_i, input int d_i, output logic [31:0] bits);
    int guard;
    begin
      @(posedge clk);
      o_tq <= tq_i[T_W-1:0];
      o_d  <= d_i[D_W-1:0];

      o_en <= 1'b1;
      o_re <= 1'b1;

      guard = 0;
      while (!o_rvalid && guard < 2000) begin
        @(posedge clk);
        guard++;
      end

      if (!o_rvalid) begin
        $display("[TB][O_RD_TIMEOUT] tq=%0d d=%0d dut_done=%0d", tq_i, d_i, dut_done);
        $finish;
      end

      bits = o_rdata;

      @(posedge clk);
      o_re <= 1'b0;
    end
  endtask

  // ----------------------------
  // Timeout
  // ----------------------------
  initial begin
    int cyc = 0;
    while (cyc < 500000) begin
      @(posedge clk);
      cyc++;
    end
    $display("[TB] TIMEOUT! dut_busy=%0d dut_done=%0d sm_done=%0d", dut_busy, dut_done, sm_done);
    $finish;
  end

  // ----------------------------
  // Matrices (bits)
  // ----------------------------
  logic [31:0] Q_bits [0:T-1][0:D_LEN-1];
  logic [31:0] K_bits [0:T-1][0:D_LEN-1];
  logic [31:0] V_bits [0:T-1][0:D_LEN-1];
  logic [31:0] P_bits [0:T-1][0:T-1];
  logic [31:0] O_bits [0:T-1][0:D_LEN-1];

  // compare vars
  int  err;
  real tol;
  real got;
  real diff;
  real acc;

  // ----------------------------
  // Test
  // ----------------------------
  initial begin
    // defaults
    start      = 1'b0;
    D_len      = D_LEN;
    pad_valid  = {T{1'b1}};
    causal_en  = 1'b0;

    cpu_q_we   = 1'b0;
    cpu_q_t    = '0;
    cpu_q_d    = '0;
    cpu_q_wdata= '0;
    cpu_q_wmask= '0;

    cpu_k_we   = 1'b0;
    cpu_k_t    = '0;
    cpu_k_d    = '0;
    cpu_k_wdata= '0;

    cpu_v_we   = 1'b0;
    cpu_v_tk   = '0;
    cpu_v_d    = '0;
    cpu_v_wdata= '0;
    cpu_v_wmask= '0;

    p_re_ref   = 1'b0;
    p_tq_ref   = '0;
    p_tk_ref   = '0;

    o_re       = 1'b0;
    o_en       = 1'b1;
    o_tq       = '0;
    o_d        = '0;

    // wait reset release
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    $display("[TB] Write Q/K/V ...");

    // deterministic patterns
    for (int t=0; t<T; t++) begin
      for (int d=0; d<D_LEN; d++) begin
        Q_bits[t][d] = 32'h40000000 + (t<<23) + (d<<18);
        cpu_write_q(t, d, Q_bits[t][d]);
      end
    end

    for (int t=0; t<T; t++) begin
      for (int d=0; d<D_LEN; d++) begin
        K_bits[t][d] = 32'h40000000 + (t<<23) + (d<<18);
        cpu_write_k(t, d, K_bits[t][d]);
      end
    end

    for (int tk=0; tk<T; tk++) begin
      for (int d=0; d<D_LEN; d++) begin
        V_bits[tk][d] = 32'h3f000000 + (tk<<2) + (d<<12 ); // around 0.5
        cpu_write_v(tk, d, V_bits[tk][d]);
      end
    end

    $display("[TB] start=1 until done...");
    @(posedge clk);
    start <= 1'b1;

    while (!dut_busy) @(posedge clk);
    while (!dut_done) @(posedge clk);

    // drop start to re-arm
    @(posedge clk);
    start <= 1'b0;
    $display("[TB] dut_done asserted.");

    // ------------------------------------------------------------
    // Dump Q/K/V
    // ------------------------------------------------------------
    $display("");
    $display("==== Q (input) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int d=0; d<D_LEN; d++) $write("%f ", fp32_to_real(Q_bits[r][d]));
      $write("\n");
    end

    $display("");
    $display("==== K (input) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int d=0; d<D_LEN; d++) $write("%f ", fp32_to_real(K_bits[r][d]));
      $write("\n");
    end

    $display("");
    $display("==== V (input) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int d=0; d<D_LEN; d++) $write("%f ", fp32_to_real(V_bits[r][d]));
      $write("\n");
    end

    // ------------------------------------------------------------
    // Read P from reference softmax
    // ------------------------------------------------------------
    $display("");
    $display("==== P (from ref softmax) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        read_p_ref(r, c, P_bits[r][c]);
        $write("%f ", fp32_to_real(P_bits[r][c]));
      end
      $write("\n");
    end

    // quick row-sum check
    $display("");
    $display("==== CHECK: row sum(P) should be ~1.0 ====");
    for (int r=0; r<T; r++) begin
      real s;
      s = 0.0;
      for (int c=0; c<T; c++) s += fp32_to_real(P_bits[r][c]);
      $display("row %0d sum(P)= %f", r, s);
    end

    // ------------------------------------------------------------
    // Read O from DUT
    // ------------------------------------------------------------
    $display("");
    $display("==== O (from DUT) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int d=0; d<D_LEN; d++) begin
        read_o(r, d, O_bits[r][d]);
        $write("%f ", fp32_to_real(O_bits[r][d]));
      end
      $write("\n");
    end

    // ------------------------------------------------------------
    // Print O_expected = P*V (TB real)
    // ------------------------------------------------------------
    $display("");
    $display("==== O_expected = P*V (TB real) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int d=0; d<D_LEN; d++) begin
        real acc2;
        acc2 = 0.0;
        for (int k=0; k<T; k++) acc2 += fp32_to_real(P_bits[r][k]) * fp32_to_real(V_bits[k][d]);
        $write("%f ", acc2);
      end
      $write("\n");
    end

    // ------------------------------------------------------------
    // Compare O vs expected
    // ------------------------------------------------------------
    $display("");
    $display("==== COMPARE: O_expected = P*V ====");
    err = 0;
    tol = 1e-2;

    for (int r=0; r<T; r++) begin
      for (int d=0; d<D_LEN; d++) begin
        acc = 0.0;
        for (int k=0; k<T; k++) acc += fp32_to_real(P_bits[r][k]) * fp32_to_real(V_bits[k][d]);

        got = fp32_to_real(O_bits[r][d]);
        diff = (got > acc) ? (got-acc) : (acc-got);

        if (diff > tol) begin
          $display("[FAIL] r=%0d d=%0d got=%f exp=%f diff=%f", r, d, got, acc, diff);
          err++;
        end
      end
    end

    if (err == 0) $display("[PASS] O matches P*V within tol=%f", tol);
    else          $display("[FAIL] total mismatches = %0d (tol=%f)", err, tol);

    $display("[TB] finish.");
    $finish;
  end

endmodule

`default_nettype wire
