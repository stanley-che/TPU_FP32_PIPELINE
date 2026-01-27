/*
=============================================================================
 tb_attention_top_file_golden_iverilog.sv  (UPDATED)
 - Icarus-friendly TB
 - Loads Q/K/V from hex files (32-bit per line)
 - Loads O_golden from hex file
 - Writes Q/K/V into DUT attention_top
 - Runs DUT
 - Reads O and compares vs golden
 - Compare supports:
     MODE=bits  : exact bits compare
     MODE=tol   : abs/rel tolerance compare on real

 Plusargs:
   +Q_HEX=path
   +K_HEX=path
   +V_HEX=path
   +O_GOLD_HEX=path
   +VCD=path                 (optional; default ./vvp/tb_att_T<T>_D<D>.vcd)
   +MODE=bits|tol            (default tol)
   +TOL=1e-2                 (abs tol, default 1e-2)
   +RTOL=1e-2                (rel tol, default 1e-2)
   +STRICT_T=1               (optional: enforce T in {4,8,16,32,64})

iverilog -g2012 -Wall -I./src \
  -P tb_attention_top_file_golden_iverilog.T=32 \
  -P tb_attention_top_file_golden_iverilog.D_LEN=32 \
  -P tb_attention_top_file_golden_iverilog.DMAX=64 \
  -o ./vvp/tb_att_T64.vvp ./test/tb_attention_top_file_golden_iverilog.sv

vvp ./vvp/tb_att_T64.vvp \
  +Q_HEX=./test/Q.hex +K_HEX=./test/K.hex +V_HEX=./test/V.hex \
  +O_GOLD_HEX=./test/O_golden.hex \
  +MODE=tol +TOL=1e-2 +RTOL=1e-2 \
  +STRICT_T=1

=============================================================================
*/

`include "./src/EPU/attention_score/attention_top.sv"

`timescale 1ns/1ps
`default_nettype wire

module tb_attention_top_file_golden_iverilog #(
  parameter int unsigned T      = 4,
  parameter int unsigned DMAX   = 8,
  parameter int unsigned D_LEN  = 4,

  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 2,
  parameter int unsigned CONFLICT_POLICY_C = 1,
  parameter int unsigned TR_M   = 6
);

  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned T_W    = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W    = (DMAX<=1)?1:$clog2(DMAX);

  // ------------------------------------------------------------
  // Sanity
  // ------------------------------------------------------------
  function automatic bit is_pow2(input int unsigned x);
    if (x == 0) return 0;
    return ((x & (x-1)) == 0);
  endfunction

  function automatic bit is_allowed_T(input int unsigned x);
    return (x==4 || x==8 || x==16 || x==32 || x==64);
  endfunction

  int strict_t;
  initial begin
    strict_t = 0;
    void'($value$plusargs("STRICT_T=%d", strict_t));

    if (D_LEN > DMAX) begin
      $display("[TB][FATAL] D_LEN(%0d) must be <= DMAX(%0d)", D_LEN, DMAX);
      $finish;
    end

    if (!is_pow2(T)) begin
      $display("[TB][WARN] T=%0d is not power-of-2 (still allowed)", T);
    end

    if (strict_t != 0 && !is_allowed_T(T)) begin
      $display("[TB][FATAL] STRICT_T=1 but T=%0d not in {4,8,16,32,64}", T);
      $finish;
    end
  end

  // ------------------------------------------------------------
  // clock/reset
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // DUT I/O
  // ------------------------------------------------------------
  logic        start_dut;
  logic [15:0] D_len;
  logic        dut_busy, dut_done;

  logic [T-1:0] pad_valid;
  logic         causal_en;

  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  logic                 cpu_k_we;
  logic [T_W-1:0]       cpu_k_t;
  logic [D_W-1:0]       cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  logic                 cpu_v_we;
  logic [T_W-1:0]       cpu_v_tk;
  logic [D_W-1:0]       cpu_v_d;
  logic [DATA_W-1:0]    cpu_v_wdata;
  logic [BYTE_W-1:0]    cpu_v_wmask;

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

  // ------------------------------------------------------------
  // Storage from files
  // ------------------------------------------------------------
  localparam int unsigned QKV_N = T*D_LEN;
  localparam int unsigned O_N   = T*D_LEN;

  logic [31:0] Q_bits   [0:QKV_N-1];
  logic [31:0] K_bits   [0:QKV_N-1];
  logic [31:0] V_bits   [0:QKV_N-1];
  logic [31:0] O_golden [0:O_N-1];
  logic [31:0] O_got    [0:O_N-1];

  // ------------------------------------------------------------
  // FP32 bits -> real
  // ------------------------------------------------------------
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
  task automatic print_Q;
  begin
    $display("\n==== O (DUT) vs O (GOLDEN) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d GOT : ", r);
      for (int c=0; c<D_LEN; c++) begin
        int idx;
        idx = r*D_LEN + c;
        $write("%10.6f ", fp32_to_real(Q_bits[idx]));
      end
      $write("\n");
    end
  end
endtask
task automatic print_K;
  begin
    $display("\n==== O (DUT) vs O (GOLDEN) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d GOT : ", r);
      for (int c=0; c<D_LEN; c++) begin
        int idx;
        idx = r*D_LEN + c;
        $write("%10.6f ", fp32_to_real(K_bits[idx]));
      end
      $write("\n");
    end
  end
endtask
task automatic print_V;
  begin
    $display("\n==== O (DUT) vs O (GOLDEN) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d GOT : ", r);
      for (int c=0; c<D_LEN; c++) begin
        int idx;
        idx = r*D_LEN + c;
        $write("%10.6f ", fp32_to_real(V_bits[idx]));
      end
      $write("\n");
    end
  end
endtask

  task automatic print_O;
  begin
    $display("\n==== O (DUT) vs O (GOLDEN) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d GOT : ", r);
      for (int c=0; c<D_LEN; c++) begin
        int idx;
        idx = r*D_LEN + c;
        $write("%10.6f ", fp32_to_real(O_got[idx]));
      end
      $write("\n");

      $write("row %0d GLD : ", r);
      for (int c=0; c<D_LEN; c++) begin
        int idx;
        idx = r*D_LEN + c;
        $write("%10.6f ", fp32_to_real(O_golden[idx]));
      end
      $write("\n");
    end
  end
endtask


  // ------------------------------------------------------------
  // Plusargs / file IO
  // ------------------------------------------------------------
  string Q_hex, K_hex, V_hex, O_hex, vcd_name, mode_s;
  real tol, rtol;

  function automatic string default_vcd_name();
    string s;
    begin
      s = $sformatf("./vvp/tb_att_T%0d_D%0d.vcd", T, D_LEN);
      return s;
    end
  endfunction

  task automatic load_vectors_from_files;
    begin
      // defaults
      Q_hex    = "./test/Q.hex";
      K_hex    = "./test/K.hex";
      V_hex    = "./test/V.hex";
      O_hex    = "./test/O_golden.hex";
      vcd_name = default_vcd_name();

      mode_s = "tol";   // "tol" or "bits"
      tol    = 1e-2;    // abs tolerance
      rtol   = 1e-2;    // relative tolerance

      void'($value$plusargs("Q_HEX=%s", Q_hex));
      void'($value$plusargs("K_HEX=%s", K_hex));
      void'($value$plusargs("V_HEX=%s", V_hex));
      void'($value$plusargs("O_GOLD_HEX=%s", O_hex));
      void'($value$plusargs("VCD=%s", vcd_name));
      void'($value$plusargs("MODE=%s", mode_s));
      void'($value$plusargs("TOL=%f", tol));
      void'($value$plusargs("RTOL=%f", rtol));

      $display("[TB] T=%0d D_LEN=%0d DMAX=%0d", T, D_LEN, DMAX);
      $display("[TB] Q_HEX      = %s", Q_hex);
      $display("[TB] K_HEX      = %s", K_hex);
      $display("[TB] V_HEX      = %s", V_hex);
      $display("[TB] O_GOLD_HEX = %s", O_hex);
      $display("[TB] MODE       = %s", mode_s);
      $display("[TB] TOL/RTOL   = %0.6f / %0.6f", tol, rtol);
      $display("[TB] VCD        = %s", vcd_name);

      $readmemh(Q_hex, Q_bits);
      $readmemh(K_hex, K_bits);
      $readmemh(V_hex, V_bits);
      $readmemh(O_hex, O_golden);
    end
  endtask

  // ------------------------------------------------------------
  // CPU write tasks
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // Start / Read O
  // ------------------------------------------------------------
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

  task automatic read_o_cell(input int tq_i, input int d_i, output logic [31:0] bits);
    int guard;
    int guard_max;
    begin
      // scale timeout with T and D_LEN
      guard_max = 20000 + (T*T)*4 + (T*D_LEN)*8;

      @(posedge clk);
      o_en <= 1'b1;
      o_re <= 1'b1;
      o_tq <= tq_i[T_W-1:0];
      o_d  <= d_i[D_W-1:0];
      @(posedge clk);
      o_re <= 1'b0;

      guard = 0;
      while (!o_rvalid && guard < guard_max) begin
        @(posedge clk);
        guard++;
      end
      if (!o_rvalid) begin
        $display("[TB][O_RD_TIMEOUT] tq=%0d d=%0d guard=%0d/%0d", tq_i, d_i, guard, guard_max);
        $finish;
      end
      bits = o_rdata;
      @(posedge clk);
    end
  endtask

  // ------------------------------------------------------------
  // Watchdog (scale with size)
  // ------------------------------------------------------------
  initial begin
    int cyc = 0;
    int max_cyc;
    max_cyc = 2_000_000 + (T*T)*2000 + (T*D_LEN)*4000;
    while (cyc < max_cyc) begin
      @(posedge clk);
      cyc++;
    end
    $display("[TB] TIMEOUT cyc=%0d/%0d dut_busy=%0d dut_done=%0d", cyc, max_cyc, dut_busy, dut_done);
    $finish;
  end

  // ------------------------------------------------------------
  // Compare helpers
  // ------------------------------------------------------------
  function automatic real absr(input real x);
    return (x < 0.0) ? -x : x;
  endfunction

  function automatic real maxr(input real a, input real b);
    return (a > b) ? a : b;
  endfunction

  function automatic bit pass_tol(input real got, input real exp, input real at, input real rt);
    real diff;
    real denom;
    begin
      diff  = absr(got - exp);
      denom = maxr(absr(exp), 1e-12); // avoid div0
      pass_tol = (diff <= at) || (diff <= rt * denom);
    end
  endfunction

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  int err;
  initial begin
    // init signals
    start_dut = 1'b0;
    D_len     = D_LEN[15:0];
    pad_valid = {T{1'b1}};
    causal_en = 1'b0;

    cpu_q_we = 0; cpu_k_we = 0; cpu_v_we = 0;
    cpu_q_t = '0; cpu_q_d = '0; cpu_q_wdata = '0; cpu_q_wmask = '0;
    cpu_k_t = '0; cpu_k_d = '0; cpu_k_wdata = '0;
    cpu_v_tk = '0; cpu_v_d = '0; cpu_v_wdata = '0; cpu_v_wmask = '0;

    o_en = 1'b1; o_re = 1'b0; o_tq = '0; o_d = '0;

    load_vectors_from_files();

    // dump waves (always; user can set VCD="" to disable)
    if (vcd_name != "") begin
      $dumpfile(vcd_name);
      $dumpvars(0, tb_attention_top_file_golden_iverilog);
    end

    wait (rst_n == 1'b1);
    repeat (5) @(posedge clk);

    // write Q/K/V
    $display("[TB] Write Q ...");
    for (int i=0;i<T;i++) begin
      for (int d=0; d<D_LEN; d++) cpu_write_q(i, d, Q_bits[i*D_LEN + d]);
    end

    $display("[TB] Write K ...");
    for (int i=0;i<T;i++) begin
      for (int d=0; d<D_LEN; d++) cpu_write_k(i, d, K_bits[i*D_LEN + d]);
    end

    $display("[TB] Write V ...");
    for (int i=0;i<T;i++) begin
      for (int d=0; d<D_LEN; d++) cpu_write_v(i, d, V_bits[i*D_LEN + d]);
    end

    // run DUT
    $display("[TB] Run DUT attention_top ...");
    run_dut();
    print_Q();
    print_K();
    print_V();
    // read O
    $display("[TB] Read O ...");
    for (int r=0;r<T;r++) begin
      for (int d=0; d<D_LEN; d++) read_o_cell(r, d, O_got[r*D_LEN + d]);
    end
    print_O();
    // compare
    $display("\n==== COMPARE O vs GOLDEN ====");
    err = 0;

    if (mode_s == "bits") begin
      for (int r=0;r<T;r++) begin
        for (int d=0; d<D_LEN; d++) begin
          int idx;
          idx = r*D_LEN + d;
          if (O_got[idx] !== O_golden[idx]) begin
            $display("[FAIL][BITS] r=%0d d=%0d got=%08x exp=%08x",
                     r, d, O_got[idx], O_golden[idx]);
            err++;
          end
        end
      end
    end else begin
      for (int r=0;r<T;r++) begin
        for (int d=0; d<D_LEN; d++) begin
          int idx;
          real got, exp;
          idx = r*D_LEN + d;
          got = fp32_to_real(O_got[idx]);
          exp = fp32_to_real(O_golden[idx]);
          if (!pass_tol(got, exp, tol, rtol)) begin
            real diff;
            diff = absr(got-exp);
            $display("[FAIL][TOL] r=%0d d=%0d got=%0.6f exp=%0.6f diff=%0.6f (got=%08x exp=%08x)",
                     r, d, got, exp, diff, O_got[idx], O_golden[idx]);
            err++;
          end
        end
      end
    end

    if (err == 0) $display("[PASS] O matches GOLDEN (mode=%s tol=%0.6f rtol=%0.6f)", mode_s, tol, rtol);
    else          $display("[FAIL] mismatches=%0d (mode=%s tol=%0.6f rtol=%0.6f)", err, mode_s, tol, rtol);
    
    $display("[TB] DONE.");
    $finish;
  end

endmodule

`default_nettype wire
