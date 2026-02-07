// ============================================================================
// tb_attention_softmax_top.sv
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_softmax.vvp ./test/tb_attention_softmax_top.sv
//   vvp ./vvp/tb_softmax.vvp
// ============================================================================

`include "./src/EPU/attention_score/attention_softmax_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_softmax_top;

  // ----------------------------
  // Params (debug small)
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
  // DUT I/O
  // ----------------------------
  logic        start;
  logic [15:0] D_len;
  logic        busy, done;

  logic [T-1:0] pad_valid;
  logic         causal_en;

  // CPU write Q
  logic                 cpu_q_we;
  logic [T_W-1:0]       cpu_q_t;
  logic [D_W-1:0]       cpu_q_d;
  logic [DATA_W-1:0]    cpu_q_wdata;
  logic [BYTE_W-1:0]    cpu_q_wmask;

  // CPU write K
  logic                 cpu_k_we;
  logic [31:0]          cpu_k_t;
  logic [31:0]          cpu_k_d;
  logic [DATA_W-1:0]    cpu_k_wdata;

  // debug read E
  logic              e_re;
  logic [ROW_W-1:0]  e_tq;
  logic [COL_W-1:0]  e_tk;
  logic [DATA_W-1:0] e_rdata;
  logic              e_rvalid;

  // read P
  logic              p_re;
  logic [ROW_W-1:0]  p_tq;
  logic [COL_W-1:0]  p_tk;
  logic [DATA_W-1:0] p_rdata;
  logic              p_rvalid;

  attention_softmax_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(busy),
    .done(done),

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

    .e_re(e_re),
    .e_tq(e_tq),
    .e_tk(e_tk),
    .e_rdata(e_rdata),
    .e_rvalid(e_rvalid),

    .p_re(p_re),
    .p_tq(p_tq),
    .p_tk(p_tk),
    .p_rdata(p_rdata),
    .p_rvalid(p_rvalid)
  );

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
  // CPU write tasks (Q/K)
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

  // ----------------------------
  // Read E helper (wait e_rvalid)
  // ----------------------------
  task automatic read_e(input int tq_i, input int tk_i, output logic [31:0] bits);
    begin
      @(posedge clk);
      e_re <= 1'b1;
      e_tq <= tq_i[ROW_W-1:0];
      e_tk <= tk_i[COL_W-1:0];
      @(posedge clk);
      e_re <= 1'b0;

      while (!e_rvalid) @(posedge clk);
      bits = e_rdata;
    end
  endtask

  // ----------------------------
  // Read P helper (wait p_rvalid)
  // ----------------------------
  task automatic read_p(input int tq_i, input int tk_i, output logic [31:0] bits);
    begin
      @(posedge clk);
      p_re <= 1'b1;
      p_tq <= tq_i[ROW_W-1:0];
      p_tk <= tk_i[COL_W-1:0];
      @(posedge clk);
      p_re <= 1'b0;

      while (!p_rvalid) @(posedge clk);
      bits = p_rdata;
    end
  endtask

  // ----------------------------
  // Test
  // ----------------------------
  logic [31:0] E_mat [0:T-1][0:T-1];
  logic [31:0] P_mat [0:T-1][0:T-1];
initial begin
  int cyc = 0;
  while (cyc < 200000) begin
    @(posedge clk);
    cyc++;
  end
  $display("[TB] TIMEOUT! busy=%0d done=%0d", busy, done);
  $finish;
end

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

    e_re       = 1'b0;
    e_tq       = '0;
    e_tk       = '0;

    p_re       = 1'b0;
    p_tq       = '0;
    p_tk       = '0;

    // wait reset release
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    $display("[TB] Write Q/K ...");

    // deterministic pattern:
    // Q[t][d] = 1.0 + small unique bits
    // K[tk][d] = 1.0 + small unique bits
    for (int t=0; t<T; t++) begin
      for (int d=0; d<D_LEN; d++) begin
        cpu_write_q(t, d, 32'h3f800000 + (t<<8) + d);
      end
    end

    for (int t=0; t<T; t++) begin
      for (int d=0; d<D_LEN; d++) begin
        cpu_write_k(t, d, 32'h3f800000 + (t<<9) + (d<<1));
      end
    end

    $display("[TB] start=1 until done...");
    @(posedge clk);
    start <= 1'b1;

    // 等 busy 真的起來（可選，但很有用）
    while (!busy) @(posedge clk);

    // 等 done
    while (!done) @(posedge clk);

    // done 之後再放掉 start，讓 module 回到 IDLE re-arm
    @(posedge clk);
    start <= 1'b0;
    $display("[TB] done asserted.");


    // dump E matrix
    $display("");
    $display("==== EXP MATRIX E (hex) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        read_e(r,c,E_mat[r][c]);
        $write("%08x ", E_mat[r][c]);
      end
      $write("\n");
    end

    $display("");
    $display("==== EXP MATRIX E (real) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        $write("%f ", fp32_to_real(E_mat[r][c]));
      end
      $write("\n");
    end

    // dump P matrix
    $display("");
    $display("==== SOFTMAX MATRIX P (hex) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        read_p(r,c,P_mat[r][c]);
        $write("%08x ", P_mat[r][c]);
      end
      $write("\n");
    end

    $display("");
    $display("==== SOFTMAX MATRIX P (real) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        $write("%f ", fp32_to_real(P_mat[r][c]));
      end
      $write("\n");
    end

    // verify row sums
    $display("");
    $display("==== CHECK: row sum(P) should be ~1.0 ====");
    for (int r=0; r<T; r++) begin
      real s;
      s = 0.0;
      for (int c=0; c<T; c++) begin
        s += fp32_to_real(P_mat[r][c]);
      end
      $display("row %0d sum(P)= %f", r, s);
    end

    $display("[TB] finish.");
    $finish;
  end

endmodule

`default_nettype wire
