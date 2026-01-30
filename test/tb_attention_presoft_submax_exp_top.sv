// ------------------------------------------------------------
// tb_attention_presoft_submax_exp_top.sv
// ------------------------------------------------------------
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_presoft_exp.vvp ./test/tb_attention_presoft_submax_exp_top.sv
//   vvp ./vvp/tb_presoft_exp.vvp
// ------------------------------------------------------------

`include "./src/EPU/attention_score/attention_presoft_submax_exp_top.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_attention_presoft_submax_exp_top;

  // ----------------------------
  // Params
  // ----------------------------
  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned TR_M   = 2;
  localparam int unsigned CONFLICT_POLICY_C = 1;

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX);
  localparam int unsigned ROW_W = T_W;
  localparam int unsigned COL_W = T_W;

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  logic clk = 0;
  logic rst_n = 0;
  always #5 clk = ~clk;

  initial begin
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic start;
  logic [15:0] D_len;
  logic busy, done;

  logic [T-1:0] pad_valid;
  logic causal_en;

  logic cpu_q_we;
  logic [T_W-1:0] cpu_q_t;
  logic [D_W-1:0] cpu_q_d;
  logic [DATA_W-1:0] cpu_q_wdata;
  logic [BYTE_W-1:0] cpu_q_wmask;

  logic cpu_k_we;
  logic [T_W-1:0] cpu_k_t;
  logic [D_W-1:0] cpu_k_d;
  logic [DATA_W-1:0] cpu_k_wdata;

  logic e_re;
  logic [ROW_W-1:0] e_tq;
  logic [COL_W-1:0] e_tk;
  logic [DATA_W-1:0] e_rdata;
  logic e_rvalid;
  // ---------------------------------
  // Deterministic CASE2 Q / K
  // ---------------------------------
  localparam int unsigned D_LEN = 4;

  logic [31:0] Q_bits [0:T-1][0:D_LEN-1];
  logic [31:0] K_bits [0:T-1][0:D_LEN-1];

  // ----------------------------
  // DUT
  // ----------------------------
  attention_presoft_submax_exp_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) dut (
    .*   // <== I/O 對齊，乾淨
  );

  // ----------------------------
  // FP32 -> real (print helper)
  // ----------------------------
  function automatic real fp32_to_real(input logic [31:0] b);
    int s, e, frac;
    real mant;
    begin
      s = b[31];
      e = b[30:23];
      frac = b[22:0];
      if (e == 0) mant = frac / 8388608.0;
      else        mant = 1.0 + frac / 8388608.0;
      fp32_to_real = (s ? -1.0 : 1.0) * mant * (2.0 ** (e - 127));
    end
  endfunction

  // ----------------------------
  // Write helpers
  // ----------------------------
  task automatic q_wr(int t, int d, logic [31:0] v);
    @(posedge clk);
    cpu_q_we <= 1;
    cpu_q_t  <= t[T_W-1:0];
    cpu_q_d  <= d[D_W-1:0];
    cpu_q_wdata <= v;
    cpu_q_wmask <= {BYTE_W{1'b1}};
    @(posedge clk);
    cpu_q_we <= 0;
  endtask

  task automatic k_wr(int t, int d, logic [31:0] v);
    @(posedge clk);
    cpu_k_we <= 1;
    cpu_k_t  <= t[T_W-1:0];
    cpu_k_d  <= d[D_W-1:0];
    cpu_k_wdata <= v;
    @(posedge clk);
    cpu_k_we <= 0;
  endtask

  function automatic logic [31:0] fp32_const(int t, int d, int off);
    fp32_const = 32'h3f800000 + (t<<10) + (d<<2) + off;
  endfunction

  // ----------------------------
  // Read E
  // ----------------------------
  task automatic e_rd(int tq, int tk, output logic [31:0] v);
    @(posedge clk);
    e_re <= 1;
    e_tq <= tq;
    e_tk <= tk;
    @(posedge clk);
    e_re <= 0;
    while (!e_rvalid) @(posedge clk);
    v = e_rdata;
  endtask

  task automatic dump_E;
    logic [31:0] b;
    begin
      $display("\n==== EXP(Y) ====");
      for (int r=0;r<T;r++) begin
        $write("row %0d : ", r);
        for (int c=0;c<T;c++) begin
          e_rd(r,c,b);
          $write("%f ", fp32_to_real(b));
        end
        $write("\n");
      end
    end
  endtask
  // ----------------------------
// Dump INPUT Q / K matrices
// ----------------------------
task automatic dump_QK;
  begin
    $display("\n==== INPUT Q MATRIX (real) ====");
    for (int r = 0; r < T; r++) begin
      $write("Q row %0d : ", r);
      for (int d = 0; d < D_LEN; d++) begin
        $write("%8.4f ", fp32_to_real(Q_bits[r][d]));
      end
      $write("\n");
    end

    $display("\n==== INPUT K MATRIX (real) ====");
    for (int r = 0; r < T; r++) begin
      $write("K row %0d : ", r);
      for (int d = 0; d < D_LEN; d++) begin
        $write("%8.4f ", fp32_to_real(K_bits[r][d]));
      end
      $write("\n");
    end
  end
endtask

  // ----------------------------
  // Stimulus
  // ----------------------------
  initial begin
    $dumpfile("tb_presoft_exp.vcd");
    $dumpvars(0, tb_attention_presoft_submax_exp_top);

    start = 0;
    D_len = 4;
    pad_valid = {T{1'b1}};
    causal_en = 0;

    cpu_q_we = 0;
    cpu_k_we = 0;
    e_re     = 0;

    @(posedge rst_n);
    repeat (5) @(posedge clk);

    // Write Q / K
    // clear all (for T > 4 safety)
    for (int r=0;r<T;r++)
      for (int d=0;d<D_LEN;d++) begin
        Q_bits[r][d] = 32'h0000_0000;
        K_bits[r][d] = 32'h0000_0000;
      end

      // ----------------------------
      // CASE2 Q (rows 0..3)
      // ----------------------------
      Q_bits[0][0]=32'hc0000000; Q_bits[0][1]=32'hbfc00000; Q_bits[0][2]=32'hbf800000; Q_bits[0][3]=32'hbf000000;
      Q_bits[1][0]=32'h00000000; Q_bits[1][1]=32'h3f000000; Q_bits[1][2]=32'h3f800000; Q_bits[1][3]=32'h3fc00000;
      Q_bits[2][0]=32'h40000000; Q_bits[2][1]=32'hc0000000; Q_bits[2][2]=32'hbfc00000; Q_bits[2][3]=32'hbf800000;
      Q_bits[3][0]=32'hbf000000; Q_bits[3][1]=32'h00000000; Q_bits[3][2]=32'h3f000000; Q_bits[3][3]=32'h3f800000;

      // ----------------------------
      // CASE2 K (rows 0..3)
      // ----------------------------
      K_bits[0][0]=32'hbfc00000; K_bits[0][1]=32'h00000000; K_bits[0][2]=32'h3fc00000; K_bits[0][3]=32'hbfc00000;
      K_bits[1][0]=32'h40000000; K_bits[1][1]=32'hbf800000; K_bits[1][2]=32'h3f000000; K_bits[1][3]=32'h40000000;
      K_bits[2][0]=32'h3f800000; K_bits[2][1]=32'hc0000000; K_bits[2][2]=32'hbf000000; K_bits[2][3]=32'h3f800000;
      K_bits[3][0]=32'h00000000; K_bits[3][1]=32'h3fc00000; K_bits[3][2]=32'hbfc00000; K_bits[3][3]=32'h00000000;

     // set D_len
    D_len = D_LEN[15:0];

    $display("[TB] Write Q ...");
for (int t=0; t<T; t++) begin
  for (int d=0; d<D_LEN; d++) begin
    @(posedge clk);
    cpu_q_we    <= 1'b1;
    cpu_q_t     <= t[T_W-1:0];
    cpu_q_d     <= d[D_W-1:0];
    cpu_q_wdata <= Q_bits[t][d];
    cpu_q_wmask <= {BYTE_W{1'b1}};
    @(posedge clk);
    cpu_q_we    <= 1'b0;
    cpu_q_wmask <= '0;
  end
end

$display("[TB] Write K ...");
for (int t=0; t<T; t++) begin
  for (int d=0; d<D_LEN; d++) begin
    @(posedge clk);
    cpu_k_we    <= 1'b1;
    cpu_k_t     <= t[T_W-1:0];   // ⚠️ slice width
    cpu_k_d     <= d[D_W-1:0];
    cpu_k_wdata <= K_bits[t][d];
    @(posedge clk);
    cpu_k_we    <= 1'b0;
  end
end
   dump_QK();   // <<< ★這一行就是關鍵

    @(posedge clk);
    start <= 1;
    @(posedge clk);
    start <= 0;

    while (!done) @(posedge clk);

    dump_E();

    $finish;
  end

endmodule
`default_nettype wire
