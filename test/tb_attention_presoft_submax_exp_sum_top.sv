// ============================================================================
// tb_attention_presoft_submax_exp_sum_top.sv
// Run:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_sum.vvp ./test/tb_attention_presoft_submax_exp_sum_top.sv
//   vvp ./vvp/tb_sum.vvp
// ============================================================================
`include "./src/EPU/attention_score/attention_presoft_submax_exp_sum_top.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_attention_presoft_submax_exp_sum_top;

  // ----------------------------
  // Params (debug small)
  // ----------------------------
  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 8;
  localparam int unsigned D_LEN  = 3;

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

  // E read
  logic              e_re;
  logic [ROW_W-1:0]  e_tq;
  logic [COL_W-1:0]  e_tk;
  logic [DATA_W-1:0] e_rdata;
  logic              e_rvalid;

  // Sum read
  logic              sum_re;
  logic [ROW_W-1:0]  sum_row;
  logic [DATA_W-1:0] sum_rdata;
  logic              sum_rvalid;

  attention_presoft_submax_exp_sum_top #(
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

    .sum_re(sum_re),
    .sum_row(sum_row),
    .sum_rdata(sum_rdata),
    .sum_rvalid(sum_rvalid)
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
  // Read E[tq][tk] helper (1-cycle valid)
  // ----------------------------
  task automatic read_e(input int tq, input int tk, output logic [31:0] bits);
    begin
      @(posedge clk);
      e_re <= 1'b1;
      e_tq <= tq[ROW_W-1:0];
      e_tk <= tk[COL_W-1:0];
      @(posedge clk);
      e_re <= 1'b0;

      // wait rvalid
      while (!e_rvalid) @(posedge clk);
      bits = e_rdata;
    end
  endtask

  // ----------------------------
  // Read Sum[row] helper (1-cycle valid)
  // ----------------------------
  task automatic read_sum(input int r, output logic [31:0] bits);
    begin
      @(posedge clk);
      sum_re  <= 1'b1;
      sum_row <= r[ROW_W-1:0];
      @(posedge clk);
      sum_re <= 1'b0;

      while (!sum_rvalid) @(posedge clk);
      bits = sum_rdata;
    end
  endtask
  task automatic print_Q;
  begin
    $display("\n==== Q (T=%0d, D=%0d) ====", T, D_LEN);
    for (int r=0;r<T;r++) begin
      $write("row %0d : ", r);
      for (int c=0;c<D_LEN;c++) $write("%f ", fp32_to_real(Q_bits[r][c]));
      $write("\n");
    end
  end
endtask
task automatic print_K;
  begin
    $display("\n==== K (T=%0d, D=%0d) ====", T, D_LEN);
    for (int r=0;r<T;r++) begin
      $write("row %0d : ", r);
      for (int c=0;c<D_LEN;c++) $write("%f ", fp32_to_real(K_bits[r][c]));
      $write("\n");
    end
  end
endtask
  // ----------------------------
  // Test
  // ----------------------------
  logic [31:0] E_mat [0:T-1][0:T-1];
  logic [31:0] S_vec [0:T-1];
logic [31:0] Q_bits [0:T-1][0:D_LEN-1];
logic [31:0] K_bits [0:T-1][0:D_LEN-1];
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

    sum_re     = 1'b0;
    sum_row    = '0;

    // wait reset release
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    $display("[TB] Write Q/K ...");

    // deterministic pattern: Q[t][d] = 1.0 + 0.01*t + 0.001*d
    // and K[tk][d] = 1.0 + 0.02*tk + 0.002*d
    // We'll encode as FP32 bits using common hex constants for small set:
    // For simplicity, use the same FP32 pattern you used earlier if you have one.
    // Here: just fill with 1.0 (0x3f800000) + small unique LSB pattern (won't be real-float exact but good debug)
    /*for (int t=0; t<T; t++) begin
      for (int d=0; d<D_LEN; d++) begin
        cpu_write_q(t, d, 32'h3f800000 + (t<<8) + d);
      end
    end*/
    $display("=== Write Q (>1) ===");
for (int i = 0; i < T; i++) begin
  for (int j = 0; j < D_LEN; j++) begin
    logic [31:0] bits;
    case (i*D_LEN + j + 1)
      1:  bits = 32'h40000000; // 2
      2:  bits = 32'h40400000; // 3
      3:  bits = 32'h40800000; // 4
      default: bits = 32'h40000000;  // 2
    endcase

    Q_bits[i][j] = bits;
    cpu_write_q(i, j, bits);
  end
end
 // ----------------------------
// Write K (T x D_LEN), all > 1
// ----------------------------
$display("=== Write K (>1) ===");
for (int i = 0; i < T; i++) begin
  for (int j = 0; j < D_LEN; j++) begin
    logic [31:0] bits;
    case (i*D_LEN + j + 1)
      1:  bits = 32'h3fc00000; // 1.5
      2:  bits = 32'h40000000; // 2
      3:  bits = 32'h40200000; // 2.5
      default: bits = 32'h3fc00000; // 1.5
    endcase

    K_bits[i][j] = bits;
    cpu_write_k(i, j, bits);
  end
end

    $display("[TB] pulse start...");
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;
   
    // wait done
    while (!done) @(posedge clk);
    $display("[TB] done asserted.");
print_Q();
 print_K();
    // dump E matrix
    $display("");
    $display("==== EXP MATRIX E = exp(Y) (hex) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        read_e(r,c,E_mat[r][c]);
        $write("%08x ", E_mat[r][c]);
      end
      $write("\n");
    end

    $display("");
    $display("==== EXP MATRIX E = exp(Y) (real) ====");
    for (int r=0; r<T; r++) begin
      $write("row %0d : ", r);
      for (int c=0; c<T; c++) begin
        $write("%f ", fp32_to_real(E_mat[r][c]));
      end
      $write("\n");
    end

    // dump Sum vector
    $display("");
    $display("==== SUM VECTOR Sum[row] = Σ_k E[row][k] (hex/real) ====");
    for (int r=0; r<T; r++) begin
      read_sum(r, S_vec[r]);
      $display("row %0d sum = %08x  (%f)", r, S_vec[r], fp32_to_real(S_vec[r]));
    end

    $display("[TB] finish.");
    $finish;
  end

endmodule

`default_nettype wire
