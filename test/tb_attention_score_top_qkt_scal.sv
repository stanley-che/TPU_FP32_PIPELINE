// tb_attention_score_top_qkt_scal.sv
// ------------------------------------------------------------
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_qkt_scale.vvp ./test/tb_attention_score_top_qkt_scal.sv
// vvp ./vvp/tb_att_qkt_scale.vvp
// ------------------------------------------------------------

`include "./src/EPU/attention_score/attention_score_top_qkt_scale.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_score_top_qkt_scale;

  localparam int unsigned T      = 4;
  localparam int unsigned DMAX   = 16;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned TR_M   = 2;
  localparam int unsigned CONFLICT_POLICY_C = 1;

  localparam int unsigned T_W = (T<=1)?1:$clog2(T);
  localparam int unsigned D_W = (DMAX<=1)?1:$clog2(DMAX);

  // ----------------------------
  // clock/reset
  // ----------------------------
  logic clk, rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
  end

  // ----------------------------
  // DUT ports
  // ----------------------------
  logic        start;
  logic [15:0] D_len;
  logic        busy, done;

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

  // CPU read scaled score
  logic                 sc_re;
  logic [T_W-1:0]       sc_tq;
  logic [T_W-1:0]       sc_tk;
  logic [DATA_W-1:0]    sc_rdata;
  logic                 sc_rvalid;

  attention_score_top_qkt_scale #(
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

    .start(start),
    .D_len(D_len),
    .busy(busy),
    .done(done),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .sc_re(sc_re),
    .sc_tq(sc_tq),
    .sc_tk(sc_tk),
    .sc_rdata(sc_rdata),
    .sc_rvalid(sc_rvalid)
  );

  // ----------------------------
  // local copies for printing
  // ----------------------------
  logic [31:0] Q_bits [0:T-1][0:DMAX-1];
  logic [31:0] K_bits [0:T-1][0:DMAX-1];

  // ------------------------------------------------------------
  // helpers: fp32 -> real (iverilog-safe)
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

  // ------------------------------------------------------------
  // CPU write tasks
  // ------------------------------------------------------------
  task automatic q_write(input int t, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_q_we    <= 1'b1;
      cpu_q_t     <= t[T_W-1:0];
      cpu_q_d     <= d[D_W-1:0];
      cpu_q_wdata <= bits;
      cpu_q_wmask <= {BYTE_W{1'b1}}; // active-low mask 的話改成 '0
      @(posedge clk);
      cpu_q_we <= 1'b0;
    end
  endtask

  task automatic k_write(input int t, input int d, input logic [31:0] bits);
    begin
      @(posedge clk);
      cpu_k_we    <= 1'b1;
      cpu_k_t     <= t;
      cpu_k_d     <= d;
      cpu_k_wdata <= bits;
      @(posedge clk);
      cpu_k_we <= 1'b0;
    end
  endtask

  // ------------------------------------------------------------
  // scaled score read helper
  // ------------------------------------------------------------
  task automatic sc_read_one(input int tq, input int tk, output logic [31:0] bits);
    begin
      @(posedge clk);
      sc_re <= 1'b1;
      sc_tq <= tq[T_W-1:0];
      sc_tk <= tk[T_W-1:0];

      @(posedge clk);
      sc_re <= 1'b0;

      while (sc_rvalid !== 1'b1) @(posedge clk);
      bits = sc_rdata;
    end
  endtask

  // ------------------------------------------------------------
  // print Q and K as table (NO array ports => iverilog OK)
  // ------------------------------------------------------------
  task automatic print_Q_table(input int cols);
    int r, c;
    real v;
    begin
      $display("=== Q (float) T x D_LEN=%0d ===", cols);
      for (r = 0; r < T; r++) begin
        $write("Q[%0d]:", r);
        for (c = 0; c < cols; c++) begin
          v = fp32_to_real(Q_bits[r][c]);
          $write("   %8.4f", v);
        end
        $write("\n");
      end
      $display("");
    end
  endtask

  task automatic print_K_table(input int cols);
    int r, c;
    real v;
    begin
      $display("=== K (float) T x D_LEN=%0d ===", cols);
      for (r = 0; r < T; r++) begin
        $write("K[%0d]:", r);
        for (c = 0; c < cols; c++) begin
          v = fp32_to_real(K_bits[r][c]);
          $write("   %8.4f", v);
        end
        $write("\n");
      end
      $display("");
    end
  endtask

  // ------------------------------------------------------------
  // print SC as hex/float table
  // ------------------------------------------------------------
  task automatic print_SC_table;
    int r, c;
    logic [31:0] b;
    real v;
    begin
      $display("=== SC (hex/float) T x T ===");
      for (r = 0; r < T; r++) begin
        $write("SC[%0d]:", r);
        for (c = 0; c < T; c++) begin
          sc_read_one(r, c, b);
          v = fp32_to_real(b);
          $write("  %08h(%7.4f)", b, v);
        end
        $write("\n");
      end
      $display("");
    end
  endtask

  // ------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------
  int i, j;
  logic [31:0] bits;

  initial begin
    // defaults
    start      = 1'b0;
    D_len      = 16'd4;

    cpu_q_we   = 1'b0;
    cpu_q_t    = '0;
    cpu_q_d    = '0;
    cpu_q_wdata= '0;
    cpu_q_wmask= {BYTE_W{1'b1}};

    cpu_k_we   = 1'b0;
    cpu_k_t    = '0;
    cpu_k_d    = '0;
    cpu_k_wdata= '0;

    sc_re      = 1'b0;
    sc_tq      = '0;
    sc_tk      = '0;

    // init local matrices
    for (i = 0; i < T; i++) begin
      for (j = 0; j < DMAX; j++) begin
        Q_bits[i][j] = 32'h00000000;
        K_bits[i][j] = 32'h00000000;
      end
    end

    // wait reset release
    @(posedge rst_n);
    repeat (5) @(posedge clk);

    // ----------------------------
    // Write Q
    // ----------------------------
    $display("=== Write Q (T x D_len) ===");
    for (i = 0; i < T; i++) begin
      for (j = 0; j < D_len; j++) begin
        case (i*D_len + j + 1)
          1:  bits = 32'h3f56c8b4;
          2:  bits = 32'h3d5b8bac;
          3:  bits = 32'h3d5b8bac;
          4:  bits = 32'h3d5b8bac;
          5:  bits = 32'h3f195810;
          6:  bits = 32'h3e09374c;
          7:  bits = 32'h3e09374c;
          8:  bits = 32'h3e09374c;
          9:  bits = 32'h3f195810;
          10: bits = 32'h3e09374c;
          11: bits = 32'h3e09374c;
          12: bits = 32'h3e09374c;
          13: bits = 32'h3f195810;
          14: bits = 32'h3e09374c;
          15: bits = 32'h3e09374c;
          default: bits = 32'h3e09374c;
        endcase
        Q_bits[i][j] = bits;
        q_write(i, j, bits);
      end
    end

    // ----------------------------
    // Write K (你要的非 I)
    // K[i][j] = j+1 (1..4)
    // ----------------------------
    $display("=== Write K (T x D_len) pattern 1..4 per row ===");
    for (i = 0; i < T; i++) begin
      for (j = 0; j < D_len; j++) begin
        case (j+1)
          1: bits = 32'h3fc00000; //1
          9: bits = 32'h40000000; //2
          3: bits = 32'h40400000; //3
          default: bits = 32'h40000000; //4
        endcase
        K_bits[i][j] = bits;
        k_write(i, j, bits);
      end
    end

    // print like your screenshot
    print_Q_table(D_len);
    print_K_table(D_len);

    // ----------------------------
    // start
    // ----------------------------
    $display("=== START QKT + SCALE ===");
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    $display("Waiting for done...");
    while (done !== 1'b1) @(posedge clk);
    $display("DONE asserted!");

    // ----------------------------
    // print SC
    // ----------------------------
    $display("[TB] Read Scaled Score ...");
    print_SC_table();

    $display("TEST DONE");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
