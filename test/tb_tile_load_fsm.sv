// tb_tile_load_fsm_fixedexp.sv
// iverilog -g2012 -Wall -I. -o ./vvp/tb_load.vvp ./test/tb_tile_load_fsm.sv
// vvp ./vvp/tb_load.vvp

`include "./src/tile_load_fsm.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_tile_load_fsm_fixedexp;

  localparam int M    = 8;
  localparam int N    = 8;
  localparam int KMAX = 1024;
  localparam int AW   = 10;
  localparam int KLEN = 4;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ---------------- DUT I/O ----------------
  logic        load_start;
  logic [15:0] K_len;
  wire         load_busy;
  wire         load_done;

  wire         w_ext_re   [M];
  wire [AW-1:0] w_ext_addr [M];
  logic [31:0]  w_ext_rdata [M];
  logic         w_ext_rvalid[M];

  wire         x_ext_re   [N];
  wire [AW-1:0] x_ext_addr [N];
  logic [31:0]  x_ext_rdata [N];
  logic         x_ext_rvalid[N];

  wire [31:0]   W_tile [M][KMAX];
  wire [31:0]   X_tile [KMAX][N];

  tile_load_fsm #(.M(M),.N(N),.KMAX(KMAX),.AW(AW)) dut (
    .clk(clk),
    .rst(rst),
    .load_start(load_start),
    .K_len(K_len),
    .load_busy(load_busy),
    .load_done(load_done),

    .w_ext_re(w_ext_re),
    .w_ext_addr(w_ext_addr),
    .w_ext_rdata(w_ext_rdata),
    .w_ext_rvalid(w_ext_rvalid),

    .x_ext_re(x_ext_re),
    .x_ext_addr(x_ext_addr),
    .x_ext_rdata(x_ext_rdata),
    .x_ext_rvalid(x_ext_rvalid),

    .W_tile(W_tile),
    .X_tile(X_tile)
  );

  // ============================================================
  // Simple banked SRAM models (1-cycle latency)
  // W_mem[m][k], X_mem[n][k]
  // ============================================================
  logic [31:0] W_mem [M][0:KMAX-1];
  logic [31:0] X_mem [N][0:KMAX-1];

  logic        w_re_q   [M];
  logic [AW-1:0] w_addr_q[M];
  logic        x_re_q   [N];
  logic [AW-1:0] x_addr_q[N];

  integer i;

  always_ff @(posedge clk) begin
    if (rst) begin
      for (i=0;i<M;i++) begin
        w_re_q[i]      <= 1'b0;
        w_addr_q[i]    <= '0;
        w_ext_rvalid[i]<= 1'b0;
        w_ext_rdata[i] <= '0;
      end
      for (i=0;i<N;i++) begin
        x_re_q[i]      <= 1'b0;
        x_addr_q[i]    <= '0;
        x_ext_rvalid[i]<= 1'b0;
        x_ext_rdata[i] <= '0;
      end
    end else begin
      // stage0 capture request
      for (i=0;i<M;i++) begin
        w_re_q[i]   <= w_ext_re[i];
        w_addr_q[i] <= w_ext_addr[i];
      end
      for (i=0;i<N;i++) begin
        x_re_q[i]   <= x_ext_re[i];
        x_addr_q[i] <= x_ext_addr[i];
      end

      // stage1 respond
      for (i=0;i<M;i++) begin
        w_ext_rvalid[i] <= w_re_q[i];
        if (w_re_q[i]) w_ext_rdata[i] <= W_mem[i][w_addr_q[i]];
      end
      for (i=0;i<N;i++) begin
        x_ext_rvalid[i] <= x_re_q[i];
        if (x_re_q[i]) x_ext_rdata[i] <= X_mem[i][x_addr_q[i]];
      end
    end
  end

  // ============================================================
  // Init fixed patterns (same as your systolic_wrap TB)
  // W_mem[m][k] = aval[(k+m)%4]
  // X_mem[n][k] = bval[(k+n)%4]
  // ============================================================
  task automatic init_fixed_mem;
    int m, n, k;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      aval[0] = 32'h3f800000; // 1.0
      aval[1] = 32'h40000000; // 2.0
      aval[2] = 32'h3f000000; // 0.5
      aval[3] = 32'h40400000; // 3.0

      bval[0] = 32'h3f800000; // 1.0
      bval[1] = 32'h3f000000; // 0.5
      bval[2] = 32'h40000000; // 2.0
      bval[3] = 32'h40400000; // 3.0

      // clear
      for (m=0;m<M;m++)
        for (k=0;k<KMAX;k++)
          W_mem[m][k] = 32'h0;

      for (n=0;n<N;n++)
        for (k=0;k<KMAX;k++)
          X_mem[n][k] = 32'h0;

      // fill only KLEN
      for (k=0;k<KLEN;k++) begin
        for (m=0;m<M;m++) W_mem[m][k] = aval[(k + m) % 4];
        for (n=0;n<N;n++) X_mem[n][k] = bval[(k + n) % 4];
      end

      $display("[TB] init_fixed_mem done (KLEN=%0d)", KLEN);
    end
  endtask

  // ============================================================
  // Dump tiles (only KLEN)
  // ============================================================
  task automatic dump_tiles;
    int m, n, k;
    begin
      $display("========== W_tile (M x KLEN) ==========");
      for (m=0;m<M;m++) begin
        $write("W_row %0d :", m);
        for (k=0;k<KLEN;k++) $write(" %08h", W_tile[m][k]);
        $write("\n");
      end
      $display("========== X_tile (KLEN x N) ==========");
      for (k=0;k<KLEN;k++) begin
        $write("X_k %0d   :", k);
        for (n=0;n<N;n++) $write(" %08h", X_tile[k][n]);
        $write("\n");
      end
      $display("=======================================");
    end
  endtask

  // ============================================================
  // Check: W_tile vs W_mem, X_tile vs X_mem
  // ============================================================
  task automatic check_tiles;
    int m, n, k;
    logic [31:0] got, exp;
    begin
      // wait a couple cycles for safety
      @(posedge clk);
      @(posedge clk);

      for (m=0;m<M;m++) begin
        for (k=0;k<KLEN;k++) begin
          got = W_tile[m][k];
          exp = W_mem[m][k];
          if (got !== exp) begin
            $display("[TB] FAIL W_tile[%0d][%0d] got=%08h exp=%08h", m, k, got, exp);
            $fatal(1);
          end
        end
      end

      for (k=0;k<KLEN;k++) begin
        for (n=0;n<N;n++) begin
          got = X_tile[k][n];
          exp = X_mem[n][k];
          if (got !== exp) begin
            $display("[TB] FAIL X_tile[%0d][%0d] got=%08h exp=%08h", k, n, got, exp);
            $fatal(1);
          end
        end
      end

      $display("[TB] PASS ✅ tiles matched expected memory contents");
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    rst        = 1'b1;
    load_start = 1'b0;
    K_len      = KLEN;

    init_fixed_mem();

    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: tile_load_fsm  M=%0d N=%0d KLEN=%0d ===", M, N, KLEN);

    // pulse load_start
    @(posedge clk);
    load_start = 1'b1;
    @(posedge clk);
    load_start = 1'b0;

    // wait load_done
    begin : WAIT_DONE
      int unsigned t;
      t = 0;
      while (load_done !== 1'b1) begin
        @(posedge clk);
        t++;
        if (t > 1_000_000) $fatal(1, "[TB] TIMEOUT waiting load_done");
      end
    end

    $display("[TB] load_done seen. load_busy=%0b @%0t", load_busy, $time);

    dump_tiles();
    check_tiles();

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
