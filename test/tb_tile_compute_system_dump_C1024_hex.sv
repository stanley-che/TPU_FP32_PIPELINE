    /*
    iverilog -g2012 -Wall -I./src \
    -o ./vvp/tb_dumpC.vvp \
    ./test/tb_tile_compute_system_dump_C1024_hex.sv

    vvp ./vvp/tb_dumpC.vvp
    */


`include "./src/EPU/attention_score/sramsa.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_tile_compute_system_e2e_hex;

  // ============================
  // DUT params
  // ============================
  localparam int unsigned M    = 64;
  localparam int unsigned N    = 64;
  localparam int unsigned KMAX = 64;

  // 你要跑的 K_len（例如 4 / 64 / 1024）
  localparam int unsigned KLEN = 64;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // ============================
  // clock/reset
  // ============================
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ============================
  // command
  // ============================
  logic        start;
  logic [15:0] K_len;
  logic        busy, done;

  // ============================
  // cpu writes
  // ============================
  logic                 cpu_w_we;
  logic [ROW_W-1:0]     cpu_w_row;
  logic [K_W-1:0]       cpu_w_k;
  logic [DATA_W-1:0]    cpu_w_wdata;
  logic [BYTE_W-1:0]    cpu_w_wmask;

  logic                 cpu_x_we;
  logic [K_W-1:0]       cpu_x_k;
  logic [N_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  // ============================
  // C SRAM CPU read
  // ============================
  logic              c_rd_en;
  logic              c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  // ============================
  // debug
  // ============================
  logic [M*N*DATA_W-1:0] c_out_flat_o;
  logic [M*N-1:0]        c_valid_flat_o;
  logic                  C_valid;

  logic [M*KMAX*DATA_W-1:0] W_tile_flat_dbg;
  logic [KMAX*N*DATA_W-1:0] X_tile_flat_dbg;

  // ============================
  // DUT
  // ============================
  tile_compute_system_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(1),
    .ROW_W(ROW_W), .COL_W(COL_W), .N_W(N_W), .K_W(K_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done),

    .cpu_w_we(cpu_w_we),
    .cpu_w_row(cpu_w_row),
    .cpu_w_k(cpu_w_k),
    .cpu_w_wdata(cpu_w_wdata),
    .cpu_w_wmask(cpu_w_wmask),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .c_out_flat_o(c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid(C_valid),

    .W_tile_flat_dbg(W_tile_flat_dbg),
    .X_tile_flat_dbg(X_tile_flat_dbg)
  );

  // ============================================================
  // HEX memories
  //   A.hex: M x KLEN  (row-major)  => A[r*KLEN + k]
  //   B.hex: KLEN x N  (row-major)  => B[k*N + n]
  //   C_golden.hex: M x N (row-major) => Cexp[r*N + n]
  // ============================================================
  logic [31:0] A_hex [0:M*KLEN-1];
  logic [31:0] B_hex [0:KLEN*N-1];
  logic [31:0] C_gld [0:M*N-1];

  function automatic int Aidx(input int r, input int k);
    return r*KLEN + k;
  endfunction
  function automatic int Bidx(input int k, input int n);
    return k*N + n;
  endfunction
  function automatic int Cidx(input int r, input int n);
    return r*N + n;
  endfunction

  // ============================================================
  // CPU write helpers
  // ============================================================
  task automatic cpu_write_w(input int unsigned row, input int unsigned kk, input logic [31:0] data);
    begin
      @(negedge clk);
      cpu_w_we    <= 1'b1;
      cpu_w_row   <= row[ROW_W-1:0];
      cpu_w_k     <= kk[K_W-1:0];
      cpu_w_wdata <= data;
      cpu_w_wmask <= {BYTE_W{1'b1}};
      @(negedge clk);
      cpu_w_we    <= 1'b0;
    end
  endtask

  task automatic cpu_write_x(input int unsigned kk, input int unsigned nn, input logic [31:0] data);
    begin
      @(negedge clk);
      cpu_x_we    <= 1'b1;
      cpu_x_k     <= kk[K_W-1:0];
      cpu_x_n     <= nn[N_W-1:0];
      cpu_x_wdata <= data;
      cpu_x_wmask <= {BYTE_W{1'b1}};
      @(negedge clk);
      cpu_x_we    <= 1'b0;
    end
  endtask

  // ============================================================
  // CPU read helper (C SRAM)
  // ============================================================
  task automatic cpu_read_c_word(input int r, input int c, output logic [31:0] data);
    int unsigned timeout;
    begin
      data = '0;

      @(negedge clk);
      c_rd_row <= r[ROW_W-1:0];
      c_rd_col <= c[COL_W-1:0];
      c_rd_re  <= 1'b1;

      @(negedge clk);
      c_rd_re  <= 1'b0;

      timeout = 0;
      while (c_rd_rvalid !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 5_000_000)
          $fatal(1, "[TB] TIMEOUT cpu_read_c_word r=%0d c=%0d", r, c);
      end

      #1;
      data = c_rd_rdata;

      @(posedge clk); // 1-cycle gap
    end
  endtask

  // ============================================================
  // Preload from hex
  // ============================================================
  task automatic preload_from_hex;
    int r, n, k;
    begin
      // A: M x KLEN
      for (k = 0; k < KLEN; k++) begin
        for (r = 0; r < M; r++) begin
          cpu_write_w(r, k, A_hex[Aidx(r,k)]);
        end
      end

      // B: KLEN x N
      for (k = 0; k < KLEN; k++) begin
        for (n = 0; n < N; n++) begin
          cpu_write_x(k, n, B_hex[Bidx(k,n)]);
        end
      end
    end
  endtask

  // ============================================================
  // Compare C SRAM vs golden hex
  // ============================================================
  task automatic check_c_vs_golden;
    int r, n;
    logic [31:0] got;
    int err;
    begin
      err = 0;
      for (r = 0; r < M; r++) begin
        for (n = 0; n < N; n++) begin
          cpu_read_c_word(r, n, got);

          if (got !== C_gld[Cidx(r,n)]) begin
            $display("[FAIL] C[%0d][%0d] got=%08h exp=%08h",
                     r, n, got, C_gld[Cidx(r,n)]);
            err++;
          end
        end
      end

      if (err != 0) begin
        $display("[TB] FAILED ❌ total mismatches = %0d", err);
        $fatal(1);
      end else begin
        $display("[TB] PASS ✅ C SRAM matched C_golden.hex");
      end
    end
  endtask

  // ============================================================
  // Wave dump
  // ============================================================
  /*
  initial begin
    $dumpfile("./vvp/tb_e2e_hex.vcd");
    $dumpvars(0, tb_tile_compute_system_e2e_hex);
  end*/

  // ============================================================
  // Main
  // ============================================================
  int unsigned timeout;

  initial begin
    // init
    rst   = 1'b1;
    start = 1'b0;
    K_len = KLEN[15:0];

    cpu_w_we='0; cpu_w_row='0; cpu_w_k='0; cpu_w_wdata='0; cpu_w_wmask='0;
    cpu_x_we='0; cpu_x_k='0;   cpu_x_n='0; cpu_x_wdata='0; cpu_x_wmask='0;

    c_rd_en='1; c_rd_re='0; c_rd_row='0; c_rd_col='0;

    // reset
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // load hex
    $display("[TB] readmemh A.hex / B.hex / C_golden.hex ...");
    $readmemh("A.hex", A_hex);
    $readmemh("B.hex", B_hex);
    $readmemh("C_golden.hex", C_gld);

    // preload
    $display("[TB] preload W/X from hex ...");
    preload_from_hex();

    // start
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    // wait done
    timeout = 0;
    while (done !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000) $fatal(1, "[TB] TIMEOUT waiting done");
    end
    $display("[TB] done seen. busy=%0b", busy);

    // wait C_valid
    timeout = 0;
    while (C_valid !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000) $fatal(1, "[TB] TIMEOUT waiting C_valid");
    end
    $display("[TB] C_valid seen.");

    // compare
    check_c_vs_golden();

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire

