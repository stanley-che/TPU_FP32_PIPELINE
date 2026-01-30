/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_x.vvp ./test/tb_xtile_loader_top.sv
vvp ./vvp/tb_x.vvp
gtkwave ./vvp/tb_x.vcd

「當我用 CPU preload 一個矩陣到 X SRAM 之後，
loader 對每一個指定的 k（row index），
會依序把 n=0..N-1 的資料從 SRAM 正確讀出來，
而且控制流程（start → load → row_valid → accept）是正確的。」
*/

`include "./src/xtile_loader_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_xtile_loader_top;

  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // 1: 也檢查 X_tile port（Verilator/questa 通常 OK）
  // 0: 不檢查 X_tile，只檢查 SRAM stream（Icarus 最穩）
  localparam bit CHECK_XTILE_PORT = 1'b0;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  logic           start_k;
  logic [K_W-1:0] k_idx;
  wire            row_valid;
  logic           row_accept;

  // =========================
  // CPU preload port (replaces tb_*)
  // =========================
  logic              cpu_x_we;
  logic [K_W-1:0]    cpu_x_k;
  logic [N_W-1:0]    cpu_x_n;
  logic [DATA_W-1:0] cpu_x_wdata;
  logic [BYTE_W-1:0] cpu_x_wmask;

  wire [DATA_W-1:0] X_tile [KMAX][N];

  xtile_loader_top #(
    .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .row_valid(row_valid),
    .row_accept(row_accept),

    .cpu_x_we   (cpu_x_we),
    .cpu_x_k    (cpu_x_k),
    .cpu_x_n    (cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .X_tile(X_tile)
  );

  // ===== Golden matrix (只存你用到的 k 範圍即可) =====
  localparam int unsigned K_LO = 0;
  localparam int unsigned K_HI = 9;
  logic [DATA_W-1:0] golden [K_HI-K_LO+1][N]; // golden[k][n]

  // X 的 pattern：B0000000 + (k<<16) + n
  function automatic [DATA_W-1:0] mem_value_x(input int unsigned kk, input int unsigned nn);
    mem_value_x = 32'hB000_0000 + (kk << 16) + nn;
  endfunction

  // ===== tasks =====
  task automatic cpu_write(input int unsigned kk, input int unsigned nn, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_x_we    <= 1'b1;
      cpu_x_k     <= kk[K_W-1:0];
      cpu_x_n     <= nn[N_W-1:0];
      cpu_x_wdata <= data;
      cpu_x_wmask <= {BYTE_W{1'b1}}; // all bytes write
      @(negedge clk);
      cpu_x_we    <= 1'b0;

      if (kk >= K_LO && kk <= K_HI) golden[kk-K_LO][nn] = data;

      $display("[CPU_WRITE] k=%0d n=%0d data=%08x t=%0t", kk, nn, data, $time);
    end
  endtask

  task automatic init_sram_with_pattern(input int unsigned k_lo, input int unsigned k_hi);
    int unsigned kk, nn;
    begin
      for (kk=k_lo; kk<=k_hi; kk++) begin
        for (nn=0; nn<N; nn++) begin
          cpu_write(kk, nn, mem_value_x(kk, nn));
        end
      end
    end
  endtask

  task automatic pulse_start(input int unsigned kk);
    begin
      @(negedge clk);
      k_idx   <= kk[K_W-1:0];
      start_k <= 1'b1;
      @(negedge clk);
      start_k <= 1'b0;
    end
  endtask

  task automatic accept_row;
    begin
      @(negedge clk);
      row_accept <= 1'b1;
      @(negedge clk);
      row_accept <= 1'b0;
    end
  endtask

  // ===== 核心：row 驗證（固定 k，掃 n=0..N-1）=====
  task automatic run_and_check_row(input int unsigned kk);
    int unsigned got;
    int unsigned n_obs;
    logic [DATA_W-1:0] exp;
    begin
      $display("\n=== RUN k=%0d ===", kk);

      got = 0;
      pulse_start(kk);

      // 收滿 N 筆 SRAM return
      while (got < N) begin
        @(posedge clk);
        if (dut.x_rvalid) begin
          // 觀測 loader 此刻對應哪個 n（你 loader 內部要有 n_issued）
          n_obs = dut.u_loader.n_issued;
          exp   = mem_value_x(kk, n_obs);

          // 1) SRAM stream check
          if (dut.x_rdata !== exp) begin
            $display("[FAIL][SRAM_STREAM] k=%0d n=%0d rdata=%08x exp=%08x t=%0t",
                     kk, n_obs, dut.x_rdata, exp, $time);
            $fatal(1);
          end

          // 2) 可選：檢查 X_tile port 是否真的被寫入
          if (CHECK_XTILE_PORT) begin
            #1ps;
            if (X_tile[kk][n_obs] !== exp) begin
              $display("[FAIL][XTILE_PORT] k=%0d n=%0d X_tile=%08x exp=%08x t=%0t",
                       kk, n_obs, X_tile[kk][n_obs], exp, $time);
              $fatal(1);
            end
          end

          got++;
        end
      end

      // row_done handshake
      wait(row_valid === 1'b1);
      @(negedge clk);
      $display("[PASS] row done for k=%0d (beats=%0d) t=%0t", kk, got, $time);

      accept_row();
      wait(row_valid === 1'b0);
    end
  endtask

  // ===== debug probes =====
  always @(posedge clk) begin
    if (dut.x_en && dut.x_we) begin
      $display("[SRAM_WE] k=%0d n=%0d data=%08x mask=%0h t=%0t",
               dut.x_k, dut.x_n, dut.x_wdata, dut.x_wmask, $time);
    end
    if (dut.x_rvalid) begin
      $display("[SRAM_RD] rdata=%08x t=%0t", dut.x_rdata, $time);
    end
  end

  // ===== dump =====
  initial begin
    $dumpfile("./vvp/tb_x.vcd");
    $dumpvars(0, tb_xtile_loader_top);
  end

  // ===== main =====
  initial begin
    start_k    = 1'b0;
    k_idx      = '0;
    row_accept = 1'b0;

    cpu_x_we    = 1'b0;
    cpu_x_k     = '0;
    cpu_x_n     = '0;
    cpu_x_wdata = '0;
    cpu_x_wmask = '0;

    for (int kk=K_LO; kk<=K_HI; kk++) begin
      for (int nn=0; nn<N; nn++) begin
        golden[kk-K_LO][nn] = '0;
      end
    end

    // Phase 0：reset
    rst = 1'b1;
    repeat(5) @(posedge clk);
    rst = 1'b0;
    repeat(2) @(posedge clk);

    // Phase 1：CPU preload SRAM
    init_sram_with_pattern(K_LO, K_HI);
    repeat(5) @(posedge clk);

    // Phase 2：load rows
    run_and_check_row(0);
    run_and_check_row(7);
    run_and_check_row(3);
    run_and_check_row(9);

    $display("\nAll tests passed.");
    $finish;
  end

endmodule

`default_nettype wire
