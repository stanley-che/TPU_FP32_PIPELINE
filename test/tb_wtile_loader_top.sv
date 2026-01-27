/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_top.vvp ./test/tb_wtile_loader_top.sv
vvp ./vvp/tb_top.vvp
gtkwave ./vvp/tb_top.vcd

「當我用 CPU port preload 一個矩陣到 SRAM 之後，
loader 對每一個指定的 k（column），
會依序把 row=0..M-1 的資料從 SRAM 正確讀出來，
而且控制流程（start → load → col_valid → accept）是正確的。」
*/

`include "./src/wtile_loader_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_wtile_loader_top_cpu;

  localparam int unsigned M      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // 1: 也檢查 W_tile port（Verilator/questa 通常 OK）
  // 0: 不檢查 W_tile，只檢查 SRAM 回來的 stream（Icarus 最穩）
  localparam bit CHECK_WTILE_PORT = 1'b0;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  logic           start_k;
  logic [K_W-1:0] k_idx;
  wire            col_valid;
  logic           col_accept;

  // CPU write port (replaces tb_*)
  logic              cpu_w_we;
  logic [ROW_W-1:0]  cpu_w_row;
  logic [K_W-1:0]    cpu_w_k;
  logic [DATA_W-1:0] cpu_w_wdata;
  logic [BYTE_W-1:0] cpu_w_wmask;

  wire [DATA_W-1:0] W_tile [M][KMAX];

  wtile_loader_top #(
    .M(M), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .col_valid(col_valid),
    .col_accept(col_accept),

    // CPU ports
    .cpu_w_we   (cpu_w_we),
    .cpu_w_row  (cpu_w_row),
    .cpu_w_k    (cpu_w_k),
    .cpu_w_wdata(cpu_w_wdata),
    .cpu_w_wmask(cpu_w_wmask),

    .W_tile(W_tile)
  );

  // ===== Golden matrix (只存你用到的 k 範圍即可) =====
  localparam int unsigned K_LO = 0;
  localparam int unsigned K_HI = 9;
  logic [DATA_W-1:0] golden [M][K_HI-K_LO+1];

  function automatic [DATA_W-1:0] mem_value(input int unsigned row, input int unsigned kk);
    mem_value = 32'hA000_0000 + (row << 16) + kk;
  endfunction

  // ===== CPU write task =====
  task automatic cpu_write(input int unsigned row, input int unsigned kk, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_w_we    <= 1'b1;
      cpu_w_row   <= row[ROW_W-1:0];
      cpu_w_k     <= kk[K_W-1:0];
      cpu_w_wdata <= data;
      cpu_w_wmask <= {BYTE_W{1'b1}}; // all bytes write
      @(negedge clk);
      cpu_w_we    <= 1'b0;

      // 更新 golden
      if (kk >= K_LO && kk <= K_HI) golden[row][kk-K_LO] = data;

      $display("[CPU_WRITE] row=%0d k=%0d data=%08x t=%0t", row, kk, data, $time);
    end
  endtask

  task automatic init_sram_with_pattern(input int unsigned k_lo, input int unsigned k_hi);
    int unsigned r, k;
    begin
      for (r=0; r<M; r++) begin
        for (k=k_lo; k<=k_hi; k++) begin
          cpu_write(r, k, mem_value(r,k));
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

  task automatic accept_col;
    begin
      @(negedge clk);
      col_accept <= 1'b1;
      @(negedge clk);
      col_accept <= 1'b0;
    end
  endtask

  // ===== 核心：column 驗證 =====
  task automatic run_and_check_column(input int unsigned kk);
    int unsigned got;
    int unsigned r_obs;
    logic [DATA_W-1:0] exp;
    begin
      $display("\n=== RUN k=%0d ===", kk);

      got = 0;
      pulse_start(kk);

      // 收滿 M 筆 SRAM return
      while (got < M) begin
        @(posedge clk);
        if (dut.w_rvalid) begin
          // 觀測 loader 此刻對應哪個 row
          r_obs = dut.u_loader.row_issued;
          exp   = mem_value(r_obs, kk);

          // 1) 檢查 SRAM stream
          if (dut.w_rdata !== exp) begin
            $display("[FAIL][SRAM_STREAM] k=%0d row=%0d rdata=%08x exp=%08x t=%0t",
                     kk, r_obs, dut.w_rdata, exp, $time);
            $fatal(1);
          end

          // 2) 可選：檢查 W_tile port
          if (CHECK_WTILE_PORT) begin
            #1ps;
            if (W_tile[r_obs][kk] !== exp) begin
              $display("[FAIL][WTILE_PORT] k=%0d row=%0d W_tile=%08x exp=%08x t=%0t",
                       kk, r_obs, W_tile[r_obs][kk], exp, $time);
              $fatal(1);
            end
          end

          got++;
        end
      end

      // 等 col_valid 拉高 -> done
      wait(col_valid === 1'b1);
      @(negedge clk);
      $display("[PASS] column done for k=%0d (beats=%0d) t=%0t", kk, got, $time);

      accept_col();
      wait(col_valid === 1'b0);
    end
  endtask

  // ===== debug probes =====
  always @(posedge clk) begin
    if (dut.w_en && dut.w_we) begin
      $display("[SRAM_WE] row=%0d k=%0d data=%08x mask=%0h t=%0t",
               dut.w_row, dut.w_k, dut.w_wdata, dut.w_wmask, $time);
    end
    if (dut.w_rvalid) begin
      $display("[SRAM_RD] rdata=%08x t=%0t", dut.w_rdata, $time);
    end
  end

  initial begin
    $dumpfile("./vvp/tb_top.vcd");
    $dumpvars(0, tb_wtile_loader_top_cpu);
  end

  initial begin
    start_k    = 1'b0;
    k_idx      = '0;
    col_accept = 1'b0;

    cpu_w_we    = 1'b0;
    cpu_w_row   = '0;
    cpu_w_k     = '0;
    cpu_w_wdata = '0;
    cpu_w_wmask = '0;

    // golden init
    for (int r=0; r<M; r++) begin
      for (int kk=K_LO; kk<=K_HI; kk++) begin
        golden[r][kk-K_LO] = '0;
      end
    end

    // Phase 0: reset
    rst = 1'b1;
    repeat(5) @(posedge clk);
    rst = 1'b0;
    repeat(2) @(posedge clk);

    // Phase 1: CPU preload SRAM
    init_sram_with_pattern(K_LO, K_HI);
    repeat(5) @(posedge clk);

    // Phase 2: run loader for selected k
    run_and_check_column(0);
    run_and_check_column(7);
    run_and_check_column(3);
    run_and_check_column(9);

    $display("\nAll tests passed.");
    $finish;
  end

endmodule

`default_nettype wire
