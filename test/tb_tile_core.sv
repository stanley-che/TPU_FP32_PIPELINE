// tb_tile_top_3cases_fixedexp.sv
/*
iverilog -g2012 -Wall -I. -o ./vvp/tb_tile_top_3cases.vvp ./test/tb_tile_core.sv
vvp ./vvp/tb_tile_top_3cases.vvp
*/

`include "./src/tile_core.sv"          // 你貼的 tile_top（內含 include wrap + axi_sram_slave）
`timescale 1ns/1ps
`default_nettype none

// ============================================================
// One parameterized testcase for tile_top (SRAM host write -> load_start -> compute_start)
// Expected = slice from 8x8 base expected (your original table)
// ============================================================
module tb_tile_top_case_fixedexp #(
  parameter int M    = 8,
  parameter int N    = 8,
  parameter int KMAX = 4,
  parameter int KLEN = 4,
  parameter string NAME = "CASE"
)();

  localparam int W_ROW_W = (M<=1)?1:$clog2(M);
  localparam int X_ROW_W = (N<=1)?1:$clog2(N);
  localparam int K_W     = (KMAX<=1)?1:$clog2(KMAX);

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ---------------- Host ports to SRAM (W/X) ----------------
  logic ww_en, ww_re, ww_we;
  logic [W_ROW_W-1:0] ww_row;
  logic [K_W-1:0]     ww_k;
  logic [31:0]        ww_wdata;
  logic [3:0]         ww_wmask;
  wire  [31:0]        ww_rdata;
  wire                ww_rvalid;

  logic wx_en, wx_re, wx_we;
  logic [X_ROW_W-1:0] wx_row;
  logic [K_W-1:0]     wx_k;
  logic [31:0]        wx_wdata;
  logic [3:0]         wx_wmask;
  wire  [31:0]        wx_rdata;
  wire                wx_rvalid;

  // ---------------- Commands ----------------
  logic        load_start;
  logic        compute_start;
  logic [15:0] K_len;

  // ---------------- Status/Outputs ----------------
  wire load_busy, load_done;
  wire busy, done;
  wire C_valid;
  wire [31:0] C_tile [M][N];

  // ---------------- DUT ----------------
  tile_top #(.M(M), .N(N), .KMAX(KMAX)) dut (
    .clk(clk),
    .rst(rst),

    .ww_en(ww_en), .ww_re(ww_re), .ww_we(ww_we),
    .ww_row(ww_row), .ww_k(ww_k),
    .ww_wdata(ww_wdata), .ww_wmask(ww_wmask),
    .ww_rdata(ww_rdata), .ww_rvalid(ww_rvalid),

    .wx_en(wx_en), .wx_re(wx_re), .wx_we(wx_we),
    .wx_row(wx_row), .wx_k(wx_k),
    .wx_wdata(wx_wdata), .wx_wmask(wx_wmask),
    .wx_rdata(wx_rdata), .wx_rvalid(wx_rvalid),

    .load_start(load_start),
    .compute_start(compute_start),
    .K_len(K_len),

    .load_busy(load_busy),
    .load_done(load_done),
    .busy(busy),
    .done(done),
    .C_valid(C_valid),
    .C_tile(C_tile)
  );

  // ============================================================
  // Base expected table: 8x8 KLEN=4 (your original)
  // ============================================================
  localparam int BASE_M = 8;
  localparam int BASE_N = 8;
  logic [31:0] C_exp_8x8 [0:BASE_M*BASE_N-1];

  task automatic init_base_8x8_expected;
    begin
      // Row 0
      C_exp_8x8[0]  = 32'h41400000; C_exp_8x8[1]  = 32'h41100000; C_exp_8x8[2]  = 32'h41200000; C_exp_8x8[3]  = 32'h41340000;
      C_exp_8x8[4]  = 32'h41400000; C_exp_8x8[5]  = 32'h41100000; C_exp_8x8[6]  = 32'h41200000; C_exp_8x8[7]  = 32'h41340000;
      // Row 1
      C_exp_8x8[8]  = 32'h41340000; C_exp_8x8[9]  = 32'h41400000; C_exp_8x8[10] = 32'h41100000; C_exp_8x8[11] = 32'h41200000;
      C_exp_8x8[12] = 32'h41340000; C_exp_8x8[13] = 32'h41400000; C_exp_8x8[14] = 32'h41100000; C_exp_8x8[15] = 32'h41200000;
      // Row 2
      C_exp_8x8[16] = 32'h41200000; C_exp_8x8[17] = 32'h41340000; C_exp_8x8[18] = 32'h41400000; C_exp_8x8[19] = 32'h41100000;
      C_exp_8x8[20] = 32'h41200000; C_exp_8x8[21] = 32'h41340000; C_exp_8x8[22] = 32'h41400000; C_exp_8x8[23] = 32'h41100000;
      // Row 3
      C_exp_8x8[24] = 32'h41100000; C_exp_8x8[25] = 32'h41200000; C_exp_8x8[26] = 32'h41340000; C_exp_8x8[27] = 32'h41400000;
      C_exp_8x8[28] = 32'h41100000; C_exp_8x8[29] = 32'h41200000; C_exp_8x8[30] = 32'h41340000; C_exp_8x8[31] = 32'h41400000;
      // Row 4
      C_exp_8x8[32] = 32'h41400000; C_exp_8x8[33] = 32'h41100000; C_exp_8x8[34] = 32'h41200000; C_exp_8x8[35] = 32'h41340000;
      C_exp_8x8[36] = 32'h41400000; C_exp_8x8[37] = 32'h41100000; C_exp_8x8[38] = 32'h41200000; C_exp_8x8[39] = 32'h41340000;
      // Row 5
      C_exp_8x8[40] = 32'h41340000; C_exp_8x8[41] = 32'h41400000; C_exp_8x8[42] = 32'h41100000; C_exp_8x8[43] = 32'h41200000;
      C_exp_8x8[44] = 32'h41340000; C_exp_8x8[45] = 32'h41400000; C_exp_8x8[46] = 32'h41100000; C_exp_8x8[47] = 32'h41200000;
      // Row 6
      C_exp_8x8[48] = 32'h41200000; C_exp_8x8[49] = 32'h41340000; C_exp_8x8[50] = 32'h41400000; C_exp_8x8[51] = 32'h41100000;
      C_exp_8x8[52] = 32'h41200000; C_exp_8x8[53] = 32'h41340000; C_exp_8x8[54] = 32'h41400000; C_exp_8x8[55] = 32'h41100000;
      // Row 7
      C_exp_8x8[56] = 32'h41100000; C_exp_8x8[57] = 32'h41200000; C_exp_8x8[58] = 32'h41340000; C_exp_8x8[59] = 32'h41400000;
      C_exp_8x8[60] = 32'h41100000; C_exp_8x8[61] = 32'h41200000; C_exp_8x8[62] = 32'h41340000; C_exp_8x8[63] = 32'h41400000;
    end
  endtask

  // ============================================================
  // Deterministic tile init (same pattern as your wrap TB)
  // But now: write into SRAM using host ports
  // W SRAM addr: (i,k) via ww_row=i, ww_k=k
  // X SRAM addr: (j,k) via wx_row=j, wx_k=k  (note: tile_top loader stores X_tile[k][j])
  // ============================================================
  task automatic host_write_w(input int i, input int kk, input logic [31:0] data);
    begin
      @(negedge clk);
      ww_en    <= 1'b1;
      ww_we    <= 1'b1;
      ww_re    <= 1'b0;
      ww_row   <= i[W_ROW_W-1:0];
      ww_k     <= kk[K_W-1:0];
      ww_wdata <= data;
      ww_wmask <= 4'b1111;

      @(posedge clk); // write sampled

      @(negedge clk);
      ww_en    <= 1'b0;
      ww_we    <= 1'b0;
      ww_wdata <= '0;
      ww_wmask <= 4'b0000;
    end
  endtask

  task automatic host_write_x(input int j, input int kk, input logic [31:0] data);
    begin
      @(negedge clk);
      wx_en    <= 1'b1;
      wx_we    <= 1'b1;
      wx_re    <= 1'b0;
      wx_row   <= j[X_ROW_W-1:0];
      wx_k     <= kk[K_W-1:0];
      wx_wdata <= data;
      wx_wmask <= 4'b1111;

      @(posedge clk); // write sampled

      @(negedge clk);
      wx_en    <= 1'b0;
      wx_we    <= 1'b0;
      wx_wdata <= '0;
      wx_wmask <= 4'b0000;
    end
  endtask

  task automatic init_fixed_tiles_into_sram;
    int i, j, kk;
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

      // Write only KLEN region; KMAX==KLEN here (per your tile_top)
      for (kk = 0; kk < KLEN; kk++) begin
        for (i = 0; i < M; i++) begin
          host_write_w(i, kk, aval[(kk + i) % 4]);
        end
        for (j = 0; j < N; j++) begin
          // X SRAM is indexed by (row=j, k=kk)
          host_write_x(j, kk, bval[(kk + j) % 4]);
        end
      end

      $display("[%s] init_fixed_tiles_into_sram done (M=%0d N=%0d KLEN=%0d)", NAME, M, N, KLEN);
    end
  endtask

  // ============================================================
  // Wait helpers
  // ============================================================
  task automatic pulse_load_start;
    begin
      @(posedge clk);
      load_start <= 1'b1;
      @(posedge clk);
      load_start <= 1'b0;
    end
  endtask

  task automatic pulse_compute_start;
    begin
      @(posedge clk);
      compute_start <= 1'b1;
      @(posedge clk);
      compute_start <= 1'b0;
    end
  endtask

  task automatic wait_load_done;
    int unsigned tmo;
    begin
      tmo = 0;
      while (load_done !== 1'b1) begin
        @(posedge clk);
        tmo++;
        if (tmo > 1_000_000) $fatal(1, "[%s] TIMEOUT waiting load_done", NAME);
      end
    end
  endtask

  task automatic wait_compute_done;
    int unsigned tmo;
    begin
      tmo = 0;
      while (done !== 1'b1) begin
        @(posedge clk);
        tmo++;
        if (tmo > 50_000_000) $fatal(1, "[%s] TIMEOUT waiting done", NAME);
      end
    end
  endtask

  // ============================================================
  // Debug dump & compare
  // ============================================================
  task automatic dump_c_matrix;
    int i, j;
    begin
      $display("========== %s : C_tile ==========", NAME);
      for (i=0;i<M;i++) begin
        $write("Row %0d :", i);
        for (j=0;j<N;j++) $write(" %08h", C_tile[i][j]);
        $write("\n");
      end
      $display("================================");
    end
  endtask

  task automatic check_final_sliced_from_8x8;
    int i, j;
    logic [31:0] got, exp;
    begin
      $display("[%s] Final compare (slice from 8x8 base) ...", NAME);

      // safety
      @(posedge clk);
      @(posedge clk);

      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          got = C_tile[i][j];
          exp = C_exp_8x8[i*BASE_N + j];
          if (got !== exp) begin
            $display("[%s] FAIL C[%0d][%0d] got=%08h exp=%08h", NAME, i, j, got, exp);
            $fatal(1);
          end
        end
      end

      $display("[%s] PASS ✅", NAME);
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    // init default
    ww_en=0; ww_re=0; ww_we=0; ww_row='0; ww_k='0; ww_wdata='0; ww_wmask='0;
    wx_en=0; wx_re=0; wx_we=0; wx_row='0; wx_k='0; wx_wdata='0; wx_wmask='0;

    load_start    = 1'b0;
    compute_start = 1'b0;
    K_len         = KLEN[15:0];

    // reset
    rst = 1'b1;
    repeat (6) @(posedge clk);

    // init expected + preload SRAM (while in reset is OK since your SRAM model is sync; we’ll do after reset deassert to be safe)
    init_base_8x8_expected();

    rst = 1'b0;
    repeat (2) @(posedge clk);

    // host writes to SRAM
    init_fixed_tiles_into_sram();

    $display("=== %s start: M=%0d N=%0d KLEN=%0d ===", NAME, M, N, KLEN);

    // 1) load SRAM -> tile buffers
    pulse_load_start();
    wait_load_done();
    if (load_busy !== 1'b0) $display("[%s] WARN load_busy still high after load_done", NAME);

    // 2) compute
    pulse_compute_start();
    wait_compute_done();

    if (C_valid !== 1'b1)
      $display("[%s] WARN C_valid not high on done cycle", NAME);

    dump_c_matrix();
    check_final_sliced_from_8x8();

    $finish;
  end

endmodule


// ============================================================
// Top: 4 cases (8x8, 8x6, 6x6, 6x5) with KMAX=4 (as your tile_top default)
// ============================================================
module tb_tile_top_3cases_fixedexp;

  tb_tile_top_case_fixedexp #(.M(8), .N(8), .KMAX(4), .KLEN(4), .NAME("8x8")) u0();
  tb_tile_top_case_fixedexp #(.M(8), .N(6), .KMAX(4), .KLEN(4), .NAME("8x6")) u1();
  tb_tile_top_case_fixedexp #(.M(6), .N(6), .KMAX(4), .KLEN(4), .NAME("6x6")) u2();
  tb_tile_top_case_fixedexp #(.M(6), .N(5), .KMAX(4), .KLEN(4), .NAME("6x5")) u3();

endmodule

`default_nettype wire
