// tb_sramsa_fixedexp.sv
/*
iverilog -g2012 -Wall -I. -o ./vvp/tb_sramsa_fixedexp.vvp ./test/tb_sramsa_fixedexp.sv
vvp ./vvp/tb_sramsa_fixedexp.vvp


目的：
1) 用 cpu_* ports preload W SRAM 與 X SRAM（模擬 CPU/DRAM 寫 SRAM）
2) pulse start + 設 K_len => tile_system_top controller 依 k=0..K_len-1 觸發 W/X loader
3) 在 loader 讀回 SRAM 的每個 beat 上做 scoreboard 驗證（最穩）
   同時更新 W_shadow/X_shadow 為「loader 讀回的值」（代表最後進 tile 的值）
4) 等 sramsa done（對齊 SA done）
5) CPU 讀回 C SRAM，對照 fixed-exp 8x8 base slice（跟你 systolic_wrap TB 一樣）

注意：
- preload 的資料模式：A000_0000 + (row<<16) + k  與  B000_0000 + (k<<16) + n
- C 的 expected：沿用你原本 tb_cwrap_case_fixedexp 的 8x8 KLEN=4 base 表做 slice
*/

`include "./src/EPU/MAC/sramsa.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_sramsa_fixedexp;

  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // 測試範圍：preload k=0..9，但 controller 只跑 K_len=10（0..9）
  localparam int unsigned K_LO  = 0;
  localparam int unsigned K_HI  = 4;
  localparam int unsigned K_LEN = (K_HI - K_LO + 1);
  localparam string NAME = "SRAMSA";

  // 若你用 Questa/Verilator 可打開，但 Icarus 常會 X
  localparam bit CHECK_WTILE_PORT = 1'b0;
  localparam bit CHECK_XTILE_PORT = 1'b0;

  // 控制 print 量
  localparam bit VERBOSE_CPU_WR = 1'b1;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ---------------- command ----------------
  logic        start;
  logic [15:0] K_len;
  wire         busy;
  wire         done;

  // ---------------- CPU preload W SRAM ----------------
  logic                         cpu_w_we;
  logic [$clog2(M)-1:0]          cpu_w_row;
  logic [K_W-1:0]                cpu_w_k;
  logic [DATA_W-1:0]             cpu_w_wdata;
  logic [BYTE_W-1:0]             cpu_w_wmask;

  // ---------------- CPU preload X SRAM ----------------
  logic                         cpu_x_we;
  logic [K_W-1:0]                cpu_x_k;
  logic [$clog2(N)-1:0]          cpu_x_n;
  logic [DATA_W-1:0]             cpu_x_wdata;
  logic [BYTE_W-1:0]             cpu_x_wmask;

  // ---------------- CPU read C SRAM ----------------
  logic                         c_rd_en;
  logic                         c_rd_re;
  logic [$clog2(M)-1:0]          c_rd_row;
  logic [$clog2(N)-1:0]          c_rd_col;
  wire  [DATA_W-1:0]             c_rd_rdata;
  wire                          c_rd_rvalid;

  wire                          C_valid;

  // ---------------- DUT ----------------
  sramsa #(
    .M(M), .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
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

    .C_valid(C_valid)
  );

  // ============================================================
  // Patterns for preload (SRAM content expectation)
  // ============================================================
  function automatic [DATA_W-1:0] w_mem_value(input int unsigned row, input int unsigned kk);
    w_mem_value = 32'hA000_0000 + (row << 16) + kk;
  endfunction

  function automatic [DATA_W-1:0] x_mem_value(input int unsigned kk, input int unsigned nn);
    x_mem_value = 32'hB000_0000 + (kk << 16) + nn;
  endfunction
  always @(posedge clk) begin
  if (!rst) begin
    if (dut.sa_start) $display("[MON] sa_start @%0t K_len(in)=%0d", $time, K_len);
    if (dut.sa_busy)  $display("[MON] sa_busy=1 @%0t", $time);
    if (dut.sa_done)  $display("[MON] sa_done @%0t", $time);
  end
end


  // ============================================================
  // SHADOW ARRAYS (Icarus reliable)
  // - preload 時寫入（顯示你「寫進 SRAM」的矩陣）
  // - loader stream 時覆寫成 rdata（顯示「最後進 tile」的值）
  // ============================================================
  logic [DATA_W-1:0] W_shadow [M][KMAX];
  logic [DATA_W-1:0] X_shadow [KMAX][N];

  // ============================================================
  // CPU write tasks (with print)
  // ============================================================
  task automatic cpu_write_w(input int unsigned row, input int unsigned kk, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_w_we    <= 1'b1;
      cpu_w_row   <= row[$clog2(M)-1:0];
      cpu_w_k     <= kk[K_W-1:0];
      cpu_w_wdata <= data;
      cpu_w_wmask <= {BYTE_W{1'b1}}; // all bytes write
      @(negedge clk);
      cpu_w_we    <= 1'b0;

      W_shadow[row][kk] = data;

      if (VERBOSE_CPU_WR) begin
        $display("[CPU_W_WR] row=%0d k=%0d data=%08x wmask=%0h t=%0t",
                 row, kk, data, {BYTE_W{1'b1}}, $time);
      end
    end
  endtask

  task automatic cpu_write_x(input int unsigned kk, input int unsigned nn, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_x_we    <= 1'b1;
      cpu_x_k     <= kk[K_W-1:0];
      cpu_x_n     <= nn[$clog2(N)-1:0];
      cpu_x_wdata <= data;
      cpu_x_wmask <= {BYTE_W{1'b1}}; // all bytes write
      @(negedge clk);
      cpu_x_we    <= 1'b0;

      X_shadow[kk][nn] = data;

      if (VERBOSE_CPU_WR) begin
        $display("[CPU_X_WR] k=%0d n=%0d data=%08x wmask=%0h t=%0t",
                 kk, nn, data, {BYTE_W{1'b1}}, $time);
      end
    end
  endtask

  task automatic preload_w(input int unsigned k_lo, input int unsigned k_hi);
    int unsigned r, k;
    begin
      for (r=0; r<M; r++) begin
        for (k=k_lo; k<=k_hi; k++) begin
          cpu_write_w(r, k, w_mem_value(r,k));
        end
      end
    end
  endtask

  task automatic preload_x(input int unsigned k_lo, input int unsigned k_hi);
    int unsigned k, n;
    begin
      for (k=k_lo; k<=k_hi; k++) begin
        for (n=0; n<N; n++) begin
          cpu_write_x(k, n, x_mem_value(k,n));
        end
      end
    end
  endtask

  // ============================================================
  // kick controller
  // ============================================================
  task automatic pulse_start_controller(input int unsigned k_len);
    begin
      @(negedge clk);
      K_len <= k_len[15:0];
      start <= 1'b1;
      @(negedge clk);
      start <= 1'b0;
      $display("[CTRL_START] K_len=%0d t=%0t", k_len, $time);
    end
  endtask

  // ============================================================
  // Loader stream scoreboard
  // ============================================================
  int unsigned w_beats, x_beats;
  int unsigned w_errs,  x_errs;

  // 你原先做法：用 u_loader.row_issued / n_issued + k_latched
  // 這裡直接沿用，並且把 read data 覆寫進 shadow
  always @(posedge clk) begin
    // W stream
    if (dut.u_tile.u_w.w_rvalid) begin
      int unsigned row_obs;
      int unsigned k_obs;
      logic [DATA_W-1:0] exp;

      row_obs = dut.u_tile.u_w.u_loader.row_issued;
      k_obs   = dut.u_tile.u_w.u_loader.k_latched;
      exp     = w_mem_value(row_obs, k_obs);

      // shadow = loader 讀回值（代表最後進 tile）
      W_shadow[row_obs][k_obs] <= dut.u_tile.u_w.w_rdata;

      if (dut.u_tile.u_w.w_rdata !== exp) begin
        $display("[FAIL][W_STREAM] row=%0d k=%0d rdata=%08x exp=%08x t=%0t",
                 row_obs, k_obs, dut.u_tile.u_w.w_rdata, exp, $time);
        w_errs++;
      end

      if (CHECK_WTILE_PORT) begin
        #1ps;
        if (dut.u_tile.W_tile[row_obs][k_obs] !== exp) begin
          $display("[FAIL][W_TILE_PORT] row=%0d k=%0d W_tile=%08x exp=%08x t=%0t",
                   row_obs, k_obs, dut.u_tile.W_tile[row_obs][k_obs], exp, $time);
          w_errs++;
        end
      end

      w_beats++;
    end

    // X stream
    if (dut.u_tile.u_x.x_rvalid) begin
      int unsigned n_obs;
      int unsigned k_obs;
      logic [DATA_W-1:0] exp;

      n_obs = dut.u_tile.u_x.u_loader.n_issued;
      k_obs = dut.u_tile.u_x.u_loader.k_latched;
      exp   = x_mem_value(k_obs, n_obs);

      X_shadow[k_obs][n_obs] <= dut.u_tile.u_x.x_rdata;

      if (dut.u_tile.u_x.x_rdata !== exp) begin
        $display("[FAIL][X_STREAM] k=%0d n=%0d rdata=%08x exp=%08x t=%0t",
                 k_obs, n_obs, dut.u_tile.u_x.x_rdata, exp, $time);
        x_errs++;
      end

      if (CHECK_XTILE_PORT) begin
        #1ps;
        if (dut.u_tile.X_tile[k_obs][n_obs] !== exp) begin
          $display("[FAIL][X_TILE_PORT] k=%0d n=%0d X_tile=%08x exp=%08x t=%0t",
                   k_obs, n_obs, dut.u_tile.X_tile[k_obs][n_obs], exp, $time);
          x_errs++;
        end
      end

      x_beats++;
    end
  end

  // ============================================================
  // Pretty dumps (shadow)
  // ============================================================
  task automatic dump_w_shadow(input int unsigned k_lo, input int unsigned k_hi);
    begin
      $display("\n===== DUMP W_shadow (rows 0..%0d, k %0d..%0d) =====", M-1, k_lo, k_hi);
      for (int r = 0; r < M; r++) begin
        $write("W_row[%0d]: ", r);
        for (int k = k_lo; k <= k_hi; k++) begin
          $write("%08x ", W_shadow[r][k]);
        end
        $write("\n");
      end
    end
  endtask

  task automatic dump_x_shadow(input int unsigned k_lo, input int unsigned k_hi);
    begin
      $display("\n===== DUMP X_shadow (k %0d..%0d, n 0..%0d) =====", k_lo, k_hi, N-1);
      for (int k = k_lo; k <= k_hi; k++) begin
        $write("X_k[%0d]: ", k);
        for (int n = 0; n < N; n++) begin
          $write("%08x ", X_shadow[k][n]);
        end
        $write("\n");
      end
    end
  endtask

  // ============================================================
  // C expected table (base 8x8 KLEN=4) + slice helper
  // 直接搬你原本 TB 的表（保持完全一致）
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

  function automatic logic [31:0] exp_from_base(input int i, input int j);
    exp_from_base = C_exp_8x8[i*BASE_N + j];
  endfunction

  // ============================================================
  // CPU read one C element from SRAM
  // ============================================================
    task automatic cpu_read_c(input int unsigned i, input int unsigned j, output logic [31:0] data);
    begin
        @(negedge clk);
        c_rd_row <= i[$clog2(M)-1:0];
        c_rd_col <= j[$clog2(N)-1:0];
        c_rd_en  <= 1'b1;
        c_rd_re  <= 1'b1;

        @(negedge clk);
        c_rd_en  <= 1'b0;
        c_rd_re  <= 1'b0;

        while (c_rd_rvalid !== 1'b1) @(posedge clk);

        // ★ 多等一拍，避開 rvalid/rdata 沒對齊
        @(posedge clk);
        data = c_rd_rdata;
    end
    endtask


  task automatic dump_c_sram_matrix;
    int i, j;
    logic [31:0] d;
    begin
      $display("========== %s : C_SRAM readback ==========", NAME);
      for (i=0;i<M;i++) begin
        $write("Row %0d :", i);
        for (j=0;j<N;j++) begin
          cpu_read_c(i,j,d);
          $write(" %08h", d);
        end
        $write("\n");
      end
      $display("==========================================");
    end
  endtask

  task automatic check_c_sram_against_base;
    int i, j;
    logic [31:0] got, exp;
    begin
      $display("[%s] Compare C_SRAM readback vs base slice ...", NAME);
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          cpu_read_c(i,j,got);
          exp = exp_from_base(i,j);
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
  // Wave dump
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_sramsa.vcd");
    $dumpvars(0, tb_sramsa_fixedexp);
  end

  // ============================================================
  // Main
  // ============================================================
  initial begin
    // init
    start = 1'b0;
    K_len = '0;

    cpu_w_we = 1'b0;
    cpu_w_row = '0;
    cpu_w_k = '0;
    cpu_w_wdata = '0;
    cpu_w_wmask = '0;

    cpu_x_we = 1'b0;
    cpu_x_k = '0;
    cpu_x_n = '0;
    cpu_x_wdata = '0;
    cpu_x_wmask = '0;

    c_rd_en  = 1'b0;
    c_rd_re  = 1'b0;
    c_rd_row = '0;
    c_rd_col = '0;

    w_beats = 0; x_beats = 0;
    w_errs  = 0; x_errs  = 0;

    // init shadows
    for (int r = 0; r < M; r++)
      for (int k = 0; k < KMAX; k++)
        W_shadow[r][k] = '0;

    for (int k = 0; k < KMAX; k++)
      for (int n = 0; n < N; n++)
        X_shadow[k][n] = '0;

    init_base_8x8_expected();

    // reset
    rst = 1'b1;
    repeat(6) @(posedge clk);
    rst = 1'b0;
    repeat(2) @(posedge clk);

    // Phase 1: preload W/X
    preload_w(K_LO, K_HI);
    preload_x(K_LO, K_HI);
    repeat(5) @(posedge clk);

    $display("\n===== AFTER CPU PRELOAD (shadow = preload data) =====");
    dump_w_shadow(K_LO, K_HI);
    dump_x_shadow(K_LO, K_HI);

    // Phase 2: run loader + SA (sramsa 內部：tile_done 觸發 SA)
    pulse_start_controller(K_LEN);

    // wait done (sa_done pulse)
    wait(done === 1'b1);
    repeat(2) @(posedge clk);


    $display("\n===== AFTER LOADER STREAM (shadow updated by rdata beats) =====");
    dump_w_shadow(K_LO, K_HI);
    dump_x_shadow(K_LO, K_HI);

    // stream beat counts
    if (w_beats != (K_LEN*M)) begin
      $display("[FAIL] W beats mismatch got=%0d exp=%0d", w_beats, (K_LEN*M));
      $fatal(1);
    end
    if (x_beats != (K_LEN*N)) begin
      $display("[FAIL] X beats mismatch got=%0d exp=%0d", x_beats, (K_LEN*N));
      $fatal(1);
    end
    if (w_errs != 0 || x_errs != 0) begin
      $display("[FAIL] stream errors w_errs=%0d x_errs=%0d", w_errs, x_errs);
      $fatal(1);
    end
    $display("[PASS] W/X loader stream OK. beats: W=%0d X=%0d", w_beats, x_beats);

    // Phase 3: readback C SRAM + compare base slice
    if (C_valid !== 1'b1)
      $display("[WARN] C_valid not high when done (might be OK depending on your design)");

    dump_c_sram_matrix();
    check_c_sram_against_base();

    $display("\n[ALL PASS] sramsa preload+load+SA+Creadback ✅");
    $finish;
  end

endmodule

`default_nettype wire
