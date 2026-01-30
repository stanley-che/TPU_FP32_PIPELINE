// tb_sramsa_fixedexp_new.sv
/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_sramsa_fixedexp_new.vvp ./test/tb_sramsa_fixedexp_new.sv
vvp ./vvp/tb_sramsa_fixedexp_new.vvp
gtkwave ./vvp/tb_sramsa_fixedexp_new.vcd

E2E 目的：
Phase A: CPU preload W/X SRAM (fixed-exp pattern, for C expected)
Phase B: pulse start + K_len => tile loaders stream tiles, per-beat scoreboard check
Phase C: wait sa done => CPU readback C SRAM => compare vs base expected (8x8, KLEN=4)
*/
`include "./src/EPU/attention_score/sramsa.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_tile_compute_system_e2e_fixedexp;

  localparam int unsigned M    = 64;
  localparam int unsigned N    = 64;
  localparam int unsigned K    = 64;
  localparam int unsigned KMAX = 64;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ---------------- command ----------------
  logic        start;
  logic [15:0] K_len;
  logic        busy, done;

  // ---------------- cpu writes ----------------
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

  // ---------------- C SRAM CPU read ----------------
  logic              c_rd_en;
  logic              c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  // ---------------- debug ----------------
  logic [M*N*DATA_W-1:0] c_out_flat_o;
  logic [M*N-1:0]        c_valid_flat_o;
  logic                  C_valid;

  logic [M*KMAX*DATA_W-1:0] W_tile_flat_dbg;
  logic [KMAX*N*DATA_W-1:0] X_tile_flat_dbg;

  // ---------------- DUT ----------------
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
  // Fixed pattern + expected C (same as你之前)
  // ============================================================
  logic [31:0] A_mat [0:M-1][0:K-1];
  logic [31:0] B_mat [0:K-1][0:N-1];

  logic [31:0] C_exp_flat [0:M*N-1];

  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  task automatic build_fixed_A_B_and_expected;
    int i, j, kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      // A pattern: (kk+i)%4 => {1.0,2.0,0.5,3.0}
      aval[0] = 32'h3f800000; // 1.0
      aval[1] = 32'h40000000; // 2.0
      aval[2] = 32'h3f000000; // 0.5
      aval[3] = 32'h40400000; // 3.0

      // B pattern: (kk+j)%4 => {1.0,0.5,2.0,3.0}
      bval[0] = 32'h3f800000; // 1.0
      bval[1] = 32'h3f000000; // 0.5
      bval[2] = 32'h40000000; // 2.0
      bval[3] = 32'h40400000; // 3.0

      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) A_mat[i][kk] = aval[(kk + i) % 4];
        for (j = 0; j < N; j++) B_mat[kk][j] = bval[(kk + j) % 4];
      end

      // ---- Fixed expected C (hardcoded) ----
      // Row 0
      C_exp_flat[0]  = 32'h41400000; C_exp_flat[1]  = 32'h41100000; C_exp_flat[2]  = 32'h41200000; C_exp_flat[3]  = 32'h41340000;
      C_exp_flat[4]  = 32'h41400000; C_exp_flat[5]  = 32'h41100000; C_exp_flat[6]  = 32'h41200000; C_exp_flat[7]  = 32'h41340000;
      // Row 1
      C_exp_flat[8]  = 32'h41340000; C_exp_flat[9]  = 32'h41400000; C_exp_flat[10] = 32'h41100000; C_exp_flat[11] = 32'h41200000;
      C_exp_flat[12] = 32'h41340000; C_exp_flat[13] = 32'h41400000; C_exp_flat[14] = 32'h41100000; C_exp_flat[15] = 32'h41200000;
      // Row 2
      C_exp_flat[16] = 32'h41200000; C_exp_flat[17] = 32'h41340000; C_exp_flat[18] = 32'h41400000; C_exp_flat[19] = 32'h41100000;
      C_exp_flat[20] = 32'h41200000; C_exp_flat[21] = 32'h41340000; C_exp_flat[22] = 32'h41400000; C_exp_flat[23] = 32'h41100000;
      // Row 3
      C_exp_flat[24] = 32'h41100000; C_exp_flat[25] = 32'h41200000; C_exp_flat[26] = 32'h41340000; C_exp_flat[27] = 32'h41400000;
      C_exp_flat[28] = 32'h41100000; C_exp_flat[29] = 32'h41200000; C_exp_flat[30] = 32'h41340000; C_exp_flat[31] = 32'h41400000;
      // Row 4
      C_exp_flat[32] = 32'h41400000; C_exp_flat[33] = 32'h41100000; C_exp_flat[34] = 32'h41200000; C_exp_flat[35] = 32'h41340000;
      C_exp_flat[36] = 32'h41400000; C_exp_flat[37] = 32'h41100000; C_exp_flat[38] = 32'h41200000; C_exp_flat[39] = 32'h41340000;
      // Row 5
      C_exp_flat[40] = 32'h41340000; C_exp_flat[41] = 32'h41400000; C_exp_flat[42] = 32'h41100000; C_exp_flat[43] = 32'h41200000;
      C_exp_flat[44] = 32'h41340000; C_exp_flat[45] = 32'h41400000; C_exp_flat[46] = 32'h41100000; C_exp_flat[47] = 32'h41200000;
      // Row 6
      C_exp_flat[48] = 32'h41200000; C_exp_flat[49] = 32'h41340000; C_exp_flat[50] = 32'h41400000; C_exp_flat[51] = 32'h41100000;
      C_exp_flat[52] = 32'h41200000; C_exp_flat[53] = 32'h41340000; C_exp_flat[54] = 32'h41400000; C_exp_flat[55] = 32'h41100000;
      // Row 7
      C_exp_flat[56] = 32'h41100000; C_exp_flat[57] = 32'h41200000; C_exp_flat[58] = 32'h41340000; C_exp_flat[59] = 32'h41400000;
      C_exp_flat[60] = 32'h41100000; C_exp_flat[61] = 32'h41200000; C_exp_flat[62] = 32'h41340000; C_exp_flat[63] = 32'h41400000;
    end
  endtask

  // ============================================================
  // CPU write helpers (preload into tile_system_top SRAM models)
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

  task automatic preload_fixed_A_B();
    int i, j, kk;
    begin
      // preload only k=0..K-1 (others don't care)
      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) cpu_write_w(i, kk, A_mat[i][kk]);
        for (j = 0; j < N; j++) cpu_write_x(kk, j, B_mat[kk][j]);
      end
    end
  endtask

  task automatic sanity_check_preload();
    begin
      // wait a couple cycles for last write to settle
      repeat (2) @(posedge clk);

      if (dut.u_tile.w_mem[0][0] !== A_mat[0][0]) begin
        $display("[PRELOAD_FAIL][W] w_mem[0][0]=%08h exp=%08h", dut.u_tile.w_mem[0][0], A_mat[0][0]);
        $fatal(1);
      end
      if (dut.u_tile.x_mem[0][0] !== B_mat[0][0]) begin
        $display("[PRELOAD_FAIL][X] x_mem[0][0]=%08h exp=%08h", dut.u_tile.x_mem[0][0], B_mat[0][0]);
        $fatal(1);
      end

      $display("[PRELOAD_OK] sample points match.");
    end
  endtask

  // ============================================================
  // CPU read helper: read one (row,col) from C SRAM
  // ============================================================
    // ============================================================
  // CPU read helper: read one (row,col) from C SRAM
  // - keep c_rd_en always 1
  // - hold c_rd_re high until c_rd_rvalid
  // - keep addr stable while waiting
  // - add 1-cycle gap to avoid back-to-back hazard
  // ============================================================
  task automatic cpu_read_c_word(input int r, input int c, output logic [31:0] data);
  int unsigned timeout;
  begin
    data = '0;

    // request: set addr + pulse re for 1 cycle
    @(negedge clk);
    c_rd_row <= r[ROW_W-1:0];
    c_rd_col <= c[COL_W-1:0];
    c_rd_re  <= 1'b1;

    @(negedge clk);
    c_rd_re  <= 1'b0;

    // wait response (addr can stay stable; but keep it stable anyway)
    timeout = 0;
    while (c_rd_rvalid !== 1'b1) begin
      @(posedge clk);
      c_rd_row <= r[ROW_W-1:0];
      c_rd_col <= c[COL_W-1:0];
      timeout++;
      if (timeout > 5_000_000)
        $fatal(1, "[TB] TIMEOUT cpu_read_c_word r=%0d c=%0d", r, c);
    end

    #1;
    data = c_rd_rdata;

    // 1-cycle gap (avoid back-to-back hazard)
    @(posedge clk);
  end
endtask


  // ============================================================
  // Dump debug flat (SA output snapshot)
  // ============================================================
  task automatic dump_debug_flat;
    int i, j;
    begin
      $display("========== DEBUG c_out_flat_o (from SA) ==========");
      for (i = 0; i < M; i++) begin
        $write("Row %0d : ", i);
        for (j = 0; j < N; j++) begin
          $write("%08h ", c_out_flat_o[C_idx(i,j)*32 +: 32]);
        end
        $write("\n");
      end
      $display("==================================================");
    end
  endtask

  // ============================================================
  // Scan C SRAM and compare vs (debug flat) and (hardcoded expected)
  // ============================================================
  task automatic scan_and_check_c_sram;
    int i, j;
    logic [31:0] got, exp_dbg, exp_fix;
    begin
      $display("[TB] Scan C SRAM and compare...");
      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          cpu_read_c_word(i, j, got);

          exp_dbg = c_out_flat_o[C_idx(i,j)*32 +: 32];
          exp_fix = C_exp_flat[C_idx(i,j)];

          if (got !== exp_dbg) begin
            $display("[TB] FAIL C_SRAM!=DBG C[%0d][%0d] got=%08h dbg=%08h", i, j, got, exp_dbg);
            $fatal(1);
          end
          if (got !== exp_fix) begin
            $display("[TB] FAIL C_SRAM!=FIX C[%0d][%0d] got=%08h fix=%08h", i, j, got, exp_fix);
            $fatal(1);
          end
        end
      end
      $display("[TB] PASS ✅ C SRAM matched (debug flat) and (hardcoded expected)");
    end
  endtask
  //MEASUREMENT
  // ---- FP32 bits -> shortreal (decimal printing) ----
// ---- IEEE754 FP32 bits -> real (iverilog-friendly) ----
// Supports normal numbers + subnormals + zero. (NaN/Inf prints huge/NaN-ish)
function automatic real f32_to_real(input logic [31:0] b);
  int sign;
  int exp;
  int frac;
  real mant;
  real val;
  begin
    sign = b[31];
    exp  = b[30:23];
    frac = b[22:0];

    // Zero
    if (exp == 0 && frac == 0) begin
      val = 0.0;
    end
    // Subnormal: exp=0, leading 0.xxx, exponent = -126
    else if (exp == 0) begin
      mant = frac / (2.0**23);          // 0.xxx
      val  = mant * (2.0**(-126));
    end
    // Inf/NaN (optional handling)
    else if (exp == 255) begin
      // You can customize printing here; keep it simple:
      val = 1.0/0.0; // Inf (iverilog may show inf)
    end
    // Normal: leading 1.xxx, exponent = exp-127
    else begin
      mant = 1.0 + (frac / (2.0**23));
      val  = mant * (2.0**(exp-127));
    end

    if (sign) val = -val;
    return val;
  end
endfunction


task automatic dump_A_B_C_decimal;
  int i, j, k;
  real v;

  begin
    $display("\n================ A (M x K) decimal ================");
    for (i = 0; i < M; i++) begin
      $write("A[%0d,:] : ", i);
      for (k = 0; k < K; k++) begin
        v = f32_to_real(A_mat[i][k]);
        $write("%0.6f ", v);
      end
      $write("\n");
    end

    $display("\n================ B (K x N) decimal ================");
    for (k = 0; k < K; k++) begin
      $write("B[%0d,:] : ", k);
      for (j = 0; j < N; j++) begin
        v = f32_to_real(B_mat[k][j]);
        $write("%0.6f ", v);
      end
      $write("\n");
    end

    $display("\n================ C (M x N) decimal (from SA debug c_out_flat_o) ================");
    for (i = 0; i < M; i++) begin
      $write("C[%0d,:] : ", i);
      for (j = 0; j < N; j++) begin
        v = f32_to_real(c_out_flat_o[C_idx(i,j)*32 +: 32]);
        $write("%0.6f ", v);
      end
      $write("\n");
    end
    $display("===============================================================================\n");
  end
endtask
  task automatic dump_C_sram_decimal;
  int i, j;
  logic [31:0] got;
  real v;
  begin
    $display("\n================ C (M x N) decimal (from C SRAM readback) ================");
    for (i = 0; i < M; i++) begin
      $write("C_sram[%0d,:] : ", i);
      for (j = 0; j < N; j++) begin
        cpu_read_c_word(i, j, got);
        v = f32_to_real(got);
        $write("%0.6f ", v);
      end
      $write("\n");
    end
    $display("============================================================================\n");
  end
endtask

  // ============================================================
  // Wave dump
  // ============================================================
  `ifdef DUMP_WAVE
initial begin
  $dumpfile("./vvp/tb_e2e_fixedexp.vcd");
  $dumpvars(1, tb_tile_compute_system_e2e_fixedexp);
end
`endif

  int unsigned timeout;
  // ============================================================
  // Main
  // ============================================================
  initial begin
    // init
    rst   = 1'b1;
    start = 1'b0;
    K_len = K[15:0];

    cpu_w_we='0; cpu_w_row='0; cpu_w_k='0; cpu_w_wdata='0; cpu_w_wmask='0;
    cpu_x_we='0; cpu_x_k='0; cpu_x_n='0; cpu_x_wdata='0; cpu_x_wmask='0;

    c_rd_en='1; c_rd_re='0; c_rd_row='0; c_rd_col='0;

    build_fixed_A_B_and_expected();

    // reset
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: E2E tile_system_top + systolic_wrap_c_sram (M=%0d N=%0d K=%0d) ===", M, N, K);

    // preload W/X SRAM
    preload_fixed_A_B();
    sanity_check_preload();

    // start pulse
    @(posedge clk);
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;

    // wait done (loader/controller finished)
    timeout = 0;
    while (done !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000) $fatal(1, "[TB] TIMEOUT waiting done");
    end
    $display("[TB] done seen. busy=%0b", busy);

    // wait C_valid (C SRAM drained)
    timeout = 0;
    while (C_valid !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000) $fatal(1, "[TB] TIMEOUT waiting C_valid");
    end
    $display("[TB] C_valid seen.");
    dump_A_B_C_decimal();
    dump_debug_flat();
    scan_and_check_c_sram();
    
    dump_C_sram_decimal();


    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
