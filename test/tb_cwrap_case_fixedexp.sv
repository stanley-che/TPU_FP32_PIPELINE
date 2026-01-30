// tb_systolic_wrap_c_sram_fixedexp_3cases.sv
/*
iverilog -g2012 -Wall -I. -o ./vvp/tb_cwrap_3cases.vvp ./test/tb_cwrap_case_fixedexp.sv
vvp ./vvp/tb_cwrap_3cases.vvp
*/

`include "./src/systolic_wrap_c_sram.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_systolic_wrap_c_sram_fixedexp;

  localparam int unsigned M    = 8;
  localparam int unsigned N    = 8;
  localparam int unsigned K    = 4;
  localparam int unsigned KMAX = 1024;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  // ---------------- DUT ports ----------------
  logic        start;
  logic [15:0] K_len;
  logic        busy, done;

  logic [M*KMAX*DATA_W-1:0] W_tile_flat;
  logic [KMAX*N*DATA_W-1:0] X_tile_flat;

  logic              c_rd_en;
  logic              c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  logic [M*N*DATA_W-1:0] c_out_flat_o;
  logic [M*N-1:0]        c_valid_flat_o;
  logic                  C_valid;

  // ---------------- DUT ----------------
  systolic_wrap_c_sram #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(1),
    .ROW_W(ROW_W), .COL_W(COL_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done),

    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .c_out_flat_o(c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid(C_valid)
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

  // pack offsets
  function automatic int w_off(input int i, input int k);
    return (i*KMAX + k);
  endfunction
  function automatic int x_off(input int k, input int j);
    return (k*N + j);
  endfunction

  task automatic init_fixed_vectors_and_pack;
    int i, j, kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      W_tile_flat = '0;
      X_tile_flat = '0;

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

      // Fill A_mat/B_mat for kk=0..K-1
      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) A_mat[i][kk] = aval[(kk + i) % 4];
        for (j = 0; j < N; j++) B_mat[kk][j] = bval[(kk + j) % 4];
      end

      // Pack into W_tile_flat/X_tile_flat (only k=0..K-1 used)
      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++)
          W_tile_flat[w_off(i, kk)*32 +: 32] = A_mat[i][kk];
        for (j = 0; j < N; j++)
          X_tile_flat[x_off(kk, j)*32 +: 32] = B_mat[kk][j];
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
  // CPU read helper: read one (row,col) and return data
  // ============================================================
  task automatic cpu_read_word(input int r, input int c, output logic [31:0] data);
  int unsigned timeout;
  begin
    // --- ensure idle first (also avoids stale valid) ---
    @(negedge clk);
    c_rd_en <= 1'b0;
    c_rd_re <= 1'b0;
    c_rd_row <= '0;
    c_rd_col <= '0;

    // wait rvalid drop
    timeout = 0;
    while (c_rd_rvalid === 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 1_000_000)
        $fatal(1, "[TB] TIMEOUT waiting rvalid drop");
    end

    // --- drive request CLEANLY on negedge (no posedge race) ---
    @(negedge clk);
    c_rd_row <= r[ROW_W-1:0];
    c_rd_col <= c[COL_W-1:0];
    c_rd_en  <= 1'b1;
    c_rd_re  <= 1'b1;

    // --- wait for response ---
    timeout = 0;
    while (c_rd_rvalid !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 5_000_000)
        $fatal(1, "[TB] TIMEOUT cpu_read_word r=%0d c=%0d", r, c);
    end

    // sample after the posedge settles (avoids delta-cycle ordering issues)
    #1;
    data = c_rd_rdata;

    // --- deassert request on negedge ---
    @(negedge clk);
    c_rd_en <= 1'b0;
    c_rd_re <= 1'b0;
  end
endtask



  // ============================================================
  // Dump debug ports (SA output snapshot)
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
  // Scan SRAM and compare with expected
  // ============================================================
  task automatic scan_and_check_sram;
    int i, j;
    logic [31:0] got, exp;
    begin
      $display("[TB] Scan SRAM and compare vs expected C ...");
      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          cpu_read_word(i, j, got);
          exp = c_out_flat_o[C_idx(i,j)*32 +: 32];

          if (got !== exp) begin
            $display("[TB] FAIL SRAM C[%0d][%0d] got=%08h exp=%08h", i, j, got, exp);
            $fatal(1);
          end
        end
      end
      $display("[TB] PASS ✅ SRAM matched expected C");
    end
  endtask

  // ============================================================
  // Main
  // ============================================================
  initial begin
    int unsigned timeout;

    // init
    rst   = 1'b1;
    start = 1'b0;
    K_len = K[15:0];

    c_rd_en    = 1'b0;
    c_rd_re    = 1'b0;
    c_rd_row   = '0;
    c_rd_col   = '0;

    W_tile_flat = '0;
    X_tile_flat = '0;

    init_fixed_vectors_and_pack();

    // reset
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: systolic_wrap_c_sram fixed-exp (M=%0d N=%0d K=%0d) ===", M, N, K);

    // start pulse
    @(posedge clk);
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;

    // wait done (driver finished K steps)
    timeout = 0;
    while (done !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000)
        $fatal(1, "[TB] TIMEOUT waiting done");
    end
    $display("[TB] done seen. busy=%0b", busy);

    // wait C_valid (SRAM drained)
    timeout = 0;
    while (C_valid !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 50_000_000)
        $fatal(1, "[TB] TIMEOUT waiting C_valid");
    end
    $display("[TB] C_valid seen.");

    dump_debug_flat();
    scan_and_check_sram();

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
