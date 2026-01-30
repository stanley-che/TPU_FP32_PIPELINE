// TB (Method B): fixed 2D A/B + fixed expected C (NO calculation in TB)
// M=8, N=8, K=4
//
// iverilog -g2012 -Wall -I. -o ./vvp/tb_1pe.vvp ./test/tb_systolic_array_os.sv
// vvp ./vvp/tb_1pe.vvp

`include "./src/systolic_array_os.sv"
`timescale 1ns/1ps

`default_nettype none

module tb_systolic_array_os_flat_expect;

  localparam int M = 8;
  localparam int N = 8;
  localparam int K = 4;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  // ---------------- DUT I/O ----------------
  logic              step_valid;
  logic [M*32-1:0]   a_row_flat;
  logic [N*32-1:0]   b_col_flat;
  logic              k_first, k_last;

  logic              step_ready;
  logic [M*N*32-1:0] c_out_flat;
  logic [M*N-1:0]    c_valid_flat;
  logic [M*N*32-1:0] psum_out_flat;

  systolic_array_os_flat #(.M(M), .N(N)) dut (
    .clk(clk),
    .rst(rst),
    .step_valid(step_valid),
    .a_row_flat(a_row_flat),
    .b_col_flat(b_col_flat),
    .k_first(k_first),
    .k_last(k_last),
    .step_ready(step_ready),
    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat),
    .psum_out_flat(psum_out_flat)
  );

  // ============================================================
  // Fixed stimulus A/B (2D)
  // ============================================================
  logic [31:0] A_mat [0:M-1][0:K-1];
  logic [31:0] B_mat [0:K-1][0:N-1];

  // ============================================================
  // Fixed expected C (flat): C_exp_flat[i*N + j]
  // ============================================================
  logic [31:0] C_exp_flat [0:M*N-1];

  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  // ------------------------------------------------------------
  // Transaction: do one step (wait ready -> drive -> pulse valid -> wait done)
  // ------------------------------------------------------------
  task automatic do_step(input int kk);
    int unsigned timeout;
    int i, j;
    bit is_first, is_last;
    begin
      is_first = (kk == 0);
      is_last  = (kk == (K-1));

      // wait ready
      timeout = 0;
      while (step_ready !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 5_000_000)
          $fatal(1, "[TB] TIMEOUT waiting step_ready (kk=%0d)", kk);
      end

      // drive A/B into flatten bus
      for (i = 0; i < M; i++) a_row_flat[i*32 +: 32] = A_mat[i][kk];
      for (j = 0; j < N; j++) b_col_flat[j*32 +: 32] = B_mat[kk][j];

      k_first = is_first;
      k_last  = is_last;

      $display("[TB] >>> START step %0d (k_first=%0b k_last=%0b)", kk, is_first, is_last);

      // pulse step_valid
      @(posedge clk);
      step_valid = 1'b1;
      @(posedge clk);
      step_valid = 1'b0;
      k_first    = 1'b0;
      k_last     = 1'b0;

      // wait done (step_ready goes high again)
      timeout = 0;
      while (step_ready !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 50_000_000)
          $fatal(1, "[TB] TIMEOUT waiting step complete (kk=%0d)", kk);
      end

      $display("[TB] <<< DONE  step %0d", kk);
    end
  endtask

  // Final check: compare psum_out_flat vs fixed C_exp_flat
  task automatic check_final;
    int i, j;
    logic [31:0] got, exp;
    begin
      $display("[TB] Final compare psum_out_flat vs C_exp ...");

      @(posedge clk);
      @(posedge clk);

      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          got = psum_out_flat[C_idx(i,j)*32 +: 32];
          exp = C_exp_flat[C_idx(i,j)];
          if (got !== exp) begin
            $display("[TB] FAIL C[%0d][%0d] got=%08h exp=%08h", i, j, got, exp);
            $fatal(1);
          end
        end
      end

      $display("[TB] PASS ✅ all matched fixed expected C");
    end
  endtask

  task automatic dump_all_c;
    int i, j;
    begin
      $display("========== DUT psum_out_flat (C matrix) ==========");
      for (i = 0; i < M; i++) begin
        $write("Row %0d : ", i);
        for (j = 0; j < N; j++) begin
          $write("%08h ", psum_out_flat[C_idx(i,j)*32 +: 32]);
        end
        $write("\n");
      end
      $display("==================================================");
    end
  endtask

  // Init all fixed vectors and expected values (hardcoded expected C)
  task automatic init_fixed_vectors;
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

      // Fill A_mat
      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) begin
          A_mat[i][kk] = aval[(kk + i) % 4];
        end
      end

      // Fill B_mat
      for (kk = 0; kk < K; kk++) begin
        for (j = 0; j < N; j++) begin
          B_mat[kk][j] = bval[(kk + j) % 4];
        end
      end

      // ---- Fixed expected C (your hardcoded values) ----
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
  // Main
  // ============================================================
  initial begin
    int kk;

    // init
    rst        = 1'b1;
    step_valid = 1'b0;
    k_first    = 1'b0;
    k_last     = 1'b0;

    a_row_flat = '0;
    b_col_flat = '0;

    init_fixed_vectors();

    // reset
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: systolic_array_os_flat fixed-expect M=%0d N=%0d K=%0d ===", M, N, K);

    // run K steps
    for (kk = 0; kk < K; kk++) begin
      do_step(kk);
      $display("[TB] after step %0d: C00=%08h", kk, psum_out_flat[0*32 +: 32]);
    end

    dump_all_c();
    check_final();

    $finish;
  end

endmodule

`default_nettype wire
