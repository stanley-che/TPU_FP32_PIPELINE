// tb_sa_tile_driver_fixedexp.sv
// iverilog -g2012 -Wall -I. -o ./vvp/tb_sa_tile_driver.vvp ./test/tb_sa_tile_driver.sv
// vvp ./vvp/tb_sa_tile_driver.vvp

`include "./src/sa_tile_driver.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_sa_tile_driver_flat_expect;

  localparam int M    = 8;
  localparam int N    = 8;
  localparam int K    = 4;
  localparam int KMAX = 1024;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  // ---------------- command ----------------
  logic        tile_start;
  logic [15:0] K_len;
  logic        tile_busy;
  logic        tile_done;

  // ---------------- tile buffers (flatten) ----------------
  logic [M*KMAX*32-1:0] W_tile_flat;
  logic [KMAX*N*32-1:0] X_tile_flat;

  // ---------------- driver <-> SA wires ----------------
  logic              step_valid;
  logic [M*32-1:0]   a_row_flat;
  logic [N*32-1:0]   b_col_flat;
  logic              k_first;
  logic              k_last;
  logic              step_ready;

  logic [M*N*32-1:0] c_out_flat;
  logic [M*N-1:0]    c_valid_flat;
  logic [M*N*32-1:0] psum_out_flat;

  // ---------------- DUTs ----------------
  sa_tile_driver_flat #(.M(M), .N(N), .KMAX(KMAX)) u_drv (
    .clk(clk),
    .rst(rst),
    .tile_start(tile_start),
    .K_len(K_len),
    .tile_busy(tile_busy),
    .tile_done(tile_done),
    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat),

    .step_valid(step_valid),
    .a_row_flat(a_row_flat),
    .b_col_flat(b_col_flat),
    .k_first(k_first),
    .k_last(k_last),
    .step_ready(step_ready),

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat)
  );

  systolic_array_os_flat #(.M(M), .N(N)) u_sa (
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
  // Fixed pattern (same as你之前 Method B)
  // A_mat[i][kk] = aval[(kk+i)%4]
  // B_mat[kk][j] = bval[(kk+j)%4]
  // ============================================================
  logic [31:0] A_mat [0:M-1][0:K-1];
  logic [31:0] B_mat [0:K-1][0:N-1];

  // expected C (flat) idx=i*N+j
  logic [31:0] C_exp_flat [0:M*N-1];
  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  // pack offsets into flatten W/X
  function automatic int w_off(input int i, input int k);
    return (i*KMAX + k);
  endfunction
  function automatic int x_off(input int k, input int j);
    return (k*N + j);
  endfunction

  // ============================================================
  // init vectors + pack into W_tile_flat/X_tile_flat
  // ============================================================
  task automatic init_fixed_vectors_and_pack;
    int i, j, kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      // clear flats
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

      // Pack into W_tile_flat/X_tile_flat (only fill k=0..K-1)
      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) begin
          W_tile_flat[w_off(i, kk)*32 +: 32] = A_mat[i][kk];
        end
        for (j = 0; j < N; j++) begin
          X_tile_flat[x_off(kk, j)*32 +: 32] = B_mat[kk][j];
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

  // dump psum_out_flat
  task automatic dump_all_c;
    int i, j;
    begin
      $display("========== SA psum_out_flat (C matrix) ==========");
      for (i = 0; i < M; i++) begin
        $write("Row %0d : ", i);
        for (j = 0; j < N; j++) begin
          $write("%08h ", psum_out_flat[C_idx(i,j)*32 +: 32]);
        end
        $write("\n");
      end
      $display("=================================================");
    end
  endtask

  // compare expected
  task automatic check_final;
    int i, j;
    logic [31:0] got, exp;
    begin
      $display("[TB] Final compare SA psum_out_flat vs C_exp ...");

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

  // ============================================================
  // Main
  // ============================================================
  initial begin
    int unsigned timeout;

    // init
    rst        = 1'b1;
    tile_start = 1'b0;
    K_len      = K[15:0];

    W_tile_flat = '0;
    X_tile_flat = '0;

    init_fixed_vectors_and_pack();

    // reset
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: sa_tile_driver_flat + systolic_array_os_flat (M=%0d N=%0d K=%0d) ===", M, N, K);

    // start one tile
    @(posedge clk);
    tile_start = 1'b1;
    @(posedge clk);
    tile_start = 1'b0;

    // wait tile_done
    timeout = 0;
    while (tile_done !== 1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout > 100_000_000)
        $fatal(1, "[TB] TIMEOUT waiting tile_done");
    end

    $display("[TB] tile_done seen. tile_busy=%0b", tile_busy);

    dump_all_c();
    check_final();

    $finish;
  end

endmodule

`default_nettype wire
