// tb_systolic_wrap_3cases_fixedexp.sv
/*
iverilog -g2012 -Wall -I. -o ./vvp/tb_wrap_3cases.vvp ./test/tb_systolic_wrap_fixedexp.sv
 vvp ./vvp/tb_wrap_3cases.vvp
*/
`include "./src/systolic_wrap_c_sram.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_systolic_wrap_c_sram_flat;

  // -----------------------------
  // Parameters (keep consistent)
  // -----------------------------
  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);

  // Choose a small K for test (must be <= KMAX)
  localparam int unsigned K_LEN  = 4;

  // -----------------------------
  // DUT I/O
  // -----------------------------
  logic clk, rst;

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

  // -----------------------------
  // Instantiate DUT
  // -----------------------------
  systolic_wrap_c_sram_flat #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(1)
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

  // -----------------------------
  // Clock / Reset
  // -----------------------------
  initial clk = 0;
  always #5 clk = ~clk; // 100MHz

  task automatic do_reset();
    begin
      rst = 1;
      start = 0;
      K_len = '0;

      c_rd_en  = 0;
      c_rd_re  = 0;
      c_rd_row = '0;
      c_rd_col = '0;

      W_tile_flat = '0;
      X_tile_flat = '0;

      repeat (10) @(posedge clk);
      rst = 0;
      repeat (5) @(posedge clk);
    end
  endtask

  // -----------------------------
  // Test data storage
  // -----------------------------
  logic signed [DATA_W-1:0] W_mat [M][K_LEN];
  logic signed [DATA_W-1:0] X_mat [K_LEN][N];
  logic signed [DATA_W-1:0] C_golden [M][N];

  // -----------------------------
  // Helpers: pack flat buses
  // W_tile_flat index: (r*KMAX + k) * DATA_W
  // X_tile_flat index: (k*N + n) * DATA_W
  // -----------------------------
  task automatic pack_W_flat();
    int r,k;
    int unsigned idx;
    begin
      W_tile_flat = '0;
      for (r = 0; r < M; r++) begin
        for (k = 0; k < K_LEN; k++) begin
          idx = (r*KMAX + k) * DATA_W;
          W_tile_flat[idx +: DATA_W] = W_mat[r][k];
        end
      end
    end
  endtask

  task automatic pack_X_flat();
    int k,n;
    int unsigned idx;
    begin
      X_tile_flat = '0;
      for (k = 0; k < K_LEN; k++) begin
        for (n = 0; n < N; n++) begin
          idx = (k*N + n) * DATA_W;
          X_tile_flat[idx +: DATA_W] = X_mat[k][n];
        end
      end
    end
  endtask
  int tmo = 0;
  // -----------------------------
  // Golden model: C = W * X
  // -----------------------------
  task automatic compute_golden();
    int i,j,k;
    longint signed acc;
    begin
      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          acc = 0;
          for (k = 0; k < K_LEN; k++) begin
            acc += longint'(W_mat[i][k]) * longint'(X_mat[k][j]);
          end
          C_golden[i][j] = logic'(acc[DATA_W-1:0]); // wrap/truncate to DATA_W
        end
      end
    end
  endtask

  // -----------------------------
  // CPU read task: waits for rvalid
  // -----------------------------
  task automatic cpu_read_c(
    input  int unsigned row,
    input  int unsigned col,
    output logic [DATA_W-1:0] data
  );
    int timeout;
    begin
      // Issue read request (1-cycle pulse)
      @(posedge clk);
      c_rd_en  <= 1'b1;
      c_rd_re  <= 1'b1;
      c_rd_row <= row[ROW_W-1:0];
      c_rd_col <= col[COL_W-1:0];

      @(posedge clk);
      c_rd_en  <= 1'b0;
      c_rd_re  <= 1'b0;

      // Wait for rvalid
      timeout = 0;
      while (c_rd_rvalid !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 2000) begin
          $fatal(1, "[TB] cpu_read_c timeout waiting rvalid at row=%0d col=%0d", row, col);
        end
      end

      data = c_rd_rdata;
      // (optional) wait one more cycle to avoid back-to-back hazard
      @(posedge clk);
    end
  endtask

  // -----------------------------
  // Stimulus
  // -----------------------------
  task automatic init_matrices_simple();
    int i,j;
    begin
      // deterministic, easy to eyeball
      for (i = 0; i < M; i++) begin
        for (j = 0; j < K_LEN; j++) begin
          W_mat[i][j] = $signed(i + j + 1); // 1.. small
        end
      end
      for (i = 0; i < K_LEN; i++) begin
        for (j = 0; j < N; j++) begin
          X_mat[i][j] = $signed((i+1) * (j+1)); // multiplication table
        end
      end
    end
  endtask

  task automatic run_one_case();
    int unsigned i,j;
    logic [DATA_W-1:0] got;
    int err;
    begin
      err = 0;

      init_matrices_simple();
      pack_W_flat();
      pack_X_flat();
      compute_golden();

      // fire start
      K_len = K_LEN[15:0];
      @(posedge clk);
      start <= 1'b1;
      @(posedge clk);
      start <= 1'b0;

      // wait done (timeout)
      
      while (done !== 1'b1) begin
        @(posedge clk);
        tmo++;
        if (tmo > 200000) begin
          $fatal(1, "[TB] timeout waiting done");
        end
      end

      $display("[TB] done asserted, start CPU reading SRAM...");

      // read all C and compare
      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          cpu_read_c(i, j, got);

          if (got !== C_golden[i][j]) begin
            err++;
            $display("[MIS] C[%0d][%0d] got=0x%08x exp=0x%08x (dec got=%0d exp=%0d)",
                     i, j, got, C_golden[i][j], $signed(got), $signed(C_golden[i][j]));
          end
        end
      end

      if (err == 0) begin
        $display("[TB] PASS: all C entries match golden.");
      end else begin
        $fatal(1, "[TB] FAIL: %0d mismatches.", err);
      end
    end
  endtask

  // -----------------------------
  // Main
  // -----------------------------
  initial begin
    do_reset();
    run_one_case();
    $display("[TB] all tests finished.");
    $finish;
  end

endmodule

`default_nettype wire
