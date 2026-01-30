/*
  iverilog -g2012 -Wall -I./src -o ./vvp/tb_wtile.vvp ./test/tb_w_sram_to_Wtile_col.sv
vvp ./vvp/tb_wtile.vvp
*/

`include "./src/w_sram_to_Wtile_col.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_w_sram_to_Wtile_col_flat_v2;

  localparam int unsigned M      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // control
  logic           start_k;
  logic [K_W-1:0] k_idx;
  wire            col_valid;
  logic           col_accept;

  // SRAM port
  wire                 w_en, w_re, w_we;
  wire [ROW_W-1:0]     w_row;
  wire [K_W-1:0]       w_k;
  wire [DATA_W-1:0]    w_wdata;
  wire [BYTE_W-1:0]    w_wmask;
  logic [DATA_W-1:0]   w_rdata;
  logic                w_rvalid;

  // flat tile
  wire [M*KMAX*DATA_W-1:0] W_tile_flat;

  // DUT
  w_sram_to_Wtile_col #(
    .M(M), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .K_W(K_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .col_valid(col_valid),
    .col_accept(col_accept),

    .w_en(w_en),
    .w_re(w_re),
    .w_we(w_we),
    .w_row(w_row),
    .w_k(w_k),
    .w_wdata(w_wdata),
    .w_wmask(w_wmask),
    .w_rdata(w_rdata),
    .w_rvalid(w_rvalid),

    .W_tile_flat(W_tile_flat)
  );

  // ---------------- golden data ----------------
  function automatic [DATA_W-1:0] mem_value(input int unsigned row, input int unsigned kk);
    mem_value = 32'hA000_0000 + (row << 16) + kk;
  endfunction

  function automatic [DATA_W-1:0] flat_at(input int unsigned row, input int unsigned kk);
    int unsigned idx;
    begin
      idx = (row*KMAX + kk) * DATA_W;
      flat_at = W_tile_flat[idx +: DATA_W];
    end
  endfunction

  // ============================================================
  // Fake SRAM (1-cycle latency) driven by requests w_en&w_re
  // ============================================================
  wire req = (w_en && w_re);

  logic req_d;
  logic [ROW_W-1:0] row_d;
  logic [K_W-1:0]   k_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      req_d    <= 1'b0;
      row_d    <= '0;
      k_d      <= '0;
      w_rvalid <= 1'b0;
      w_rdata  <= '0;
    end else begin
      req_d <= req;
      row_d <= w_row;
      k_d   <= w_k;

      w_rvalid <= req_d;
      w_rdata  <= mem_value(row_d, k_d);
    end
  end

  // ============================================================
  // Scoreboard: count beats by rvalid and verify flat slice.
  // No #delay. Check happens 1 cycle after rvalid (NBA-safe).
  // ============================================================
  int unsigned beats;
  int unsigned errs;

  // pipeline one more stage for checking tile update
  logic chk_pending;
  logic [ROW_W-1:0] chk_row;
  logic [K_W-1:0]   chk_k;
  logic [DATA_W-1:0] chk_exp;

  always_ff @(posedge clk) begin
    if (rst) begin
      beats <= 0;
      errs  <= 0;
      chk_pending <= 1'b0;
      chk_row <= '0;
      chk_k   <= '0;
      chk_exp <= '0;
    end else begin
      // perform check from previous cycle
      if (chk_pending) begin
        if (flat_at(chk_row, chk_k) !== chk_exp) begin
          $display("[FAIL][TILE] row=%0d k=%0d flat=%08x exp=%08x t=%0t",
                   chk_row, chk_k, flat_at(chk_row, chk_k), chk_exp, $time);
          errs <= errs + 1;
        end
        chk_pending <= 1'b0;
      end

      // capture new beat on rvalid
      if (w_rvalid) begin
        beats <= beats + 1;

        chk_row <= row_d;
        chk_k   <= k_d;
        chk_exp <= mem_value(row_d, k_d);
        chk_pending <= 1'b1;
      end
    end
  end

  // ============================================================
  // Tasks
  // ============================================================
  task automatic do_reset();
    begin
      rst = 1'b1;
      start_k = 1'b0;
      col_accept = 1'b0;
      k_idx = '0;
      repeat(5) @(posedge clk);
      rst = 1'b0;
      repeat(2) @(posedge clk);
    end
  endtask

  task automatic start_column(input int unsigned kk);
    begin
      // reset counters for this column
      beats = 0;
      errs  = 0;

      @(negedge clk);
      k_idx   <= kk[K_W-1:0];
      start_k <= 1'b1;
      @(negedge clk);
      start_k <= 1'b0;

      $display("[START_K] k=%0d t=%0t", kk, $time);
    end
  endtask

  task automatic accept_column();
    begin
      @(negedge clk);
      col_accept <= 1'b1;
      @(negedge clk);
      col_accept <= 1'b0;
      $display("[ACCEPT] t=%0t", $time);
    end
  endtask

  // ============================================================
  // Watchdog: if no req/rvalid appears, dump key signals and fail
  // ============================================================
  task automatic watchdog_no_progress(input string tag, input int unsigned cycles);
    int unsigned i;
    begin
      for (i=0; i<cycles; i++) begin
        @(posedge clk);
        if (req || w_rvalid || col_valid) disable watchdog_no_progress;
      end
      $display("[HANG][%s] no progress for %0d cycles at t=%0t", tag, cycles, $time);
      $display("  start_k=%b k_idx=%0d col_valid=%b col_accept=%b", start_k, k_idx, col_valid, col_accept);
      $display("  stubs: req(w_en&w_re)=%b w_en=%b w_re=%b w_row=%0d w_k=%0d w_rvalid=%b",
               req, w_en, w_re, w_row, w_k, w_rvalid);
      $fatal(1);
    end
  endtask

  // ============================================================
  // Waves
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_wcol_v2.vcd");
    $dumpvars(0, tb_w_sram_to_Wtile_col_flat_v2);
  end

  // ============================================================
  // Main test
  // ============================================================
  initial begin
    do_reset();

    // ---------- column k=0 ----------
    fork
      watchdog_no_progress("k0", 200);
    join_none

    start_column(0);

    // progress condition: receive exactly M rvalid beats
    wait(beats == M);
    // allow last pending check to run
    @(posedge clk);

    if (errs != 0) begin
      $display("[FAIL] errs=%0d after k=0", errs);
      $fatal(1);
    end

    // col_valid should be high (nice-to-have)
    if (col_valid !== 1'b1) begin
      $display("[WARN] col_valid not high when beats reached M (col_valid=%b) t=%0t", col_valid, $time);
    end

    accept_column();
    // wait for dut to drop col_valid (if it uses HOLD)
    repeat(2) @(posedge clk);

    // ---------- column k=9 ----------
    fork
      watchdog_no_progress("k9", 200);
    join_none

    start_column(9);

    wait(beats == M);
    @(posedge clk);

    if (errs != 0) begin
      $display("[FAIL] errs=%0d after k=9", errs);
      $fatal(1);
    end

    // spot-check
    $display("W_flat[0,9]=%08x exp=%08x", flat_at(0,9), mem_value(0,9));
    $display("W_flat[7,9]=%08x exp=%08x", flat_at(7,9), mem_value(7,9));

    $display("\n[PASS] w_sram_to_Wtile_col(flat) robust TB OK.");
    $finish;
  end

endmodule

`default_nettype wire
