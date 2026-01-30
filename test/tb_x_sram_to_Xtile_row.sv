/*
  tb_x_sram_to_Xtile_row_flat_v2.sv

  compile:
    iverilog -g2012 -Wall -I./src -o ./vvp/tb_xrow_flat.vvp ./test/tb_x_sram_to_Xtile_row.sv
  run:
    vvp ./vvp/tb_xrow_flat.vvp
*/

`include "./src/x_sram_to_Xtile_row.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_x_sram_to_Xtile_row;

  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // DUT control
  logic           start_k;
  logic [K_W-1:0] k_idx;
  wire            row_valid;
  logic           row_accept;

  // SRAM port
  wire                 x_en, x_re, x_we;
  wire [K_W-1:0]        x_k;
  wire [N_W-1:0]        x_n;
  wire [DATA_W-1:0]     x_wdata;
  wire [BYTE_W-1:0]     x_wmask;
  logic [DATA_W-1:0]    x_rdata;
  logic                x_rvalid;

  // FLAT tile
  wire [KMAX*N*DATA_W-1:0] X_tile_flat;

  // DUT
  x_sram_to_Xtile_row #(
    .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .N_W(N_W), .K_W(K_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .row_valid(row_valid),
    .row_accept(row_accept),

    .x_en(x_en),
    .x_re(x_re),
    .x_we(x_we),
    .x_k(x_k),
    .x_n(x_n),
    .x_wdata(x_wdata),
    .x_wmask(x_wmask),
    .x_rdata(x_rdata),
    .x_rvalid(x_rvalid),

    .X_tile_flat(X_tile_flat)
  );

  // ---------------- golden pattern ----------------
  function automatic [DATA_W-1:0] mem_value(input int unsigned kk, input int unsigned nn);
    mem_value = 32'hB000_0000 + (kk << 16) + nn;
  endfunction

  function automatic [DATA_W-1:0] flat_at(input int unsigned kk, input int unsigned nn);
    int unsigned idx;
    begin
      idx = (kk*N + nn) * DATA_W;
      flat_at = X_tile_flat[idx +: DATA_W];
    end
  endfunction

  // ============================================================
  // Fake SRAM (1-cycle latency) driven by request x_en & x_re
  // ============================================================
  wire req = (x_en && x_re);

  logic req_d;
  logic [K_W-1:0] k_d;
  logic [N_W-1:0] n_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      req_d    <= 1'b0;
      k_d      <= '0;
      n_d      <= '0;
      x_rvalid <= 1'b0;
      x_rdata  <= '0;
    end else begin
      req_d <= req;
      k_d   <= x_k;
      n_d   <= x_n;

      x_rvalid <= req_d;
      x_rdata  <= mem_value(k_d, n_d);
    end
  end

  // ============================================================
  // Scoreboard: count beats by rvalid and verify flat slice.
  // Check happens 1 cycle after rvalid (NBA-safe).
  // ============================================================
  int unsigned beats;
  int unsigned errs;

  logic chk_pending;
  logic [K_W-1:0] chk_k;
  logic [N_W-1:0] chk_n;
  logic [DATA_W-1:0] chk_exp;

  always_ff @(posedge clk) begin
    if (rst) begin
      beats <= 0;
      errs  <= 0;
      chk_pending <= 1'b0;
      chk_k <= '0;
      chk_n <= '0;
      chk_exp <= '0;
    end else begin
      // check previous beat
      if (chk_pending) begin
        if (flat_at(chk_k, chk_n) !== chk_exp) begin
          $display("[FAIL][TILE] k=%0d n=%0d flat=%08x exp=%08x t=%0t",
                   chk_k, chk_n, flat_at(chk_k, chk_n), chk_exp, $time);
          errs <= errs + 1;
        end
        chk_pending <= 1'b0;
      end

      // capture new beat
      if (x_rvalid) begin
        beats <= beats + 1;
        chk_k <= k_d;
        chk_n <= n_d;
        chk_exp <= mem_value(k_d, n_d);
        chk_pending <= 1'b1;
      end
    end
  end

  // ---------------- tasks ----------------
  task automatic do_reset();
    begin
      rst = 1'b1;
      start_k = 1'b0;
      row_accept = 1'b0;
      k_idx = '0;
      repeat(5) @(posedge clk);
      rst = 1'b0;
      repeat(2) @(posedge clk);
    end
  endtask

  task automatic start_row(input int unsigned kk);
    begin
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

  task automatic accept_row();
    begin
      @(negedge clk);
      row_accept <= 1'b1;
      @(negedge clk);
      row_accept <= 1'b0;
      $display("[ACCEPT] t=%0t", $time);
    end
  endtask

  // watchdog
  task automatic watchdog_no_progress(input string tag, input int unsigned cycles);
    int unsigned i;
    begin
      for (i=0; i<cycles; i++) begin
        @(posedge clk);
        if (req || x_rvalid || row_valid) disable watchdog_no_progress;
      end
      $display("[HANG][%s] no progress for %0d cycles at t=%0t", tag, cycles, $time);
      $display("  start_k=%b k_idx=%0d row_valid=%b row_accept=%b", start_k, k_idx, row_valid, row_accept);
      $display("  req=%b x_en=%b x_re=%b x_k=%0d x_n=%0d x_rvalid=%b",
               req, x_en, x_re, x_k, x_n, x_rvalid);
      $fatal(1);
    end
  endtask

  // ---------------- waves ----------------
 
  // ---------------- main ----------------
  initial begin
    do_reset();

    // ---- test k=0 ----
    fork watchdog_no_progress("k0", 200); join_none
    start_row(0);

    // completion: N returned beats
    wait(beats == N);
    @(posedge clk); // let last check run

    if (errs != 0) begin
      $display("[FAIL] errs=%0d after k=0", errs);
      $fatal(1);
    end

    if (row_valid !== 1'b1)
      $display("[WARN] row_valid not high when beats reached N (row_valid=%b) t=%0t", row_valid, $time);

    accept_row();
    repeat(2) @(posedge clk);

    // ---- test k=9 ----
    fork watchdog_no_progress("k9", 200); join_none
    start_row(9);

    wait(beats == N);
    @(posedge clk);

    if (errs != 0) begin
      $display("[FAIL] errs=%0d after k=9", errs);
      $fatal(1);
    end

    // spot-check
    $display("X_flat[9,0]=%08x exp=%08x", flat_at(9,0), mem_value(9,0));
    $display("X_flat[9,7]=%08x exp=%08x", flat_at(9,7), mem_value(9,7));

    $display("\n[PASS] x_sram_to_Xtile_row(flat) TB OK.");
    $finish;
  end

endmodule

`default_nettype wire
