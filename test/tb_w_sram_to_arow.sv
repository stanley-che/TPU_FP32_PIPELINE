/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_w.vvp ./test/tb_w_sram_to_arow.sv 
vvp ./vvp/tb_w.vvp

*/

// tb_w_sram_to_arow.sv
`include "./src/w_sram_to_arow.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_w_sram_to_arow;

  // ----------------------------
  // Parameters 
  // ----------------------------
  localparam int unsigned M      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // ----------------------------
  // DUT I/O
  // ----------------------------
  logic                 start_k;
  logic [K_W-1:0]        k_idx;
  wire                  arow_valid;
  logic                 arow_accept;

  wire                  w_en;
  wire                  w_re;
  wire                  w_we;
  wire [ROW_W-1:0]      w_row;
  wire [K_W-1:0]        w_k;
  wire [DATA_W-1:0]     w_wdata;
  wire [BYTE_W-1:0]     w_wmask;

  logic [DATA_W-1:0]    w_rdata;
  logic                 w_rvalid;

  wire [DATA_W-1:0]     a_row [M];

  // ----------------------------
  // Instantiate DUT
  // ----------------------------
  w_sram_to_arow #(
    .M(M),
    .KMAX(KMAX),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .arow_valid(arow_valid),
    .arow_accept(arow_accept),

    .w_en(w_en),
    .w_re(w_re),
    .w_we(w_we),
    .w_row(w_row),
    .w_k(w_k),
    .w_wdata(w_wdata),
    .w_wmask(w_wmask),
    .w_rdata(w_rdata),
    .w_rvalid(w_rvalid),

    .a_row(a_row)
  );

  // ----------------------------
  // Simple SRAM model (read-only)
  // 1-cycle latency: when (w_en && w_re) at cycle t,
  // output rvalid=1 and rdata at cycle t+1.
  // ----------------------------
  logic                 pend_valid;
  logic [ROW_W-1:0]      pend_row;
  logic [K_W-1:0]        pend_k;

  function automatic [DATA_W-1:0] mem_value(input int unsigned row, input int unsigned kk);
    mem_value = 32'hA000_0000 + (row << 16) + kk;
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      pend_valid <= 1'b0;
      pend_row   <= '0;
      pend_k     <= '0;
      w_rvalid   <= 1'b0;
      w_rdata    <= '0;
    end else begin
      // default: rvalid pulses for 1 cycle when pending completes
      w_rvalid <= pend_valid;

      if (pend_valid) begin
        w_rdata <= mem_value(pend_row, pend_k);
      end

      // capture new request
      pend_valid <= (w_en && w_re);
      if (w_en && w_re) begin
        pend_row <= w_row;
        pend_k   <= w_k;
      end
    end
  end

  // ----------------------------
  // Test helpers
  // ----------------------------
  task automatic pulse_start(input int unsigned kk);
    begin
      @(negedge clk);
      k_idx   <= kk[K_W-1:0];
      start_k <= 1'b1;
      @(negedge clk);
      start_k <= 1'b0;
    end
  endtask

  task automatic expect_row_for_k(input int unsigned kk);
    int unsigned r;
    begin
      // wait until adaptor says row is ready
      while (arow_valid !== 1'b1) @(posedge clk);

      // check contents
      for (r = 0; r < M; r++) begin
        if (a_row[r] !== mem_value(r, kk)) begin
          $display("[FAIL] k=%0d row=%0d got=0x%08x exp=0x%08x @t=%0t",
                   kk, r, a_row[r], mem_value(r, kk), $time);
          $fatal(1);
        end
      end
      $display("[PASS] a_row matches for k=%0d @t=%0t", kk, $time);

      // accept for 1 cycle
      @(negedge clk);
      arow_accept <= 1'b1;
      @(negedge clk);
      arow_accept <= 1'b0;

      // ensure valid drops (not strictly required same cycle, but should drop after accept)
      repeat (2) @(posedge clk);
      if (arow_valid !== 1'b0) begin
        $display("[FAIL] arow_valid did not drop after accept for k=%0d @t=%0t", kk, $time);
        $fatal(1);
      end
    end
  endtask

  // ----------------------------
  // Main stimulus
  // ----------------------------
  initial begin
    // init
    start_k     = 1'b0;
    k_idx       = '0;
    arow_accept = 1'b0;

    // reset
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // Test k = 0
    pulse_start(0);
    expect_row_for_k(0);

    // Test k = 7
    pulse_start(7);
    expect_row_for_k(7);

    // Test back-to-back start while not IDLE (should be ignored in your FSM)
    // We'll assert start twice quickly; only the first should take effect.
    pulse_start(3);
    // try to start another k immediately (while busy)
    @(negedge clk);
    k_idx   <= 9;
    start_k <= 1'b1;
    @(negedge clk);
    start_k <= 1'b0;

    // should still deliver k=3 first
    expect_row_for_k(3);
    // then we can explicitly start k=9
    pulse_start(9);
    expect_row_for_k(9);

    $display("All tests passed.");
    $finish;
  end

endmodule

`default_nettype wire
