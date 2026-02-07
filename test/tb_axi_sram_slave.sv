// tb_sram_mem_ab_mxn.sv
// Run 2x2, 4x4, 8x8 write/read/compare for sram_mem_ab_mxn
//
// Compile (Icarus example):
//   iverilog -g2012 -Wall -o tb_mxn.vvp ./test/tb_axi_sram_slave.sv
//   vvp tb_mxn.vvp
//
// NOTE: This TB assumes your DUT module name is exactly: sram_mem_ab_mxn
// and is visible in the compile (same file or separately compiled).
`include "./src/axi_sram_slave.sv"
`timescale 1ns/1ps
`default_nettype none


// ------------------------------------------------------------
// Reusable test case module for a given (M, KMAX)
// ------------------------------------------------------------
module tb_sram_mem_mn_case #(
  parameter int unsigned M    = 2,
  parameter int unsigned KMAX = 2,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned CONFLICT_POLICY = 1
)();

  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // DUT ports
  logic                 w_en, w_re, w_we;
  logic [ROW_W-1:0]      w_row;
  logic [K_W-1:0]        w_k;
  logic [DATA_W-1:0]     w_wdata;
  logic [BYTE_W-1:0]     w_wmask;
  wire  [DATA_W-1:0]     w_rdata;
  wire                  w_rvalid;

  // results
  logic done = 1'b0;
  integer errors = 0;

  // DUT
  sram_mem_mn #(
    .M(M),
    .KMAX(KMAX),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) dut (
    .clk(clk),
    .rst(rst),

    .w_en(w_en),
    .w_re(w_re),
    .w_we(w_we),
    .w_row(w_row),
    .w_k(w_k),
    .w_wdata(w_wdata),
    .w_wmask(w_wmask),
    .w_rdata(w_rdata),
    .w_rvalid(w_rvalid)
  );

  // deterministic data pattern
  function automatic [DATA_W-1:0] make_data(input integer r, input integer k);
    reg [DATA_W-1:0] x;
    begin
      x = '0;
      x[31:24] = r[7:0];
      x[23:16] = k[7:0];
      x[15:0]  = 16'hA55A ^ (r*17 + k);
      make_data = x;
    end
  endfunction

  // write task (1-cycle pulse)
  task automatic do_write(
    input integer r,
    input integer k,
    input logic [DATA_W-1:0] data,
    input logic [BYTE_W-1:0] mask
  );
    begin
      @(negedge clk);
      w_en    <= 1'b1;
      w_we    <= 1'b1;
      w_re    <= 1'b0;
      w_row   <= r[ROW_W-1:0];
      w_k     <= k[K_W-1:0];
      w_wdata <= data;
      w_wmask <= mask;

      @(posedge clk); // write happens here

      @(negedge clk);
      w_en    <= 1'b0;
      w_we    <= 1'b0;
      w_wdata <= '0;
      w_wmask <= '0;
    end
  endtask

  // read & check task (wait rvalid with timeout)
  task automatic do_read_check(
    input integer r,
    input integer k,
    input logic [DATA_W-1:0] expected
  );
    integer tmo;
    begin
      // issue read request spanning a posedge
      @(negedge clk);
      w_en  <= 1'b1;
      w_re  <= 1'b1;
      w_we  <= 1'b0;
      w_row <= r[ROW_W-1:0];
      w_k   <= k[K_W-1:0];

      @(posedge clk); // request sampled

      @(negedge clk);
      w_en <= 1'b0;
      w_re <= 1'b0;

      // wait for rvalid (DUT gives 1-cycle latency, but we don't assume)
      tmo = 0;
      while (w_rvalid !== 1'b1 && tmo < 10) begin
        @(posedge clk);
        tmo = tmo + 1;
      end

      if (w_rvalid !== 1'b1) begin
        $display("[%0t] ERROR M=%0d KMAX=%0d: rvalid timeout at (r=%0d,k=%0d)",
                 $time, M, KMAX, r, k);
        errors = errors + 1;
      end else if (w_rdata !== expected) begin
        $display("[%0t] ERROR M=%0d KMAX=%0d: mismatch at (r=%0d,k=%0d) exp=%h got=%h",
                 $time, M, KMAX, r, k, expected, w_rdata);
        errors = errors + 1;
      end
    end
  endtask
    // read only (returns data via output argument), waits rvalid with timeout
  task automatic do_read(
    input  integer r,
    input  integer k,
    output logic [DATA_W-1:0] data
  );
    integer tmo;
    begin
      data = '0;

      @(negedge clk);
      w_en  <= 1'b1;
      w_re  <= 1'b1;
      w_we  <= 1'b0;
      w_row <= r[ROW_W-1:0];
      w_k   <= k[K_W-1:0];

      @(posedge clk);

      @(negedge clk);
      w_en <= 1'b0;
      w_re <= 1'b0;

      tmo = 0;
      while (w_rvalid !== 1'b1 && tmo < 10) begin
        @(posedge clk);
        tmo = tmo + 1;
      end

      if (w_rvalid !== 1'b1) begin
        $display("[%0t] DUMP ERROR: rvalid timeout at (r=%0d,k=%0d)", $time, r, k);
        data = 'x;
      end else begin
        data = w_rdata;
      end
    end
  endtask

  // dump matrix M x KMAX
  task automatic dump_matrix(input string tag);
    logic [DATA_W-1:0] v;
    integer rr, kk;
    begin
      $display("");
      $display("==== DUMP %s : M=%0d KMAX=%0d ====", tag, M, KMAX);

      for (rr = 0; rr < M; rr = rr + 1) begin
        $write("row %0d: ", rr);
        for (kk = 0; kk < KMAX; kk = kk + 1) begin
          do_read(rr, kk, v);
          $write("%08h ", v);
        end
        $write("\n");
      end

      $display("==== END DUMP %s ====", tag);
      $display("");
    end
  endtask

  // partial write (byte mask) sanity test at (0,0)
  task automatic do_mask_test;
  logic [DATA_W-1:0] d0, d1, exp;
  begin
    d0 = 32'h11223344;
    d1 = 32'hAABBCCDD;

    // 先全寫成 d0，確保 cell 不是 0
    do_write(0, 0, d0, 4'b1111);

    // 再只寫高 2 bytes
    do_write(0, 0, d1, 4'b1100);

    exp = {d1[31:16], d0[15:0]};   // AA BB + 33 44
    do_read_check(0, 0, exp);
  end
endtask


  // main
  integer r, k;
    initial begin
    // init
    w_en = 0; w_re = 0; w_we = 0;
    w_row = '0; w_k = '0; w_wdata = '0; w_wmask = '0;

    // reset
    rst = 1'b1;
    repeat (3) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("---- RUN CASE: M=%0d KMAX=%0d ----", M, KMAX);

    // 1) write all
    for (r = 0; r < M; r = r + 1)
      for (k = 0; k < KMAX; k = k + 1)
        do_write(r, k, make_data(r,k), {BYTE_W{1'b1}});

    // 2) read back & compare
    for (r = 0; r < M; r = r + 1)
      for (k = 0; k < KMAX; k = k + 1)
        do_read_check(r, k, make_data(r,k));

    // 🔹 印出整個矩陣（全寫後）
    dump_matrix("after full write");

    // 3) mask test
    if (M > 0 && KMAX > 0)
      do_mask_test();

    // 🔹 再印一次（mask 後）
    dump_matrix("after mask test");

    if (errors == 0)
      $display("---- PASS: M=%0d KMAX=%0d ----", M, KMAX);
    else
      $display("---- FAIL: M=%0d KMAX=%0d errors=%0d ----", M, KMAX, errors);

    done = 1'b1;
  end

endmodule

// ------------------------------------------------------------
// Top: run 2x2, 4x4, 8x8
// ------------------------------------------------------------
module tb_sram_mem_mn_all;

  tb_sram_mem_mn_case #(.M(2), .KMAX(2)) u_2x2();
  tb_sram_mem_mn_case #(.M(4), .KMAX(4)) u_4x4();
  tb_sram_mem_mn_case #(.M(8), .KMAX(8)) u_8x8();

  integer total_err;

  initial begin
    wait (u_2x2.done && u_4x4.done && u_8x8.done);

    total_err = u_2x2.errors + u_4x4.errors + u_8x8.errors;

    $display("======================================");
    $display("ALL DONE. total_errors = %0d", total_err);
    $display("======================================");
    
    if (total_err != 0) $finish;
    else $finish;
  end

endmodule

`default_nettype wire
