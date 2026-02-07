/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_top_transpose.vvp ./test/tb_transpose.sv

vvp ./vvp/tb_top_transpose.vvp
*/
`include "./src/EPU/transpose/transpose_crtl.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_top_transpose_cpu_fp32;

  localparam integer NRows  = 8;
  localparam integer NCols  = 8;
  localparam integer Data_W = 32;  // FP32 bits
  localparam integer ADDR_W = 16;
  localparam integer NB     = 2;
  localparam integer M      = 6;

  reg clk;
  reg rst_n;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // DUT IO
  reg start;
  wire done;
  wire busy;

  reg              cpu_a_we;
  reg [31:0]       cpu_a_row, cpu_a_col;
  reg [Data_W-1:0] cpu_a_wdata;

  reg              cpu_b_re;
  reg [31:0]       cpu_b_row, cpu_b_col;
  wire [Data_W-1:0] cpu_b_rdata;
  wire             cpu_b_rvalid;

  top_transpose_cpu #(
    .NRows(NRows), .NCols(NCols),
    .NB(NB), .ADDR_W(ADDR_W),
    .Data_W(Data_W), .M(M)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .start(start), .done(done), .busy(busy),

    .cpu_a_we(cpu_a_we),
    .cpu_a_row(cpu_a_row), .cpu_a_col(cpu_a_col),
    .cpu_a_wdata(cpu_a_wdata),

    .cpu_b_re(cpu_b_re),
    .cpu_b_row(cpu_b_row), .cpu_b_col(cpu_b_col),
    .cpu_b_rdata(cpu_b_rdata),
    .cpu_b_rvalid(cpu_b_rvalid)
  );
  function [31:0] fp32_bits;
  input integer r;
  input integer c;
  begin
    // base = 1.0f in IEEE754, then add deterministic offsets to make each cell unique
    fp32_bits = 32'h3f800000 + (r << 8) + c;
  end
  endfunction


  task wait_cycles;
    input integer n;
    integer i;
    begin
      for (i=0;i<n;i=i+1) @(posedge clk);
    end
  endtask

  task cpu_write_A;
    input integer r;
    input integer c;
    input [31:0] bits;
    begin
      @(negedge clk);
      cpu_a_we    <= 1'b1;
      cpu_a_row   <= r;
      cpu_a_col   <= c;
      cpu_a_wdata <= bits;
      @(negedge clk);
      cpu_a_we    <= 1'b0;
    end
  endtask

  task cpu_read_B;
    input integer r;
    input integer c;
    output [31:0] bits;
    integer guard;
    begin
      bits = 32'h0;
      guard = 0;

      @(negedge clk);
      cpu_b_re  <= 1'b1;
      cpu_b_row <= r;
      cpu_b_col <= c;
      @(negedge clk);
      cpu_b_re  <= 1'b0;

      // wait rvalid
      while (!cpu_b_rvalid) begin
        @(posedge clk);
        guard = guard + 1;
        if (guard > 5000) begin
          $display("[TB] cpu_read_B TIMEOUT r=%0d c=%0d", r, c);
          $fatal(1);
        end
      end
      bits = cpu_b_rdata;
    end
  endtask

  integer r,c;
  reg [31:0] got, exp;

  initial begin
    $dumpfile("./vvp/tb_tp_cpu.vcd");
    $dumpvars(0, tb_top_transpose_cpu_fp32);

    // init
    rst_n = 1'b0;
    start = 1'b0;

    cpu_a_we = 1'b0; cpu_a_row = 0; cpu_a_col = 0; cpu_a_wdata = 0;
    cpu_b_re = 1'b0; cpu_b_row = 0; cpu_b_col = 0;

    wait_cycles(5);
    rst_n = 1'b1;
    wait_cycles(5);

    // preload A (only when not busy)
    $display("[TB] preload A FP32...");
    for (r=0;r<NRows;r=r+1) begin
      for (c=0;c<NCols;c=c+1) begin
        cpu_write_A(r,c, fp32_bits(r,c));
      end
    end

    // start transpose
    $display("[TB] start transpose...");
    @(negedge clk);
    start <= 1'b1;
    @(negedge clk);
    start <= 1'b0;

    // wait done
    $display("[TB] wait done...");
    while (!done) @(posedge clk);
    $display("[TB] done!");

    // read back B[col,row] and compare to A[row,col]
    $display("[TB] check B transpose...");
    for (r=0;r<NRows;r=r+1) begin
      for (c=0;c<NCols;c=c+1) begin
        // B is stored as [rowp=col][colp=row]
        cpu_read_B(c, r, got);
        exp = fp32_bits(r,c);
        if (got !== exp) begin
          $display("[TB][FAIL] A(%0d,%0d)=0x%08x but B(%0d,%0d)=0x%08x",
                   r,c,exp, c,r,got);
          $fatal(1);
        end
      end
    end

    $display("[TB][PASS] FP32 transpose correct.");
    wait_cycles(10);
    $finish;
  end

endmodule

`default_nettype wire
