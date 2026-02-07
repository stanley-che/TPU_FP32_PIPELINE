/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_top_relu.vvp \
  ./test/tb_top_relu_cpu.sv

vvp ./vvp/tb_top_relu.vvp
*/

`include "./src/EPU/ReLu/ONE_BIT_relu_cpu.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_top_relu_cpu;

  localparam int unsigned Data_W = 32;

  // clk/rst
  logic clk;
  logic rst_n;

  // DUT I/O
  logic start;
  logic done;
  logic busy;

  logic              cpu_x_we;
  logic [31:0]       cpu_x_row, cpu_x_col;
  logic [Data_W-1:0] cpu_x_wdata;

  logic              cpu_y_re;
  logic [31:0]       cpu_y_row, cpu_y_col;
  logic [Data_W-1:0] cpu_y_rdata;
  logic              cpu_y_rvalid;

  // instantiate DUT
  ONE_BIT_relu_cpu #(
    .NRows(8),
    .NCols(8),
    .NB(2),
    .ADDR_W(16),
    .Data_W(Data_W)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .done(done),
    .busy(busy),

    .cpu_x_we(cpu_x_we),
    .cpu_x_row(cpu_x_row),
    .cpu_x_col(cpu_x_col),
    .cpu_x_wdata(cpu_x_wdata),

    .cpu_y_re(cpu_y_re),
    .cpu_y_row(cpu_y_row),
    .cpu_y_col(cpu_y_col),
    .cpu_y_rdata(cpu_y_rdata),
    .cpu_y_rvalid(cpu_y_rvalid)
  );

  // clock
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // reset
  initial begin
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
  end

  // FP32 patterns
  localparam logic [31:0] FP32_P0  = 32'h0000_0000; // +0.0
  localparam logic [31:0] FP32_P1  = 32'h3F80_0000; // +1.0
  localparam logic [31:0] FP32_M1  = 32'hBF80_0000; // -1.0
  localparam logic [31:0] FP32_M0  = 32'h8000_0000; // -0.0
  localparam logic [31:0] FP32_P2  = 32'h4000_0000; // +2.0
  localparam logic [31:0] FP32_NEG = 32'hC120_0000; // -10.0

  // helpers
  task automatic write_x(input logic [31:0] in_x);
  begin
    cpu_x_row   = 32'd0;
    cpu_x_col   = 32'd0;

    @(negedge clk);
    cpu_x_wdata = in_x;
    cpu_x_we    = 1'b1;

    @(posedge clk);
    @(negedge clk);
    cpu_x_we    = 1'b0;

    @(posedge clk);
  end
endtask


  task automatic start_relu();
    begin
      start = 1'b1;
      @(posedge clk);
      start = 1'b0;
    end
  endtask

  task automatic wait_done_pulse();
    integer i;
    begin
      // done 在你的設計是 1-cycle pulse，所以這邊等它出現
      for (i = 0; i < 50; i = i + 1) begin
        @(posedge clk);
        if (done) disable wait_done_pulse;
      end
      $display("[TB][TIMEOUT] done pulse not seen!");
      $finish;
    end
  endtask

  task automatic read_y(output logic [31:0] y_out);
  integer i;
begin
  cpu_y_row = 32'd0;
  cpu_y_col = 32'd0;

  @(negedge clk);
  cpu_y_re = 1'b1;

  for (i=0; i<10; i++) begin
    @(posedge clk);
    #0;
    if (cpu_y_rvalid) begin
      y_out = cpu_y_rdata;
      disable read_y;
    end
  end

  $display("[TB][TIMEOUT] cpu_y_rvalid not seen!");
  $finish;

  @(negedge clk);
  cpu_y_re = 1'b0;
end
endtask



  task automatic check_relu(
  input logic [31:0] in_val,
  input logic [31:0] exp_val
);
  // ⚠️ 所有宣告一定放最前面
  logic [31:0] y_val;

  begin
    $display("[TB] X=0x%08h expect Y=0x%08h", in_val, exp_val);

    write_x(in_val);

    start_relu();
    wait_done_pulse();

    read_y(y_val);

    if (y_val !== exp_val) begin
      $display("[FAIL] X=0x%08h expect Y=0x%08h got Y=0x%08h",
               in_val, exp_val, y_val);
      $finish;
    end else begin
      $display("[PASS] Y=0x%08h", y_val);
    end
  end
endtask


  // init + run
  initial begin
    // init signals
    start      = 1'b0;
    cpu_x_we   = 1'b0;
    cpu_x_row  = 32'd0;
    cpu_x_col  = 32'd0;
    cpu_x_wdata= 32'd0;

    cpu_y_re   = 1'b0;
    cpu_y_row  = 32'd0;
    cpu_y_col  = 32'd0;

    // wait reset release
    @(posedge rst_n);
    repeat (2) @(posedge clk);

    $display("[TB] Begin top_relu_cpu tests...");

    // relu(-1.0) = 0
    check_relu(FP32_M1, FP32_P0);

    // relu(+1.0) = +1.0
    check_relu(FP32_P1, FP32_P1);

    // relu(-0.0) = 0
    check_relu(FP32_M0, FP32_P0);

    // relu(+2.0) = +2.0
    check_relu(FP32_P2, FP32_P2);

    // relu(-10.0) = 0
    check_relu(FP32_NEG, FP32_P0);

    $display("[TB][PASS] All top_relu_cpu tests passed.");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
