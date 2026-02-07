/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_matrix_relu.vvp \
  ./test/tb_matrix_relu_cpu.sv

vvp ./vvp/tb_matrix_relu.vvp
*/

`include "./src/EPU/ReLu/top_relu_cpu.sv"   
`timescale 1ns/1ps
`default_nettype none

module tb_matrix_relu_cpu;

  localparam int unsigned NRows  = 8;
  localparam int unsigned NCols  = 8;
  localparam int unsigned NB     = 2;
  localparam int unsigned ADDR_W = 16;
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

  // DUT
  matrix_relu_cpu #(
    .NRows(NRows),
    .NCols(NCols),
    .NB(NB),
    .ADDR_W(ADDR_W),
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

  // clock 100MHz
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

  // -----------------------------
  // FP32 patterns
  // -----------------------------
  localparam logic [31:0] FP32_P0  = 32'h0000_0000; // +0.0
  localparam logic [31:0] FP32_M0  = 32'h8000_0000; // -0.0
  localparam logic [31:0] FP32_P1  = 32'h3F80_0000; // +1.0
  localparam logic [31:0] FP32_M1  = 32'hBF80_0000; // -1.0
  localparam logic [31:0] FP32_P2  = 32'h4000_0000; // +2.0
  localparam logic [31:0] FP32_NEG = 32'hC120_0000; // -10.0
  localparam logic [31:0] FP32_P3  = 32'h4040_0000; // +3.0
  localparam logic [31:0] FP32_M2  = 32'hC000_0000; // -2.0

  function automatic logic [31:0] relu_golden(input logic [31:0] x);
    begin
      relu_golden = x[31] ? 32'h0000_0000 : x;
    end
  endfunction

  // -----------------------------
  // Tasks
  // -----------------------------
  task automatic cpu_write_x(input int r, input int c, input logic [31:0] val);
  begin
    cpu_x_row = r;
    cpu_x_col = c;

    @(negedge clk);
    cpu_x_wdata = val;
    cpu_x_we    = 1'b1;

    @(posedge clk);
    @(negedge clk);
    cpu_x_we    = 1'b0;

    // small gap
    @(posedge clk);
  end
  endtask

  task automatic kick_start();
  begin
    @(negedge clk);
    start = 1'b1;
    @(posedge clk);
    @(negedge clk);
    start = 1'b0;
  end
  endtask

  task automatic wait_done_pulse();
    int i;
  begin
    for (i = 0; i < 2000; i++) begin
      @(posedge clk);
      if (done) disable wait_done_pulse;
    end
    $display("[TB][TIMEOUT] done not seen!");
    $finish;
  end
  endtask

  // Issue 1-cycle read pulse; DUT will latch it into pending
  task automatic cpu_read_y(input int r, input int c, output logic [31:0] y_out);
    int i;
    bit got;
  begin
    got = 0;
    y_out = 'x;

    cpu_y_row = r;
    cpu_y_col = c;

    // 1-cycle pulse
    @(negedge clk);
    cpu_y_re = 1'b1;
    @(posedge clk);
    @(negedge clk);
    cpu_y_re = 1'b0;

    // Wait for rvalid (arbitration may delay; give enough cycles)
    for (i = 0; i < 200; i++) begin
      @(posedge clk);
      #0;
      if (cpu_y_rvalid) begin
        y_out = cpu_y_rdata;
        got = 1;
        disable cpu_read_y;
      end
    end

    if (!got) begin
      $display("[TB][TIMEOUT] cpu_y_rvalid not seen at (%0d,%0d)", r, c);
      $finish;
    end

    // gap before next read request (since DUT supports 1 outstanding)
    repeat (2) @(posedge clk);
  end
  endtask

  // -----------------------------
  // Test vector
  // -----------------------------
  logic [31:0] X [0:NRows-1][0:NCols-1];

  initial begin
    // init signals
    start       = 1'b0;

    cpu_x_we    = 1'b0;
    cpu_x_row   = '0;
    cpu_x_col   = '0;
    cpu_x_wdata = '0;

    cpu_y_re    = 1'b0;
    cpu_y_row   = '0;
    cpu_y_col   = '0;

    // wait reset
    @(posedge rst_n);
    repeat (2) @(posedge clk);

    $display("[TB] Begin matrix_relu_cpu tests...");

    // init patterns
    for (int r = 0; r < NRows; r++) begin
      for (int c = 0; c < NCols; c++) begin
        case ((r*NCols + c) % 8)
          0: X[r][c] = FP32_M1;
          1: X[r][c] = FP32_P1;
          2: X[r][c] = FP32_M0;
          3: X[r][c] = FP32_P0;
          4: X[r][c] = FP32_NEG;
          5: X[r][c] = FP32_P2;
          6: X[r][c] = FP32_M2;
          7: X[r][c] = FP32_P3;
        endcase
      end
    end

    // write X
    $display("[TB] Writing X matrix...");
    for (int r = 0; r < NRows; r++) begin
      for (int c = 0; c < NCols; c++) begin
        cpu_write_x(r, c, X[r][c]);
      end
    end

    // start engine
    $display("[TB] Start ReLU engine...");
    kick_start();

    // wait done
    wait_done_pulse();
    $display("[TB] Done pulse seen.");

    // allow a few cycles for last write to settle
    repeat (5) @(posedge clk);

    // readback + check
    $display("[TB] Reading Y matrix and checking...");
    for (int r = 0; r < NRows; r++) begin
      for (int c = 0; c < NCols; c++) begin
        logic [31:0] y;
        logic [31:0] exp;
        exp = relu_golden(X[r][c]);

        cpu_read_y(r, c, y);

        if (y !== exp) begin
          $display("[TB][FAIL] (%0d,%0d) X=0x%08h expY=0x%08h gotY=0x%08h",
                   r, c, X[r][c], exp, y);
          $finish;
        end
      end
    end

    $display("[TB][PASS] All matrix_relu_cpu tests passed.");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
