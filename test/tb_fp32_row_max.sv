    // tb_fp32_row_max.sv
    // ------------------------------------------------------------
    // iverilog -g2012 -Wall -I./src -o ./vvp/tb_fp32_row_max.vvp ./test/tb_fp32_row_max.sv
    // vvp ./vvp/tb_fp32_row_max.vvp
    // ------------------------------------------------------------
`include "./src/EPU/attention_score/fp32_row_max.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_fp32_row_max;

  localparam int T = 4;

  logic clk, rst_n;

  // DUT IO
  logic        in_valid;
  logic [31:0] in_fp32;
  logic        row_start;
  logic        row_last;

  logic        max_valid;
  logic [31:0] max_fp32;

  // ----------------------------
  // clock / reset
  // ----------------------------
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
  end

  // ----------------------------
  // DUT
  // ----------------------------
  fp32_row_max #(.T(T)) dut (
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .in_fp32(in_fp32),
    .row_start(row_start),
    .row_last(row_last),
    .max_valid(max_valid),
    .max_fp32(max_fp32)
  );

  // ----------------------------
  // fp32 -> real (for print)
  // ----------------------------
  function automatic real fp32_to_real(input logic [31:0] b);
    int  s, e, frac;
    real mant, val;
    begin
      s    = b[31];
      e    = b[30:23];
      frac = b[22:0];

      if (e == 0) begin
        val = 0.0;
      end else begin
        mant = 1.0 + frac / 8388608.0;
        val  = mant * (2.0 ** (e - 127));
      end
      fp32_to_real = s ? -val : val;
    end
  endfunction

  // ----------------------------
  // helper: send one row
  // ----------------------------
  task automatic send_row(
    input logic [31:0] v0,
    input logic [31:0] v1,
    input logic [31:0] v2,
    input logic [31:0] v3,
    input logic [31:0] expect_max
  );
    begin
      // element 0
      @(posedge clk);
      in_valid  <= 1'b1;
      row_start <= 1'b1;
      row_last  <= 1'b0;
      in_fp32   <= v0;

      // element 1
      @(posedge clk);
      row_start <= 1'b0;
      in_fp32   <= v1;

      // element 2
      @(posedge clk);
      in_fp32   <= v2;

      // element 3 (last)
      @(posedge clk);
      row_last  <= 1'b1;
      in_fp32   <= v3;

      // end
      @(posedge clk);
      in_valid  <= 1'b0;
      row_last  <= 1'b0;
      in_fp32   <= 32'h0;

      // wait for max_valid
      while (max_valid !== 1'b1) @(posedge clk);

      if (max_fp32 !== expect_max) begin
        $display("[FAIL] row max = %f, expect %f",
          fp32_to_real(max_fp32),
          fp32_to_real(expect_max)
        );
        $fatal;
      end else begin
        $display("[PASS] row max = %f",
          fp32_to_real(max_fp32)
        );
      end
    end
  endtask

  // ----------------------------
  // stimulus
  // ----------------------------
  initial begin
    in_valid  = 0;
    row_start = 0;
    row_last  = 0;
    in_fp32   = 0;

    @(posedge rst_n);
    repeat (2) @(posedge clk);

    $display("=== Test row max ===");

    // row 0 : [0.1, 0.4, 0.2, 0.3] → 0.4
    send_row(
      32'h3DCCCCCD, // 0.1
      32'h3ECCCCCD, // 0.4
      32'h3E4CCCCD, // 0.2
      32'h3E99999A, // 0.3
      32'h3ECCCCCD
    );

    // row 1 : [-1.0, -0.5, -2.0, -0.25] → -0.25
    send_row(
      32'hBF800000, // -1.0
      32'hBF000000, // -0.5
      32'hC0000000, // -2.0
      32'hBE800000, // -0.25
      32'hBE800000
    );

    // row 2 : [5, 3, 7, 6] → 7
    send_row(
      32'h40A00000, // 5
      32'h40400000, // 3
      32'h40E00000, // 7
      32'h40C00000, // 6
      32'h40E00000
    );

    $display("=== ALL TESTS PASS ===");
    #20;
    $finish;
  end

endmodule

`default_nettype wire
