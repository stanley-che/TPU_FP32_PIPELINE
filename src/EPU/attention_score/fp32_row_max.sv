`timescale 1ns/1ps
`default_nettype none

module fp32_row_max #(
  parameter int unsigned T = 4,
  localparam int unsigned CNT_W = (T<=1)?1:$clog2(T)
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,
  input  logic        row_start,   // 開始新 row
  input  logic        row_last,    // 這筆是 row 最後一個

  output logic        max_valid,
  output logic [31:0] max_fp32
);

  // fp32 ordering key (works for normal numbers, also keeps -inf very small)
  function automatic logic [31:0] fp32_key(input logic [31:0] a);
    begin
      fp32_key = a[31] ? ~a : (a ^ 32'h8000_0000);
    end
  endfunction

  logic [31:0] cur_max;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cur_max   <= 32'h0000_0000;
      max_valid <= 1'b0;
      max_fp32  <= 32'h0000_0000;
    end else begin
      max_valid <= 1'b0;

      if (in_valid) begin
        if (row_start) begin
          // row 第一筆直接當 max
          cur_max <= in_fp32;
        end else begin
          // 更新 max
          if (fp32_key(in_fp32) > fp32_key(cur_max))
            cur_max <= in_fp32;
        end

        if (row_last) begin
          max_fp32  <= row_start ? in_fp32 : (fp32_key(in_fp32) > fp32_key(cur_max) ? in_fp32 : cur_max);
          max_valid <= 1'b1;
        end
      end
    end
  end

endmodule

`default_nettype wire
