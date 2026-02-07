`ifndef PE_SV
`define PE_SV
// pe.sv — IEEE-754 FP32 processing element using dawsonjon/fpu
`include "./src/EPU/attention_score/fp_adder_driver.sv"
`include "./src/EPU/attention_score/fp_mul_driver.sv"
`timescale 1ns/1ps

module pe (
  input  logic        clk,
  input  logic        rst,
  input  logic        in_valid,
  input  logic [31:0] a_bits,
  input  logic [31:0] b_bits,
  input  logic [31:0] psum_in,
  output logic        out_valid,
  output logic [31:0] psum_out
);

  typedef enum logic [2:0] {IDLE, START_MUL, WAIT_MUL, START_ADD, WAIT_ADD} st_t;
  st_t st;

  logic [31:0] a_r, b_r;
  logic [31:0] mul_r;
  logic [31:0] psum_r;

  logic mul_start, add_start;

  logic [31:0] mul_out;
  logic        mul_done, mul_busy;

  logic [31:0] add_out;
  logic        add_done, add_busy;

  // start pulses ONLY when busy=0 and we're in START_* states
  assign mul_start = (st == START_MUL) && !mul_busy;
  assign add_start = (st == START_ADD) && !add_busy;

  fp_mul_driver fp_mul_inst (
    .clk(clk), .rst(rst),
    .start(mul_start),
    .a_bits(a_r),
    .b_bits(b_r),
    .busy(mul_busy),
    .done(mul_done),
    .z_bits(mul_out)
  );

  fp_adder_driver fp_add_inst (
    .clk(clk), .rst(rst),
    .start(add_start),
    .a_bits(mul_r),
    .b_bits(psum_r),
    .busy(add_busy),
    .done(add_done),
    .z_bits(add_out)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      st        <= IDLE;
      a_r       <= 32'd0;
      b_r       <= 32'd0;
      mul_r     <= 32'd0;
      psum_r    <= 32'd0;
      psum_out  <= 32'd0;
      out_valid <= 1'b0;
    end else begin
      out_valid <= 1'b0;

      case (st)
        IDLE: begin
          if (in_valid) begin
            // 先鎖資料
            a_r    <= a_bits;
            b_r    <= b_bits;
            psum_r <= psum_in;
            st     <= START_MUL;
          end
        end

        START_MUL: begin
          // 這一拍 mul_start 會被拉高（若 !mul_busy）
          if (!mul_busy) st <= WAIT_MUL;
        end

        WAIT_MUL: begin
          if (mul_done) begin
            // 先鎖乘法結果
            mul_r <= mul_out;
            st    <= START_ADD;
          end
        end

        START_ADD: begin
          // 這一拍 add_start 會被拉高（若 !add_busy）
          if (!add_busy) st <= WAIT_ADD;
        end

        WAIT_ADD: begin
          if (add_done) begin
            psum_out  <= add_out;
            out_valid <= 1'b1;
            st        <= IDLE;
          end
        end

        default: st <= IDLE;
      endcase
    end
  end

endmodule
`endif