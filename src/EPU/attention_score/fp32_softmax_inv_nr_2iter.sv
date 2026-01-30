// ============================================================================
// fp32_softmax_inv_nr_2iter.sv
// FP32 reciprocal using LUT + 2x Newton-Raphson
// y1 = y0 * (2 - x*y0)
// y2 = y1 * (2 - x*y1)
// ============================================================================

`include "./src/EPU/attention_score/fp_mul_driver.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp32_softmax_inv_lut.sv"

`timescale 1ns/1ps
`default_nettype none


module fp32_softmax_inv_nr_2iter #(
  parameter int unsigned M_BITS = 6
  
)(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        in_valid,
  input  logic [31:0] in_fp32,
  output logic        out_valid,
  output logic [31:0] out_fp32
);

  localparam logic [31:0] FP32_TWO = 32'h4000_0000;
  localparam int unsigned N_ITER = 5;
  // align LUT 1-cycle
  logic [31:0] x_d1;
  always_ff @(posedge clk) begin
    if (!rst_n) x_d1 <= 32'h0;
    else if (in_valid) x_d1 <= in_fp32;
  end

  // LUT
  logic lut_v;
  logic [31:0] y0;
  fp32_softmax_inv_lut #(.M_BITS(M_BITS)) u_lut (
    .clk(clk), .rst_n(rst_n),
    .in_valid(in_valid), .in_fp32(in_fp32),
    .out_valid(lut_v), .out_fp32(y0)
  );

  // mul/add drivers
  logic        mul_start, mul_done, mul_busy;
  logic [31:0] mul_a, mul_b, mul_z;

  fp_mul_driver u_mul (
    .clk(clk), .rst(~rst_n),
    .start(mul_start), .a_bits(mul_a), .b_bits(mul_b),
    .busy(mul_busy), .done(mul_done), .z_bits(mul_z)
  );

  logic        add_start, add_done, add_busy;
  logic [31:0] add_a, add_b, add_z;

  fp_adder_driver_ba u_add (
    .clk(clk), .rst(~rst_n),
    .start(add_start), .a_bits(add_a), .b_bits(add_b),
    .busy(add_busy), .done(add_done), .z_bits(add_z)
  );

  // done pulse
  logic mul_done_q, add_done_q;
  wire mul_done_pulse = mul_done & ~mul_done_q;
  wire add_done_pulse = add_done & ~add_done_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin mul_done_q <= 0; add_done_q <= 0; end
    else begin mul_done_q <= mul_done; add_done_q <= add_done; end
  end

  function automatic logic [31:0] fp32_neg(input logic [31:0] a);
    fp32_neg = {~a[31], a[30:0]};
  endfunction

  // FSM
  typedef enum logic [2:0] {S_IDLE, S_MUL_P, S_WAIT_P, S_SUB_T, S_WAIT_T, S_MUL_Y, S_WAIT_Y, S_DONE} st_t;
  st_t st;

  logic [31:0] x_hold, y_hold;   // y_hold is current y
  logic [31:0] p_hold, t_hold;
  logic [$clog2(N_ITER+1)-1:0] iter; // counts 0..N_ITER

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE;
      out_valid <= 1'b0;
      out_fp32  <= 32'h0;
      mul_start <= 1'b0;
      add_start <= 1'b0;
      mul_a <= 0; mul_b <= 0;
      add_a <= 0; add_b <= 0;
      x_hold <= 0; y_hold <= 0;
      p_hold <= 0; t_hold <= 0;
      iter <= '0;
    end else begin
      out_valid <= 1'b0;
      mul_start <= 1'b0;
      add_start <= 1'b0;

      case (st)
        S_IDLE: begin
          if (lut_v) begin
            x_hold <= x_d1;
            y_hold <= y0;
            iter   <= '0;
            st     <= S_MUL_P;
          end
        end

        // p = x*y
        S_MUL_P: if (!mul_busy) begin
          mul_a <= x_hold;
          mul_b <= y_hold;
          mul_start <= 1'b1;
          st <= S_WAIT_P;
        end

        S_WAIT_P: if (mul_done_pulse) begin
          p_hold <= mul_z;
          st <= S_SUB_T;
        end

        // t = 2 - p
        S_SUB_T: if (!add_busy) begin
          add_a <= FP32_TWO;
          add_b <= fp32_neg(p_hold);
          add_start <= 1'b1;
          st <= S_WAIT_T;
        end

        S_WAIT_T: if (add_done_pulse) begin
          t_hold <= add_z;
          st <= S_MUL_Y;
        end

        // y = y*t
        S_MUL_Y: if (!mul_busy) begin
          mul_a <= y_hold;
          mul_b <= t_hold;
          mul_start <= 1'b1;
          st <= S_WAIT_Y;
        end

        S_WAIT_Y: if (mul_done_pulse) begin
          y_hold <= mul_z;
          iter <= iter + 1'b1;

          if (iter + 1'b1 == N_ITER[$bits(iter)-1:0]) begin
            out_fp32  <= mul_z;
            out_valid <= 1'b1;
            st <= S_IDLE;
          end else begin
            st <= S_MUL_P; // next iteration (for-loop)
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
