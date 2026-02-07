// ============================================================================
// fp32_softmax_inv_nr.sv
// Reciprocal using LUT initial guess + 1x Newton-Raphson
// y1 = y0 * (2 - x*y0)
// Uses: fp_mul_driver + fp_adder_driver_ba
// ============================================================================
`include "./src/EPU/attention_score/fp_mul_driver.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp32_softmax_inv_lut.sv" // your existing LUT (as y0)
`timescale 1ns/1ps
`default_nettype none

`timescale 1ns/1ps
`default_nettype none

module fp32_softmax_inv_nr #(
  parameter int unsigned M_BITS = 6
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] in_fp32,

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  localparam logic [31:0] FP32_TWO = 32'h4000_0000; // 2.0

  // -------------------------
  // 0) pipeline input so x aligns with lut_v (lut is 1-cycle)
  // -------------------------
  logic [31:0] x_d1;
  always_ff @(posedge clk) begin
    if (!rst_n) x_d1 <= 32'h0;
    else if (in_valid) x_d1 <= in_fp32;
  end

  // -------------------------
  // 1) LUT initial guess y0
  // -------------------------
  logic        lut_v;
  logic [31:0] y0;

  fp32_softmax_inv_lut #(.M_BITS(M_BITS)) u_lut (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (in_valid),
    .in_fp32  (in_fp32),
    .out_valid(lut_v),
    .out_fp32 (y0)
  );

  // -------------------------
  // FP MUL driver
  // -------------------------
  logic        mul_start, mul_done, mul_busy;
  logic [31:0] mul_a, mul_b, mul_z;

  fp_mul_driver u_mul (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (mul_start),
    .a_bits (mul_a),
    .b_bits (mul_b),
    .busy   (mul_busy),
    .done   (mul_done),
    .z_bits (mul_z)
  );

  // -------------------------
  // FP ADD driver (for 2 - p)
  // -------------------------
  logic        add_start, add_done, add_busy;
  logic [31:0] add_a, add_b, add_z;

  fp_adder_driver_ba u_add (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (add_start),
    .a_bits (add_a),
    .b_bits (add_b),
    .busy   (add_busy),
    .done   (add_done),
    .z_bits (add_z)
  );

  // -------------------------
  // FSM: p=x*y0, t=2-p, y1=y0*t
  // -------------------------
  typedef enum logic [2:0] {
    S_IDLE,
    S_MUL1, S_WAIT1,
    S_SUB2, S_WAIT2,
    S_MUL2, S_WAIT3
  } st_t;

  st_t st;

  logic [31:0] x_hold, y_hold;
  logic [31:0] p_hold, t_hold;

  // negate helper for FP32 sign bit
  function automatic logic [31:0] fp32_neg(input logic [31:0] a);
    fp32_neg = {~a[31], a[30:0]};
  endfunction
  // ===== DONE pulse detect =====
logic mul_done_q, add_done_q;
logic mul_done_pulse, add_done_pulse;

assign mul_done_pulse = mul_done & ~mul_done_q;
assign add_done_pulse = add_done & ~add_done_q;

always_ff @(posedge clk) begin
  if (!rst_n) begin
    mul_done_q <= 1'b0;
    add_done_q <= 1'b0;
  end else begin
    mul_done_q <= mul_done;
    add_done_q <= add_done;
  end
end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st        <= S_IDLE;
      out_valid <= 1'b0;
      out_fp32  <= 32'h0;

      x_hold    <= 32'h0;
      y_hold    <= 32'h0;
      p_hold    <= 32'h0;
      t_hold    <= 32'h0;

      mul_start <= 1'b0;
      add_start <= 1'b0;
      mul_a     <= 32'h0;
      mul_b     <= 32'h0;
      add_a     <= 32'h0;
      add_b     <= 32'h0;
    end else begin
      // defaults every cycle (IMPORTANT)
      out_valid <= 1'b0;
      mul_start <= 1'b0;
      add_start <= 1'b0;

      case (st)
        S_IDLE: begin
          if (lut_v) begin
            // x aligns with lut_v via x_d1
            x_hold <= x_d1;
            y_hold <= y0;
            st     <= S_MUL1;
          end
        end

        S_MUL1: begin
          // p = x*y0
          if (!mul_busy) begin
            mul_a     <= x_hold;
            mul_b     <= y_hold;
            mul_start <= 1'b1;      // 1-cycle pulse
            st        <= S_WAIT1;
          end
        end

        S_WAIT1: begin
          if (mul_done_pulse) begin
            p_hold <= mul_z;
            st     <= S_SUB2;
          end
        end

        S_SUB2: begin
          // t = 2 - p
          if (!add_busy) begin
            add_a     <= FP32_TWO;
            add_b     <= fp32_neg(p_hold);
            add_start <= 1'b1;
            st        <= S_WAIT2;
          end
        end

        S_WAIT2: begin
          // S_WAIT2
          if (add_done_pulse) begin
            t_hold <= add_z;
            st     <= S_MUL2;
          end
        end

        S_MUL2: begin
          // y1 = y0 * t
          if (!mul_busy) begin
            mul_a     <= y_hold;
            mul_b     <= t_hold;
            mul_start <= 1'b1;
            st        <= S_WAIT3;
          end
        end

        S_WAIT3: begin
          if (mul_done_pulse) begin
            out_fp32  <= mul_z;
            out_valid <= 1'b1;
            st        <= S_IDLE;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
