// Floating-point multiplier driver using Booth encoding and CSA
// Wraps around fp_multiplier_booth_csa module
// Provides simple start/done handshake interface
//parameterized for 32-bit single-precision FP

`include "./src/EPU/attention_score/fp_multiplier_booth_csa.sv"
`timescale 1ns/1ps

module fp_mul_driver (
  input  logic        clk,
  input  logic        rst,        // active-high reset (sync)

  // simple request interface
  input  logic        start,      // pulse or level; sampled when idle
  input  logic [31:0] a_bits,
  input  logic [31:0] b_bits,

  // response
  output logic        busy,
  output logic        done,       // LEVEL: stays 1 until next start
  output logic [31:0] z_bits
);

  // -----------------------------
  // Wires to DUT (handshake)
  // -----------------------------
  logic [31:0] dut_input_a, dut_input_b;
  logic        dut_input_a_stb, dut_input_b_stb;

  wire         dut_output_z_ack;      // ★ continuous assign → use wire
  wire  [31:0] dut_output_z;
  wire         dut_output_z_stb;
  wire         dut_input_a_ack;
  wire         dut_input_b_ack;

  // ★ Always ready to accept output (avoid deadlock)
  assign dut_output_z_ack = 1'b1;

  // Instantiate your FP multiplier
  fp_multiplier_booth_csa dut (
    .input_a(dut_input_a),
    .input_b(dut_input_b),
    .input_a_stb(dut_input_a_stb),
    .input_b_stb(dut_input_b_stb),
    .output_z_ack(dut_output_z_ack),
    .clk(clk),
    .rst(rst),
    .output_z(dut_output_z),
    .output_z_stb(dut_output_z_stb),
    .input_a_ack(dut_input_a_ack),
    .input_b_ack(dut_input_b_ack)
  );

  // -----------------------------
  // Driver FSM
  // -----------------------------
  typedef enum logic [1:0] {
    IDLE   = 2'd0,
    SEND_A = 2'd1,
    SEND_B = 2'd2,
    WAIT_Z = 2'd3
  } state_t;

  state_t state;

  // latch request operands when accepted
  logic [31:0] a_lat, b_lat;

  // outputs
  always_comb begin
    busy = (state != IDLE);
  end

  // sequential (sync reset as your comment says)
  always_ff @(posedge clk) begin
    if (rst) begin
      state           <= IDLE;
      a_lat           <= 32'd0;
      b_lat           <= 32'd0;

      dut_input_a     <= 32'd0;
      dut_input_b     <= 32'd0;
      dut_input_a_stb <= 1'b0;
      dut_input_b_stb <= 1'b0;

      z_bits          <= 32'd0;
      done            <= 1'b0;
    end else begin

      case (state)
        IDLE: begin
          dut_input_a_stb <= 1'b0;
          dut_input_b_stb <= 1'b0;

          // ★ clear done only when a new start is accepted
          if (start) begin
            done  <= 1'b0;

            // capture operands
            a_lat <= a_bits;
            b_lat <= b_bits;

            // drive buses
            dut_input_a <= a_bits;
            dut_input_b <= b_bits;

            dut_input_a_stb <= 1'b1;
            state <= SEND_A;
          end
        end

        SEND_A: begin
          dut_input_a <= a_lat;
          if (dut_input_a_ack && dut_input_a_stb) begin
            dut_input_a_stb <= 1'b0;
            dut_input_b_stb <= 1'b1;
            state <= SEND_B;
          end
        end

        SEND_B: begin
          dut_input_b <= b_lat;
          if (dut_input_b_ack && dut_input_b_stb) begin
            dut_input_b_stb <= 1'b0;
            state <= WAIT_Z;
          end
        end

        WAIT_Z: begin
          if (dut_output_z_stb) begin
            z_bits <= dut_output_z;

            // ★ done becomes LEVEL (sticky)
            done   <= 1'b1;

            // back to idle, ready for next start
            state  <= IDLE;
          end
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule
