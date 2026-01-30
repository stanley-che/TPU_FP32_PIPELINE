`include "adder.sv"
`timescale 1ns/1ps
module subber(
  input  [31:0] input_a,
  input  [31:0] input_b,
  input         input_a_stb,
  input         input_b_stb,
  input         output_z_ack,
  input         clk,
  input         rst,
  output [31:0] output_z,
  output        output_z_stb,
  output        input_a_ack,
  output        input_b_ack
);

  wire [31:0] b_neg = {~input_b[31], input_b[30:0]}; // -B

  adder u_add (
    .input_a(input_a),
    .input_b(b_neg),            // 丟 -B
    .input_a_stb(input_a_stb),
    .input_b_stb(input_b_stb),
    .output_z_ack(output_z_ack),
    .clk(clk),
    .rst(rst),
    .output_z(output_z),
    .output_z_stb(output_z_stb),
    .input_a_ack(input_a_ack),
    .input_b_ack(input_b_ack)
  );

endmodule
