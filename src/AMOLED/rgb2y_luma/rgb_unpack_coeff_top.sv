// rgb_unpack_coeff_top.sv
// rgb_in -> rgb_unpack_clip -> rgb_coeff_mult
`ifndef RGB_UNPACK_COEFF_TOP_SV
`define RGB_UNPACK_COEFF_TOP_SV
`include "./src/AMOLED/rgb2y_luma/rgb_unpack_clip.sv"
`include "./src/AMOLED/rgb2y_luma/rgb_coeff_mult.sv"
`timescale 1ns/1ps
`default_nettype none


module rgb_unpack_coeff_top #(
  parameter int unsigned FORMAT = 0,
  parameter int unsigned LAT_U  = 1,    // unpack latency
  parameter bit          ZERO_WHEN_INVALID = 1'b1,

  parameter int unsigned LAT_M  = 1,    // mult latency
  parameter bit          USE_DSP = 1'b0,
  parameter bit          BYPASS  = 1'b0
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        en,        // for multiplier pipeline
  input  logic        pix_valid,
  input  logic [35:0] rgb_in,

  output logic        v2_valid,
  output logic [15:0] r_term,
  output logic [15:0] g_term,
  output logic [15:0] b_term,

  // optional tap-out (debug)
  output logic        v1_valid,
  output logic [7:0]  r8,
  output logic [7:0]  g8,
  output logic [7:0]  b8
);

  rgb_unpack_clip #(
    .FORMAT(FORMAT),
    .LAT(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID)
  ) u_unpack (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .rgb_in(rgb_in),
    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8)
  );

  rgb_coeff_mult #(
    .LAT(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS)
  ) u_mult (
    .clk(clk),
    .rst(rst),
    .en(en),
    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8),
    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term)
  );

endmodule

`default_nettype wire
`endif
