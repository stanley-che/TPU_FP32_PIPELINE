`ifndef FP_ADDER_DRIVER_BA_SV
`define FP_ADDER_DRIVER_BA_SV
// Floating-point adder driver
// Wraps around adder module
// Provides simple start/done handshake interface
// parameterized for 32-bit single-precision FP
`include "./src/EPU/attention_score/adder_ba.sv"
`timescale 1ns/1ps

`default_nettype none

// ============================================================
// fp_adder_driver (PIPELINED CORE, 1 inflight)
// ============================================================
module fp_adder_driver_ba #(
  parameter integer LAT = 6
)(
  input              clk,
  input              rst,        // active-high sync reset

  input              start,
  input      [31:0]  a_bits,
  input      [31:0]  b_bits,

  output             busy,
  output reg         done,       // sticky
  output reg [31:0]  z_bits
);

  wire in_fire;
  reg  running;

  assign busy    = running;
  assign in_fire = start & ~running;

  wire        core_out_valid;
  wire [31:0] core_z;

  fp32_add_pipe_fixed #(.LAT(LAT)) u_core (
    .clk       (clk),
    .rst_n     (~rst),
    .in_valid  (in_fire),
    .a_bits    (a_bits),
    .b_bits    (b_bits),
    .out_valid (core_out_valid),
    .z_bits    (core_z)
  );

  always @(posedge clk) begin
    if (rst) begin
      running <= 1'b0;
      done    <= 1'b0;
      z_bits  <= 32'd0;
    end else begin
      if (in_fire) begin
        running <= 1'b1;
        done    <= 1'b0;
      end
      if (core_out_valid) begin
        z_bits  <= core_z;
        done    <= 1'b1;
        running <= 1'b0;
      end
    end
  end
endmodule

// ============================================================
// fp32_add_pipe_fixed (iverilog-safe)
// ============================================================

`endif

