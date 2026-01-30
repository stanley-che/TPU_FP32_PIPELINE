`ifndef EDGE_PULSE_GEN_SV
`define EDGE_PULSE_GEN_SV

`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// edge_pulse_gen
// Purpose:
//   Generate 1-cycle edge pulses (rise/fall) for clean
//   vsync/hsync/de signals (already synchronized).
//
// In : clk, rst, vsync, hsync, de
// Out: vs_rise, vs_fall, hs_rise, hs_fall, de_rise, de_fall
//
// Implementation:
//   - Register previous-cycle values: *_d
//   - rise = sig & ~sig_d
//   - fall = ~sig & sig_d
// ------------------------------------------------------------

module edge_pulse_gen (
  input  logic clk,
  input  logic rst,

  input  logic vsync,
  input  logic hsync,
  input  logic de,

  output logic vs_rise,
  output logic vs_fall,
  output logic hs_rise,
  output logic hs_fall,
  output logic de_rise,
  output logic de_fall
);

  logic vs_d, hs_d, de_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_d    <= 1'b0;
      hs_d    <= 1'b0;
      de_d    <= 1'b0;

      vs_rise <= 1'b0;
      vs_fall <= 1'b0;
      hs_rise <= 1'b0;
      hs_fall <= 1'b0;
      de_rise <= 1'b0;
      de_fall <= 1'b0;
    end else begin
      // 1-cycle pulses (computed from OLD *_d because of NBA semantics)
      vs_rise <=  vsync & ~vs_d;
      vs_fall <= ~vsync &  vs_d;

      hs_rise <=  hsync & ~hs_d;
      hs_fall <= ~hsync &  hs_d;

      de_rise <=  de    & ~de_d;
      de_fall <= ~de    &  de_d;

      // update previous value
      vs_d <= vsync;
      hs_d <= hsync;
      de_d <= de;
    end
  end

endmodule
`default_nettype wire
`endif
