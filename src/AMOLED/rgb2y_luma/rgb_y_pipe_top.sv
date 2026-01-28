// rgb_y_pipe_top.sv
// Combine rgb_sum_acc + rgb_round_shift into a single streaming pipe
//
// v2_valid,r/g/b terms -> (sum17, ovf_sum) -> round/shift -> y10/y8, ovf_y
//
// Handshake:
// - out_ready is downstream ready
// - in_ready is upstream ready
// - When stalled, whole pipe holds state (no internal skew)

`ifndef RGB_Y_PIPE_TOP_SV
`define RGB_Y_PIPE_TOP_SV
`include "./src/AMOLED/rgb2y_luma/rgb_round_shift.sv"
`include "./src/AMOLED/rgb2y_luma/rgb_sum_acc.sv"
`timescale 1ns/1ps
`default_nettype none

module rgb_y_pipe_top #(
  // --- sum stage ---
  parameter int unsigned LAT_SUM = 1,
  parameter bit          ZERO_WHEN_INVALID = 1'b1,
  parameter bit          HOLD_VALID_WHEN_STALL = 1'b1,

  // --- round/shift stage ---
  parameter int unsigned LAT_RND = 1,
  parameter int unsigned ROUND_MODE = 1,
  parameter int unsigned Y8_MODE = 0
)(
  input  logic        clk,
  input  logic        rst,

  // global enable (pipeline advances only when en && can_accept)
  input  logic        en,

  // input stage (v2)
  input  logic        v2_valid,
  input  logic [15:0] r_term,
  input  logic [15:0] g_term,
  input  logic [15:0] b_term,

  // downstream backpressure
  input  logic        out_ready,
  output logic        in_ready,

  // outputs (v4)
  output logic        v4_valid,
  output logic [16:0] sum17,
  output logic [9:0]  y10,
  output logic [7:0]  y8,
  output logic        ovf_sum,
  output logic        ovf_y
);

  // ------------------------------------------------------------
  // internal handshake:
  // we want BOTH stages to stall together
  // so we drive both with the SAME out_ready/en gating.
  // ------------------------------------------------------------

  // Stage-2 wires (sum stage outputs = round stage inputs)
  logic        v3_valid;
  logic [16:0] sum_s;
  logic [7:0]  y_dummy;   // not used (we let round_shift generate y10/y8)
  logic        ovf_s;

  // in_ready comes from round_shift (end of pipe), because it knows can_accept
  // But rgb_sum_acc doesn't have ready/valid; it only has en hold.
  // So we enforce: only drive v2_valid when in_ready=1 at system level (TB or upstream).
  //
  // Practically: we can simply forward in_ready from round stage.
  // sum stage uses the same en, and will stall when en=0 (same as round stage stall).
  //
  // IMPORTANT: when round stage stalls due to out_ready=0, we must also stall sum stage.
  // We'll create a combined enable: en_go = en && (out_ready || !v4_valid_internal)
  //
  // But v4_valid is from round stage, so we use round's in_ready as the “go” indication.
  // in_ready = en && can_accept  (from round stage)
  // => use en_go = in_ready as the pipeline-advance pulse.
  //
  logic en_go;

  // round stage in_ready
  logic in_ready_r;

  // ------------------------------------------------------------
  // round/shift stage (end of pipe) – provides in_ready_r
  // ------------------------------------------------------------
  rgb_round_shift #(
    .LAT(LAT_RND),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE)
  ) u_rnd (
    .clk(clk),
    .rst(rst),
    .en(en_go),            // <<< advance only when whole pipe can accept
    .v3_valid(v3_valid),
    .sum(sum_s),
    .out_ready(out_ready),
    .in_ready(in_ready_r),
    .v4_valid(v4_valid),
    .y10(y10),
    .y8(y8),
    .ovf(ovf_y)
  );

  // propagate in_ready outward
  always @* begin
    in_ready = in_ready_r;
    en_go    = in_ready_r;   // because in_ready_r already includes en && can_accept
  end

  // ------------------------------------------------------------
  // sum stage
  // NOTE: we set SHIFT/ROUND/YW so it does NOT duplicate rounding.
  // It outputs only sum17 and ovf_sum; y_out is ignored.
  // ------------------------------------------------------------
  rgb_sum_acc #(
    .LAT(LAT_SUM),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),

    // disable "final y" generation in sum stage (avoid double rounding)
    .SHIFT(0),
    .ROUND(1'b0),
    .YW(8),
    .SATURATE(1'b0)
  ) u_sum (
    .clk(clk),
    .rst(rst),
    .en(en_go),            // <<< stall together with round stage
    .v2_valid(v2_valid),

    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term),

    .v3_valid(v3_valid),
    .sum(sum_s),
    .y_out(y_dummy),
    .ovf(ovf_s)
  );

  // expose debug outputs
  always @* begin
    sum17   = sum_s;
    ovf_sum = ovf_s;
  end

endmodule

`default_nettype wire
`endif
