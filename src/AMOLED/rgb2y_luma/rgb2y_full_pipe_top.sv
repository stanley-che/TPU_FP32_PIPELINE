// rgb2y_full_pipe_top.sv
// One-module integration:
// rgb_in -> unpack/clip (stallable) -> rgb_coeff_mult (stallable) -> rgb_y_pipe_top (ready/valid stall)
//
// Handshake rule:
// - in_ready comes from the END of pipe (rgb_y_pipe_top), and we use it as en_go for ALL stages
// - Upstream should only assert pix_valid when in_ready=1 (or you can drive pix_valid anyway; this top will only advance on in_ready)

`ifndef RGB2Y_FULL_PIPE_TOP_SV
`define RGB2Y_FULL_PIPE_TOP_SV

`include "./src/AMOLED/rgb2y_luma/rgb_coeff_mult.sv"
`include "./src/AMOLED/rgb2y_luma/rgb_y_pipe_top.sv"

`timescale 1ns/1ps
`default_nettype none

module rgb2y_full_pipe_top #(
  // ---- unpack/clip ----
  parameter int unsigned FORMAT = 0,      // 0 RGB888, 1 RGB101010, 2 RGB121212, 3 RGB565
  parameter int unsigned LAT_U  = 1,
  parameter bit          ZERO_WHEN_INVALID = 1'b1,

  // ---- coeff mult ----
  parameter int unsigned LAT_M  = 1,
  parameter bit          USE_DSP = 1'b0,
  parameter bit          BYPASS  = 1'b0,

  // ---- sum + round pipe ----
  parameter int unsigned LAT_SUM = 1,
  parameter bit          HOLD_VALID_WHEN_STALL = 1'b1,

  parameter int unsigned LAT_RND = 1,
  parameter int unsigned ROUND_MODE = 1,
  parameter int unsigned Y8_MODE = 0
)(
  input  logic        clk,
  input  logic        rst,

  // global enable
  input  logic        en,

  // input pixel stream
  input  logic        pix_valid,
  input  logic [35:0] rgb_in,

  // downstream backpressure
  input  logic        out_ready,
  output logic        in_ready,

  // outputs (final)
  output logic        v4_valid,
  output logic [16:0] sum17,
  output logic [9:0]  y10,
  output logic [7:0]  y8,
  output logic        ovf_sum,
  output logic        ovf_y,

  // optional taps (debug)
  output logic        v1_valid,
  output logic [7:0]  r8,
  output logic [7:0]  g8,
  output logic [7:0]  b8,

  output logic        v2_valid,
  output logic [15:0] r_term,
  output logic [15:0] g_term,
  output logic [15:0] b_term
);

  // ============================================================
  // end-of-pipe ready determines whole-pipe advance
  // ============================================================
  logic en_go;          // single advance pulse for ALL stages

  // ============================================================
  // Stage0 unpack combinational (from current rgb_in)
  // ============================================================
  function automatic [7:0] exp5_to_8(input [4:0] x);
    begin exp5_to_8 = {x, x[4:2]}; end
  endfunction
  function automatic [7:0] exp6_to_8(input [5:0] x);
    begin exp6_to_8 = {x, x[5:4]}; end
  endfunction
  function automatic [7:0] msb10_to_8(input [9:0] x);
    begin msb10_to_8 = x[9:2]; end
  endfunction
  function automatic [7:0] msb12_to_8(input [11:0] x);
    begin msb12_to_8 = x[11:4]; end
  endfunction

  logic        u0_v;
  logic [7:0]  u0_r, u0_g, u0_b;

  always @* begin
    logic [7:0] rc, gc, bc;
    rc = 8'h00; gc = 8'h00; bc = 8'h00;

    if (FORMAT == 0) begin
      rc = rgb_in[23:16];
      gc = rgb_in[15:8];
      bc = rgb_in[7:0];
    end else if (FORMAT == 1) begin
      rc = msb10_to_8(rgb_in[29:20]);
      gc = msb10_to_8(rgb_in[19:10]);
      bc = msb10_to_8(rgb_in[9:0]);
    end else if (FORMAT == 2) begin
      rc = msb12_to_8(rgb_in[35:24]);
      gc = msb12_to_8(rgb_in[23:12]);
      bc = msb12_to_8(rgb_in[11:0]);
    end else begin
      rc = exp5_to_8(rgb_in[15:11]);
      gc = exp6_to_8(rgb_in[10:5]);
      bc = exp5_to_8(rgb_in[4:0]);
    end

    u0_v = pix_valid;

    if (ZERO_WHEN_INVALID && !pix_valid) begin
      u0_r = 8'h00; u0_g = 8'h00; u0_b = 8'h00;
    end else begin
      u0_r = rc;    u0_g = gc;    u0_b = bc;
    end
  end

  // ============================================================
  // Stage1 unpack pipeline (STALLABLE) : advances only on en_go
  // ============================================================
  logic [LAT_U:0] u_vsh;
  logic [7:0]     u_rsh [0:LAT_U];
  logic [7:0]     u_gsh [0:LAT_U];
  logic [7:0]     u_bsh [0:LAT_U];

  integer t;

  always @(posedge clk) begin
    if (rst) begin
      u_vsh <= '0;
      for (t=0; t<=LAT_U; t=t+1) begin
        u_rsh[t] <= 8'h00;
        u_gsh[t] <= 8'h00;
        u_bsh[t] <= 8'h00;
      end
    end else if (en_go) begin
      for (t=LAT_U; t>=1; t=t-1) begin
        u_vsh[t] <= u_vsh[t-1];
        u_rsh[t] <= u_rsh[t-1];
        u_gsh[t] <= u_gsh[t-1];
        u_bsh[t] <= u_bsh[t-1];
      end
      u_vsh[0] <= u0_v;
      u_rsh[0] <= u0_r;
      u_gsh[0] <= u0_g;
      u_bsh[0] <= u0_b;
    end
  end

  // expose unpack tail
  assign v1_valid = u_vsh[LAT_U];
  assign r8       = u_rsh[LAT_U];
  assign g8       = u_gsh[LAT_U];
  assign b8       = u_bsh[LAT_U];


  // ============================================================
  // Stage2 coeff mult (already stallable via .en)
  // ============================================================
  rgb_coeff_mult #(
    .LAT(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS)
  ) u_mult (
    .clk(clk),
    .rst(rst),
    .en(en_go),
    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8),
    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term)
  );

  // ============================================================
  // Stage3+4 sum + round/shift pipe (ready/valid end-of-pipe)
  // ============================================================
  rgb_y_pipe_top #(
    .LAT_SUM(LAT_SUM),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),
    .LAT_RND(LAT_RND),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE)
  ) u_y (
    .clk(clk),
    .rst(rst),
    .en(en),
    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term),
    .out_ready(out_ready),
    .in_ready(in_ready),
    .v4_valid(v4_valid),
    .sum17(sum17),
    .y10(y10),
    .y8(y8),
    .ovf_sum(ovf_sum),
    .ovf_y(ovf_y)
  );

  // whole-pipe advance pulse
  always @* begin
    // NOTE: rgb_y_pipe_top's in_ready already includes (en && can_accept)
    en_go = in_ready;
  end

endmodule

`default_nettype wire
`endif
