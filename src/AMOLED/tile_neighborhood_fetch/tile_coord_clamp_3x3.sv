// ============================================================
// tile_coord_clamp_3x3.sv  (PIPELINED, IVERILOG-SAFE)
// - 1-deep pipeline register slice
// - center(i,j) -> 3x3 neighbors with clamp
// - Hold outputs stable while nb_valid=1 && nb_ready=0
// ============================================================

`ifndef TILE_COORD_CLAMP_3X3_SV
`define TILE_COORD_CLAMP_3X3_SV

`timescale 1ns/1ps
`default_nettype none

module tile_coord_clamp_3x3 #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,
  parameter int unsigned TAG_W   = 16
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // center request
  input  logic                         center_valid,
  output logic                         center_ready,
  input  logic [$clog2(TILES_Y)-1:0]    center_i,
  input  logic [$clog2(TILES_X)-1:0]    center_j,
  input  logic [TAG_W-1:0]             center_tag,

  // bundle out
  output logic                         nb_valid,
  input  logic                         nb_ready,

  output logic [$clog2(TILES_Y)-1:0]    nb_i0,
  output logic [$clog2(TILES_X)-1:0]    nb_j0,
  output logic [$clog2(TILES_Y)-1:0]    nb_i1,
  output logic [$clog2(TILES_X)-1:0]    nb_j1,
  output logic [$clog2(TILES_Y)-1:0]    nb_i2,
  output logic [$clog2(TILES_X)-1:0]    nb_j2,
  output logic [$clog2(TILES_Y)-1:0]    nb_i3,
  output logic [$clog2(TILES_X)-1:0]    nb_j3,
  output logic [$clog2(TILES_Y)-1:0]    nb_i4,
  output logic [$clog2(TILES_X)-1:0]    nb_j4,
  output logic [$clog2(TILES_Y)-1:0]    nb_i5,
  output logic [$clog2(TILES_X)-1:0]    nb_j5,
  output logic [$clog2(TILES_Y)-1:0]    nb_i6,
  output logic [$clog2(TILES_X)-1:0]    nb_j6,
  output logic [$clog2(TILES_Y)-1:0]    nb_i7,
  output logic [$clog2(TILES_X)-1:0]    nb_j7,
  output logic [$clog2(TILES_Y)-1:0]    nb_i8,
  output logic [$clog2(TILES_X)-1:0]    nb_j8,

  output logic [TAG_W-1:0]             nb_tag,
  output logic [8:0]                   nb_is_center
);

  // ----------------------------
  // control
  // ----------------------------
  wire go = en && !rst;

  logic full;
  wire  out_fire = nb_valid && nb_ready;
  wire  in_fire  = center_valid && center_ready;

  always @* begin
    if (!go) center_ready = 1'b0;
    else     center_ready = (!full) || out_fire;
  end

  always @* begin
    if (!go) nb_valid = 1'b0;
    else     nb_valid = full;
  end

  assign nb_is_center = 9'b000010000;

  // ----------------------------
  // clamp helper
  // ----------------------------
  localparam integer I_MAX = (TILES_Y > 0) ? (TILES_Y-1) : 0;
  localparam integer J_MAX = (TILES_X > 0) ? (TILES_X-1) : 0;

  function integer clamp_i;
    input integer v;
    begin
      if (v < 0)      clamp_i = 0;
      else if (v > I_MAX) clamp_i = I_MAX;
      else           clamp_i = v;
    end
  endfunction

  function integer clamp_j;
    input integer v;
    begin
      if (v < 0)      clamp_j = 0;
      else if (v > J_MAX) clamp_j = J_MAX;
      else           clamp_j = v;
    end
  endfunction

  // ----------------------------
  // combinational compute (latched on in_fire)
  // ----------------------------
  integer ci, cj;
  integer i0,i1,i2,i3,i4,i5,i6,i7,i8;
  integer j0,j1,j2,j3,j4,j5,j6,j7,j8;

  always @* begin
    // cast center to integer safely (non-negative)
    ci = center_i;
    cj = center_j;

    i0 = clamp_i(ci-1); j0 = clamp_j(cj-1);
    i1 = clamp_i(ci-1); j1 = clamp_j(cj  );
    i2 = clamp_i(ci-1); j2 = clamp_j(cj+1);

    i3 = clamp_i(ci  ); j3 = clamp_j(cj-1);
    i4 = clamp_i(ci  ); j4 = clamp_j(cj  );
    i5 = clamp_i(ci  ); j5 = clamp_j(cj+1);

    i6 = clamp_i(ci+1); j6 = clamp_j(cj-1);
    i7 = clamp_i(ci+1); j7 = clamp_j(cj  );
    i8 = clamp_i(ci+1); j8 = clamp_j(cj+1);
  end

  // ----------------------------
  // pipeline registers
  // ----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      full   <= 1'b0;
      nb_tag <= '0;

      nb_i0 <= '0; nb_j0 <= '0;
      nb_i1 <= '0; nb_j1 <= '0;
      nb_i2 <= '0; nb_j2 <= '0;
      nb_i3 <= '0; nb_j3 <= '0;
      nb_i4 <= '0; nb_j4 <= '0;
      nb_i5 <= '0; nb_j5 <= '0;
      nb_i6 <= '0; nb_j6 <= '0;
      nb_i7 <= '0; nb_j7 <= '0;
      nb_i8 <= '0; nb_j8 <= '0;
    end else if (en) begin
      if (out_fire) begin
        full <= 1'b0;
      end

      if (in_fire) begin
        full   <= 1'b1;
        nb_tag <= center_tag;

        nb_i0 <= i0[$clog2(TILES_Y)-1:0];
        nb_j0 <= j0[$clog2(TILES_X)-1:0];
        nb_i1 <= i1[$clog2(TILES_Y)-1:0];
        nb_j1 <= j1[$clog2(TILES_X)-1:0];
        nb_i2 <= i2[$clog2(TILES_Y)-1:0];
        nb_j2 <= j2[$clog2(TILES_X)-1:0];
        nb_i3 <= i3[$clog2(TILES_Y)-1:0];
        nb_j3 <= j3[$clog2(TILES_X)-1:0];
        nb_i4 <= i4[$clog2(TILES_Y)-1:0];
        nb_j4 <= j4[$clog2(TILES_X)-1:0];
        nb_i5 <= i5[$clog2(TILES_Y)-1:0];
        nb_j5 <= j5[$clog2(TILES_X)-1:0];
        nb_i6 <= i6[$clog2(TILES_Y)-1:0];
        nb_j6 <= j6[$clog2(TILES_X)-1:0];
        nb_i7 <= i7[$clog2(TILES_Y)-1:0];
        nb_j7 <= j7[$clog2(TILES_X)-1:0];
        nb_i8 <= i8[$clog2(TILES_Y)-1:0];
        nb_j8 <= j8[$clog2(TILES_X)-1:0];
      end
    end
  end

endmodule

`default_nettype wire
`endif
