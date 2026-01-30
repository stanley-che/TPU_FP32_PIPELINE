// linebuf_y_1line.sv
// - 1-line buffer: store previous-line Y for each x (depth = ACTIVE_W)
// - provides y_left (same-line left pixel) with x==0 clamp to y_cur
// - read-before-write: y_up <= mem[x] (old), then mem[x] <= y_cur
//
// Added:
// - ready/valid backpressure
//   in : v1_valid, v1_ready, x, y_in
//   out: v2_valid, v2_ready, y_cur, y_left, y_up
//
// Behavior:
// - Accept input only when v1_valid && v1_ready
// - Output registers hold when v2_valid && !v2_ready
// - mem/prev_y_in_line update ONLY on accept (so no corruption under stall)
// - en=0 freezes internal state and deasserts v2_valid

`ifndef LINEBUF_Y_1LINE_SV
`define LINEBUF_Y_1LINE_SV

`timescale 1ns/1ps
`default_nettype none

module linebuf_y_1line #(
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 en,

  // in
  input  logic                 v1_valid,
  output logic                 v1_ready,
  input  logic [X_W-1:0]        x,
  input  logic [Y_W-1:0]        y_in,

  // out
  output logic                 v2_valid,
  input  logic                 v2_ready,
  output logic [Y_W-1:0]        y_cur,
  output logic [Y_W-1:0]        y_left,
  output logic [Y_W-1:0]        y_up
);

  // ------------------------------------------------------------
  // 1-line memory: previous line's y at each x
  // ------------------------------------------------------------
  logic [Y_W-1:0] mem [0:ACTIVE_W-1];

  // same-line left pixel tracker (state)
  logic [Y_W-1:0] prev_y_in_line;

  // hold accepted x to keep output alignment under stall
  logic [X_W-1:0] x_hold;

  // ------------------------------------------------------------
  // Address clamp function (pure combinational)
  // ------------------------------------------------------------
  function automatic [X_W-1:0] clamp_x(input [X_W-1:0] xin);
    if (xin < ACTIVE_W[X_W-1:0]) clamp_x = xin;
    else clamp_x = ACTIVE_W[X_W-1:0] - {{(X_W-1){1'b0}},1'b1};
  endfunction

  logic [X_W-1:0] x_addr_in;
  always @* x_addr_in = clamp_x(x);

  // ------------------------------------------------------------
  // Ready/valid control
  // One-stage output reg acts like pipeline register with backpressure
  // ------------------------------------------------------------
  logic accept;   // accept one input beat into output regs

  always @* begin
    // can accept new data when:
    // - enabled AND (output empty OR downstream ready to take current)
    v1_ready = en && (!v2_valid || v2_ready);
    accept   = v1_valid && v1_ready;
  end

  integer i;

  always_ff @(posedge clk) begin
    if (rst) begin
      v2_valid       <= 1'b0;
      y_cur          <= '0;
      y_left         <= '0;
      y_up           <= '0;
      prev_y_in_line <= '0;
      x_hold         <= '0;
      for (i = 0; i < ACTIVE_W; i = i + 1) begin
        mem[i] <= '0;
      end
    end else if (!en) begin
      // freeze internal state; downstream should ignore
      v2_valid <= 1'b0;
      // keep other regs as-is (frozen)
    end else begin
      // if output has valid data and downstream accepted it this cycle,
      // v2_valid may clear unless we also accept new input (accept will set it)
      if (v2_valid && v2_ready && !accept) begin
        v2_valid <= 1'b0;
      end

      // load new beat if accepted
      if (accept) begin
        // mark output valid
        v2_valid <= 1'b1;

        // latch x for consistent output alignment under stall
        x_hold <= x_addr_in;

        // current pixel
        y_cur <= y_in;

        // y_left: clamp at x==0 to y_cur (use accepted x)
        if (x_addr_in == {X_W{1'b0}}) y_left <= y_in;
        else                          y_left <= prev_y_in_line;

        // y_up: read previous line BEFORE writing current pixel
        y_up <= mem[x_addr_in];

        // update memory with current pixel for next line
        mem[x_addr_in] <= y_in;

        // update prev pixel in this line
        prev_y_in_line <= y_in;
      end
      // else: no accept => either idle or stalled, hold y_* and state
    end
  end

endmodule

`default_nettype wire
`endif
