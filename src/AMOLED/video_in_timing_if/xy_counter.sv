`ifndef XY_COUNTER_SV
`define XY_COUNTER_SV

`timescale 1ns/1ps
`default_nettype none

module xy_counter #(
  parameter int unsigned X_W = 11,
  parameter int unsigned Y_W = 10,

  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  parameter bit Y_INC_ON_DE_RISE = 1'b0,

  parameter int unsigned X_LIMIT_MODE = 2, // 0 none, 1 wrap, 2 sat
  parameter int unsigned Y_LIMIT_MODE = 2,

  parameter bit ENABLE_BOUNDS   = 1'b1,
  parameter bit GATE_BY_IN_FRAME= 1'b0,
  parameter bit ENABLE_ASSERT   = 1'b0
)(
  input  logic                 clk,
  input  logic                 rst,

  input  logic                 pix_valid,
  input  logic                 de_rise,
  input  logic                 de_fall,
  input  logic                 vs_rise,

  input  logic                 in_frame_i,

  output logic [X_W-1:0]        x,
  output logic [Y_W-1:0]        y,

  output logic                 frame_start_p,
  output logic                 line_start_p,
  output logic                 line_end_p,

  output logic                 x_at_last,
  output logic                 y_at_last,
  output logic                 x_overflow,
  output logic                 y_overflow
);

  // -----------------------------
  // constants (avoid tricky casts)
  // -----------------------------
  localparam logic [X_W-1:0] X_ONE  = {{(X_W-1){1'b0}}, 1'b1};
  localparam logic [Y_W-1:0] Y_ONE  = {{(Y_W-1){1'b0}}, 1'b1};

  localparam int unsigned X_LAST_INT = (ACTIVE_W > 0) ? (ACTIVE_W - 1) : 0;
  localparam int unsigned Y_LAST_INT = (ACTIVE_H > 0) ? (ACTIVE_H - 1) : 0;

  localparam logic [X_W-1:0] X_LAST = X_LAST_INT[X_W-1:0];
  localparam logic [Y_W-1:0] Y_LAST = Y_LAST_INT[Y_W-1:0];

  // -----------------------------
  // derived events
  // -----------------------------
  wire do_frame_start = vs_rise;
  wire do_line_start  = de_rise;
  wire do_line_end    = de_fall;

  wire do_y_inc = (Y_INC_ON_DE_RISE) ? de_rise : de_fall;
  wire do_x_inc = pix_valid;

  // gate (avoid function return logic)
  wire g_ok = (GATE_BY_IN_FRAME) ? in_frame_i : 1'b1;

  // -----------------------------
  // pulses
  // -----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      frame_start_p <= 1'b0;
      line_start_p  <= 1'b0;
      line_end_p    <= 1'b0;
    end else begin
      frame_start_p <= do_frame_start;
      line_start_p  <= do_line_start;
      line_end_p    <= do_line_end;
    end
  end

  // -----------------------------
  // bounds flags
  // -----------------------------
  always_comb begin
    if (!ENABLE_BOUNDS) begin
      x_at_last  = 1'b0;
      y_at_last  = 1'b0;
      x_overflow = 1'b0;
      y_overflow = 1'b0;
    end else begin
      x_at_last  = (x == X_LAST);
      y_at_last  = (y == Y_LAST);
      x_overflow = (do_x_inc && g_ok && (x == X_LAST));
      y_overflow = (do_y_inc && g_ok && (y == Y_LAST));
    end
  end

  // -----------------------------
  // next state
  // -----------------------------
  logic [X_W-1:0] x_n;
  logic [Y_W-1:0] y_n;

  always_comb begin
    x_n = x;

    if (do_frame_start) begin
      x_n = '0;
    end else if (do_line_start) begin
      x_n = '0;
    end else if (do_x_inc && g_ok) begin
      if (!ENABLE_BOUNDS) begin
        x_n = x + X_ONE;
      end else begin
        if (x != X_LAST) begin
          x_n = x + X_ONE;
        end else begin
          case (X_LIMIT_MODE)
            0: x_n = x + X_ONE;
            1: x_n = '0;
            default: x_n = X_LAST;
          endcase
        end
      end
    end
  end

  always_comb begin
    y_n = y;

    if (do_frame_start) begin
      y_n = '0;
    end else if (do_y_inc && g_ok && !(cut_pending)) begin
      if (!ENABLE_BOUNDS) begin
        y_n = y + Y_ONE;
      end else begin
        if (y != Y_LAST) begin
          y_n = y + Y_ONE;
        end else begin
          case (Y_LIMIT_MODE)
            0: y_n = y + Y_ONE;
            1: y_n = '0;
            default: y_n = Y_LAST;
          endcase
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      x <= '0;
      y <= '0;
    end else begin
      x <= x_n;
      y <= y_n;
    end
  end

  // -----------------------------
  // track DE level (so we know if we're mid-line)
  // -----------------------------
  logic de_level;

  always_ff @(posedge clk) begin
    if (rst) begin
      de_level <= 1'b0;
    end else begin
      // new frame cuts any ongoing line
      if (do_frame_start) begin
        de_level <= 1'b0;
      end else if (do_line_start) begin
        de_level <= 1'b1;
      end else if (do_line_end) begin
        de_level <= 1'b0;
      end
    end
  end

  // -----------------------------
  // suppress de_fall that is caused by vs_rise cutting an active line
  // -----------------------------
  logic cut_pending;

  always_ff @(posedge clk) begin
    if (rst) begin
      cut_pending <= 1'b0;
    end else begin
      if (do_frame_start) begin
        // if DE was high BEFORE frame_start, we are cutting an active line
        cut_pending <= de_level;
      end else if (do_line_end && cut_pending) begin
        // consume the suppressed line_end
        cut_pending <= 1'b0;
      end else if (do_line_start) begin
        // clean new line start clears any pending cut
        cut_pending <= 1'b0;
      end
    end
  end



endmodule

`default_nettype wire
`endif
