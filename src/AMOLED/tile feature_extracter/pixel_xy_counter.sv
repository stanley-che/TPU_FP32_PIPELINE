// pixel_xy_counter.sv — redesigned (clean skid-buffer, stall-safe) + FIX pop+push same-cycle
`ifndef PIXEL_XY_COUNTER_SV
`define PIXEL_XY_COUNTER_SV

`timescale 1ns/1ps
`default_nettype none

module pixel_xy_counter #(
  parameter int unsigned X_W      = 11,
  parameter int unsigned Y_W      = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  parameter int unsigned TILE_SHIFT = 2,     // 4x4 => shift=2
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  parameter bit          USE_SOF    = 1'b1,
  parameter bit          USE_EOL    = 1'b0,
  parameter bit          USE_EOF    = 1'b0,
  parameter bit          SAT_AT_MAX = 1'b0,

  // if 1: advance coordinate only when output is consumed (fire_out)
  // if 0: advance on every accepted input (fire_in)
  parameter bit          ADVANCE_ON_VALID_ONLY = 1'b1,

  parameter bit          ROI_EN_DEFAULT = 1'b0,
  parameter int unsigned ROI_X0_DEFAULT = 0,
  parameter int unsigned ROI_Y0_DEFAULT = 0,
  parameter int unsigned ROI_X1_DEFAULT = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT = ACTIVE_H-1,

  parameter bit          DBG_COUNTERS_EN = 1'b1
)(
  input  logic                  clk,
  input  logic                  rst,
  input  logic                  en,

  input  logic                  pix_valid,
  output logic                  pix_ready,

  input  logic                  sof,
  input  logic                  eol,
  input  logic                  eof,

  output logic                  v1_valid,
  input  logic                  v1_ready,
  output logic [X_W-1:0]        x,
  output logic [Y_W-1:0]        y,

  output logic [X_W-1:0]        tile_j,
  output logic [Y_W-1:0]        tile_i,
  output logic [1:0]            x_mod,
  output logic [1:0]            y_mod,

  output logic                  tile_first,
  output logic                  tile_last,
  output logic                  tile_x_last,
  output logic                  tile_y_last,

  output logic                  x_last,
  output logic                  y_last,
  output logic                  line_start,
  output logic                  line_last,
  output logic                  frame_start,
  output logic                  frame_last,

  output logic [$clog2(TILES_X*TILES_Y)-1:0] tile_idx,

  output logic                  in_roi,

  output logic                  err_sof_midframe,
  output logic                  err_eol_mismatch,
  output logic [31:0]           frame_cnt,
  output logic [31:0]           line_cnt
);

  // -----------------------------------------
  // constants / helpers
  // -----------------------------------------
  localparam int unsigned X_LAST_U = (ACTIVE_W == 0) ? 0 : (ACTIVE_W - 1);
  localparam int unsigned Y_LAST_U = (ACTIVE_H == 0) ? 0 : (ACTIVE_H - 1);

  function automatic logic is_x_last(input logic [X_W-1:0] xv);
    begin is_x_last = (xv == X_LAST_U[X_W-1:0]); end
  endfunction

  function automatic logic is_y_last(input logic [Y_W-1:0] yv);
    begin is_y_last = (yv == Y_LAST_U[Y_W-1:0]); end
  endfunction

  localparam int unsigned ROI_X0 = ROI_X0_DEFAULT;
  localparam int unsigned ROI_Y0 = ROI_Y0_DEFAULT;
  localparam int unsigned ROI_X1 = ROI_X1_DEFAULT;
  localparam int unsigned ROI_Y1 = ROI_Y1_DEFAULT;

  // -----------------------------------------
  // running coordinate: "next coordinate to output"
  // -----------------------------------------
  logic [X_W-1:0] x_r;
  logic [Y_W-1:0] y_r;

  // -----------------------------------------
  // 1-entry output buffer (skid)
  // -----------------------------------------
  logic [X_W-1:0] x_q;
  logic [Y_W-1:0] y_q;
  logic sof_q, eol_q, eof_q;

  // -----------------------------------------
  // handshakes
  // -----------------------------------------
  wire fire_out = v1_valid && v1_ready;

  // buffer can accept new input if empty OR it will be popped this cycle
  wire buf_can_take = en && (!v1_valid || fire_out);

  assign pix_ready = buf_can_take;         // classic 1-stage skid
  wire fire_in = pix_valid && pix_ready;   // accepted input beat

  // -----------------------------------------
  // combinational "advance one step" from current x_r/y_r (for same-cycle pop+push fix)
  // -----------------------------------------
  logic [X_W-1:0] x_r_adv;
  logic [Y_W-1:0] y_r_adv;

  always_comb begin
    x_r_adv = x_r;
    y_r_adv = y_r;

    if (!is_x_last(x_r)) begin
      x_r_adv = x_r + {{(X_W-1){1'b0}},1'b1};
    end else begin
      if (!SAT_AT_MAX) x_r_adv = '0;

      if (!is_y_last(y_r)) begin
        y_r_adv = y_r + {{(Y_W-1){1'b0}},1'b1};
      end else begin
        if (!SAT_AT_MAX) y_r_adv = '0;
      end
    end
  end

  // -----------------------------------------
  // coordinate advance logic (one step) + counters (sequential)
  // -----------------------------------------
  task automatic advance_xy;
    begin
      if (!is_x_last(x_r)) begin
        x_r <= x_r + {{(X_W-1){1'b0}},1'b1};
      end else begin
        if (!SAT_AT_MAX) x_r <= '0;

        if (!is_y_last(y_r)) begin
          y_r <= y_r + {{(Y_W-1){1'b0}},1'b1};
          if (DBG_COUNTERS_EN) line_cnt <= line_cnt + 32'd1;
        end else begin
          if (!SAT_AT_MAX) y_r <= '0;
          if (DBG_COUNTERS_EN) begin
            frame_cnt <= frame_cnt + 32'd1;
            line_cnt  <= 32'd0;
          end
        end
      end
    end
  endtask

  // -----------------------------------------
  // sequential
  // -----------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      x_r <= '0; y_r <= '0;

      x_q <= '0; y_q <= '0;
      sof_q <= 1'b0; eol_q <= 1'b0; eof_q <= 1'b0;
      v1_valid <= 1'b0;

      err_sof_midframe <= 1'b0;
      err_eol_mismatch <= 1'b0;

      frame_cnt <= 32'd0;
      line_cnt  <= 32'd0;

    end else if (!en) begin
      v1_valid <= 1'b0;
      sof_q <= 1'b0; eol_q <= 1'b0; eof_q <= 1'b0;

    end else begin
      // -------------------------------------
      // consume output
      // -------------------------------------
      if (fire_out) begin
        v1_valid <= 1'b0;

        if (ADVANCE_ON_VALID_ONLY) begin
          // SOF/EOF beat: 已在 fire_in 時把 x_r/y_r 設到 next 了，consume 不要再 advance
          if ((USE_SOF && sof_q) || (USE_EOF && eof_q)) begin
            // no-op
          end else if (USE_EOL && eol_q) begin
            // (若你未來開 USE_EOL) consume EOL 後進行換行
            x_r <= '0;
            if (!is_y_last(y_r)) y_r <= y_r + {{(Y_W-1){1'b0}},1'b1};
            else if (!SAT_AT_MAX) y_r <= '0;

            if (DBG_COUNTERS_EN) line_cnt <= line_cnt + 32'd1;
          end else begin
            // normal beat consume -> advance
            advance_xy();
          end
        end
      end

      // -------------------------------------
      // accept new input into buffer (may happen with or without fire_out)
      // -------------------------------------
      if (fire_in) begin
        // sticky debug checks based on CURRENT running coordinate
        if (USE_SOF && sof) begin
          if (!((x_r == '0) && (y_r == '0))) err_sof_midframe <= 1'b1;
        end
        if (USE_EOL && eol) begin
          if (!is_x_last(x_r)) err_eol_mismatch <= 1'b1;
        end

        // capture syncs for this buffered beat
        sof_q <= sof;
        eol_q <= eol;
        eof_q <= eof;

        // define output coordinate for THIS accepted beat
        if ((USE_SOF && sof) || (USE_EOF && eof)) begin
          // control beat outputs (0,0)
          x_q <= '0;
          y_q <= '0;

          // treat SOF/EOF as "start new frame": set running coordinate to next=(1,0) (or edge cases)
          if (ACTIVE_W <= 1) begin
            x_r <= '0;
            if (ACTIVE_H <= 1) y_r <= '0;
            else               y_r <= {{(Y_W-1){1'b0}}, 1'b1};
          end else begin
            x_r <= {{(X_W-1){1'b0}}, 1'b1}; // 1
            y_r <= '0;
          end

          if (DBG_COUNTERS_EN) begin
            frame_cnt <= frame_cnt + 32'd1;
            line_cnt  <= 32'd0;
          end

        end else begin
          // -----------------------------
          // ★FIX: pop+push same cycle
          // ADVANCE_ON_VALID_ONLY=1 時，如果同拍 fire_out+fire_in，
          // x_r/y_r 在本拍會被 consume-side advance，但 nonblocking 使得 x_r 仍是舊值。
          // 新 push 進來的 beat 必須吃 advance 後的座標，否則會重複/少一步。
          // -----------------------------
          if (fire_out && ADVANCE_ON_VALID_ONLY) begin
            x_q <= x_r_adv;
            y_q <= y_r_adv;
          end else begin
            x_q <= x_r;
            y_q <= y_r;
          end

          // advance coordinate if we advance-on-input mode
          if (!ADVANCE_ON_VALID_ONLY) begin
            advance_xy();
          end
        end

        // buffer becomes valid (even if we also fired out this cycle)
        v1_valid <= 1'b1;
      end
    end
  end

  // -----------------------------------------
  // outputs (combinational)
  // -----------------------------------------
  assign x = x_q;
  assign y = y_q;

  assign x_mod = x_q[1:0];
  assign y_mod = y_q[1:0];

  assign tile_j = (x_q >> TILE_SHIFT);
  assign tile_i = (y_q >> TILE_SHIFT);

  assign x_last      = v1_valid && is_x_last(x_q);
  assign y_last      = v1_valid && is_y_last(y_q);
  assign line_start  = v1_valid && (x_q == '0);
  assign line_last   = v1_valid && is_x_last(x_q);
  assign frame_start = v1_valid && (x_q == '0) && (y_q == '0);
  assign frame_last  = v1_valid && is_x_last(x_q) && is_y_last(y_q);

  assign tile_first  = v1_valid && (x_q[1:0]==2'd0) && (y_q[1:0]==2'd0);
  assign tile_last   = v1_valid && (x_q[1:0]==2'd3) && (y_q[1:0]==2'd3);
  assign tile_x_last = v1_valid && (x_q[1:0]==2'd3);
  assign tile_y_last = v1_valid && (y_q[1:0]==2'd3);

  // tile_idx
  wire [$clog2(TILES_X*TILES_Y)-1:0] tile_i_l;
  wire [$clog2(TILES_X*TILES_Y)-1:0] tile_j_l;
  assign tile_i_l = tile_i[$clog2(TILES_X*TILES_Y)-1:0];
  assign tile_j_l = tile_j[$clog2(TILES_X*TILES_Y)-1:0];
  assign tile_idx = tile_i_l * TILES_X[$clog2(TILES_X*TILES_Y)-1:0] + tile_j_l;

  assign in_roi =
    v1_valid &&
    (x_q >= ROI_X0[X_W-1:0]) && (x_q <= ROI_X1[X_W-1:0]) &&
    (y_q >= ROI_Y0[Y_W-1:0]) && (y_q <= ROI_Y1[Y_W-1:0]);

endmodule

`default_nettype wire
`endif
