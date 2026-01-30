`ifndef TILE_EDGE_ENERGY_4X4_SV
`define TILE_EDGE_ENERGY_4X4_SV
`timescale 1ns/1ps
`default_nettype none

module tile_edge_energy_4x4 #(
  parameter int unsigned Y_W      = 8,
  parameter int unsigned TILE_W   = 4,
  parameter int unsigned TILE_H   = 4,

  // feature switches
  parameter bit SUPPORT_PIX_VALID = 1'b1,
  parameter bit SUPPORT_TILE_START= 1'b1,
  parameter bit ASSERT_ON         = 1'b1,

  // energy modes:
  // 0: |dx|+|dy|
  // 1: max(|dx|,|dy|)
  // 2: threshold-count pixel contributes 1 if (|dx|+|dy| > THR)
  // 3: dx^2 + dy^2
  parameter int unsigned MODE     = 0,
  parameter int unsigned THR      = 8,

  // derived widths (must be parameter so ports can use it)
  parameter int unsigned EDGE_W   =
    ( (TILE_W*TILE_H*(2*((1<<Y_W)-1))) <= 1 ) ? 1 :
      $clog2( (TILE_W*TILE_H*(2*((1<<Y_W)-1))) + 1 ),

  parameter int unsigned CNT_W    =
    ( (TILE_W*TILE_H) <= 1 ) ? 1 : $clog2((TILE_W*TILE_H)+1),

  parameter int unsigned MEAN_W   = EDGE_W
)(
  input  logic                     clk,
  input  logic                     rst,

  // v2 streaming in (ready/valid)
  input  logic                     v2_valid,
  output logic                     v2_ready,

  input  logic [Y_W-1:0]            y_cur,
  input  logic [Y_W-1:0]            y_left,
  input  logic [Y_W-1:0]            y_up,
  input  logic [$clog2(TILE_W)-1:0] x_mod,
  input  logic [$clog2(TILE_H)-1:0] y_mod,

  input  logic                     tile_last,
  input  logic                     tile_start,          // optional (tie 0 if unused)
  input  logic                     pix_in_tile_valid,   // optional gating

  // v4 out (bundle)
  output logic                     v4_valid,
  input  logic                     v4_ready,
  output logic [EDGE_W-1:0]         edge_sum,
  output logic [EDGE_W-1:0]         edge_max,
  output logic [CNT_W-1:0]          edge_cnt,
  output logic [MEAN_W-1:0]         edge_mean
);

  localparam int unsigned TILE_PIXELS = TILE_W * TILE_H;

  // -----------------------------
  // output buffer (1-entry)
  // -----------------------------
  logic              out_v;
  logic [EDGE_W-1:0] out_sum, out_max;
  logic [CNT_W-1:0]  out_cnt;
  logic [MEAN_W-1:0] out_mean;

  assign v4_valid = out_v;

  // stall input whenever output buffer is full
  assign v2_ready = !out_v;

  wire accept_in  = v2_valid && v2_ready;

  // define "first pixel of tile"
  wire is_tile_first_xy = (x_mod == '0) && (y_mod == '0);
  wire is_tile_first    = (SUPPORT_TILE_START ? tile_start : 1'b0) || is_tile_first_xy;

  // pixel effective valid: B-mode (invalid pixel => contribute 0 energy, but still advance)
  wire pix_eff = (SUPPORT_PIX_VALID ? pix_in_tile_valid : 1'b1);

  // -----------------------------
  // abs diffs
  // -----------------------------
  logic [Y_W-1:0] dx_abs, dy_abs;

  always_comb begin
    if (x_mod == '0) dx_abs = '0;
    else             dx_abs = (y_cur >= y_left) ? (y_cur - y_left) : (y_left - y_cur);

    if (y_mod == '0) dy_abs = '0;
    else             dy_abs = (y_cur >= y_up) ? (y_cur - y_up) : (y_up - y_cur);
  end

  // -----------------------------
  // energy per pixel (mode selectable)
  // -----------------------------
  logic [Y_W:0]   e_abs_sum;     // |dx|+|dy| fits in Y_W+1
  logic [2*Y_W:0] e_sq_sum;      // dx^2 + dy^2: roughly up to 2*(2^Y-1)^2 => needs ~2Y+1
  logic [EDGE_W-1:0] e_pix_ext;
  logic [EDGE_W-1:0] e_for_max;  // energy value used for max tracking
  logic e_thr_hit;               // for cnt

  always_comb begin
    e_abs_sum = {1'b0, dx_abs} + {1'b0, dy_abs};

    // square (simple multiply; iverilog supports)
    e_sq_sum  = (dx_abs * dx_abs) + (dy_abs * dy_abs);

    // default
    e_thr_hit = (e_abs_sum > THR[Y_W:0]);

    // build e_pix_ext by MODE
    case (MODE)
      0: begin // abs sum
        e_for_max = {{(EDGE_W-($bits(e_abs_sum))){1'b0}}, e_abs_sum};
        e_pix_ext = pix_eff ? e_for_max : '0;
      end
      1: begin // max(|dx|,|dy|)
        logic [Y_W-1:0] m;
        m = (dx_abs >= dy_abs) ? dx_abs : dy_abs;
        e_for_max = {{(EDGE_W-Y_W){1'b0}}, m};
        e_pix_ext = pix_eff ? e_for_max : '0;
      end
      2: begin // threshold count: pixel contributes 1 if hit
        e_for_max = pix_eff && e_thr_hit ? {{(EDGE_W-1){1'b0}},1'b1} : '0;
        e_pix_ext = e_for_max;
      end
      3: begin // squared sum
        e_for_max = {{(EDGE_W-($bits(e_sq_sum))){1'b0}}, e_sq_sum[$bits(e_sq_sum)-1:0]};
        e_pix_ext = pix_eff ? e_for_max : '0;
      end
      default: begin
        e_for_max = {{(EDGE_W-($bits(e_abs_sum))){1'b0}}, e_abs_sum};
        e_pix_ext = pix_eff ? e_for_max : '0;
      end
    endcase
  end

  // -----------------------------
  // accumulators
  // -----------------------------
  logic [EDGE_W-1:0] sum_r, max_r;
  logic [CNT_W-1:0]  cnt_r;

  // next-state values
  logic [EDGE_W-1:0] sum_next, max_next;
  logic [CNT_W-1:0]  cnt_next;

  wire hit_cnt = (MODE==2) ? (e_pix_ext != '0) : (pix_eff && (e_abs_sum > THR[Y_W:0]));

  always_comb begin
    // sum
    if (is_tile_first) sum_next = e_pix_ext;
    else               sum_next = sum_r + e_pix_ext;

    // max
    if (is_tile_first) max_next = e_for_max;
    else               max_next = (e_for_max > max_r) ? e_for_max : max_r;

    // cnt
    if (is_tile_first) cnt_next = hit_cnt ? {{(CNT_W-1){1'b0}},1'b1} : '0;
    else               cnt_next = cnt_r + (hit_cnt ? {{(CNT_W-1){1'b0}},1'b1} : '0);
  end

  // mean (constant division)
  function automatic [MEAN_W-1:0] calc_mean(input [EDGE_W-1:0] s);
    calc_mean = s / TILE_PIXELS;
  endfunction

  // -----------------------------
  // sequential
  // -----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      sum_r    <= '0;
      max_r    <= '0;
      cnt_r    <= '0;
      out_v    <= 1'b0;
      out_sum  <= '0;
      out_max  <= '0;
      out_cnt  <= '0;
      out_mean <= '0;
    end else begin
      // consume output
      if (out_v && v4_ready) begin
        out_v <= 1'b0;
      end

      // accept input only when not holding output
      if (accept_in) begin
        // update accumulators
        sum_r <= sum_next;
        max_r <= max_next;
        cnt_r <= cnt_next;

        // tile_last => latch output to buffer (even if v4_ready=0)
        if (tile_last) begin
          out_sum  <= sum_next;
          out_max  <= max_next;
          out_cnt  <= cnt_next;
          out_mean <= calc_mean(sum_next);
          out_v    <= 1'b1;

          // prepare next tile
          sum_r <= '0;
          max_r <= '0;
          cnt_r <= '0;
        end
      end
    end
  end

  assign edge_sum  = out_sum;
  assign edge_max  = out_max;
  assign edge_cnt  = out_cnt;
  assign edge_mean = out_mean;

  // -----------------------------
  // assertions / checkers
  // -----------------------------
  generate if (ASSERT_ON) begin : G_ASSERT
    always_ff @(posedge clk) begin
      if (!rst) begin
        if (accept_in && tile_last) begin
          // If tile_last is meant to be raster-aligned, enforce
          assert (x_mod == TILE_W-1 && y_mod == TILE_H-1)
            else $fatal(1, "tile_last misaligned: x_mod=%0d y_mod=%0d", x_mod, y_mod);
        end

        if (accept_in && (SUPPORT_TILE_START && tile_start)) begin
          // if tile_start asserted, we expect x_mod/y_mod==0/0 (optional, but good)
          assert (x_mod=='0 && y_mod=='0)
            else $fatal(1, "tile_start but x_mod/y_mod not 0: x=%0d y=%0d", x_mod, y_mod);
        end

        // no accepting input while output buffer full (by design)
        assert (!(v2_valid && !v2_ready))
          else begin end
      end
    end
  end endgenerate

endmodule

`default_nettype wire
`endif