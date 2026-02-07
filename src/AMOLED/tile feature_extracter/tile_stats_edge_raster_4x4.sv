`ifndef TILE_STATS_EDGE_RASTER_4X4_SV
`define TILE_STATS_EDGE_RASTER_4X4_SV

`timescale 1ns/1ps
`default_nettype none

module tile_stats_edge_raster_4x4 #(
  parameter int unsigned Y_W      = 8,
  parameter int unsigned TILE_W   = 4,
  parameter int unsigned TILE_H   = 4,
  parameter int unsigned TILE_PIX = TILE_W*TILE_H, // 16
  parameter int unsigned TILES_X  = 320,           // tile columns

  // stats widths
  parameter int unsigned SUM_W    = 12,            // enough for 16*255=4080 -> 12 bits
  parameter int unsigned EDGE_W   = 32,

  // output fifo depth (burst)
  parameter int unsigned OUT_FIFO_DEPTH = 2
)(
  input  logic                   clk,
  input  logic                   rst,
  input  logic                   en,

  // -----------------------------
  // raster pixel stream (already has x/y/tile meta)
  // -----------------------------
  input  logic                   v2_valid,
  output logic                   v2_ready,

  input  logic [Y_W-1:0]         y_cur,
  input  logic [Y_W-1:0]         y_left,
  input  logic [Y_W-1:0]         y_up,

  input  logic [15:0]            tile_x_idx,   // == tile_j
  input  logic [15:0]            tile_y_idx,   // == tile_i
  input  logic [1:0]             x_mod,
  input  logic [1:0]             y_mod,

  input  logic [15:0]            frame_id,

  // -----------------------------
  // output per-tile (paired stats+edge)
  // -----------------------------
  output logic                   out_valid,
  input  logic                   out_ready,

  output logic [SUM_W-1:0]       sumY,
  output logic [Y_W-1:0]         minY,
  output logic [Y_W-1:0]         maxY,

  output logic [EDGE_W-1:0]      edge_sum,

  output logic [15:0]            tile_x_o,
  output logic [15:0]            tile_y_o,
  output logic [15:0]            frame_id_o
);

  // ============================================================
  // Helpers
  // ============================================================
  function automatic [Y_W:0] abs_diff(input logic [Y_W-1:0] a, input logic [Y_W-1:0] b);
    logic [Y_W:0] aa, bb;
    begin
      aa = {1'b0,a};
      bb = {1'b0,b};
      abs_diff = (aa >= bb) ? (aa - bb) : (bb - aa);
    end
  endfunction

  // tile boundary in raster
  wire is_tile_first = (x_mod == 2'd0) && (y_mod == 2'd0);
  wire is_tile_last  = (x_mod == (TILE_W-1)) && (y_mod == (TILE_H-1));

  // index (assume tile_x_idx < TILES_X)
  wire [$clog2(TILES_X)-1:0] tx = tile_x_idx[$clog2(TILES_X)-1:0];

  // ============================================================
  // Per-tile-column accumulators (for current tile-row band)
  // ============================================================
  logic [SUM_W-1:0]  sum_acc   [TILES_X];
  logic [Y_W-1:0]    min_acc   [TILES_X];
  logic [Y_W-1:0]    max_acc   [TILES_X];
  logic [EDGE_W-1:0] edge_acc  [TILES_X];
  logic [15:0]       ty_acc    [TILES_X];
  logic [15:0]       fid_acc   [TILES_X];

  integer i;
  always_ff @(posedge clk) begin
    if (rst) begin
      for (i=0; i<TILES_X; i=i+1) begin
        sum_acc[i]  <= '0;
        min_acc[i]  <= {Y_W{1'b1}};
        max_acc[i]  <= '0;
        edge_acc[i] <= '0;
        ty_acc[i]   <= '0;
        fid_acc[i]  <= '0;
      end
    end else if (en) begin
      if (v2_valid && v2_ready) begin
        // reset this tile column at tile_first of that tile
        if (is_tile_first) begin
          sum_acc[tx]  <= {{(SUM_W-Y_W){1'b0}}, y_cur};
          min_acc[tx]  <= y_cur;
          max_acc[tx]  <= y_cur;
          edge_acc[tx] <= '0;
          ty_acc[tx]   <= tile_y_idx;
          fid_acc[tx]  <= frame_id;
        end else begin
          // normal accumulate
          sum_acc[tx] <= sum_acc[tx] + {{(SUM_W-Y_W){1'b0}}, y_cur};
          if (y_cur < min_acc[tx]) min_acc[tx] <= y_cur;
          if (y_cur > max_acc[tx]) max_acc[tx] <= y_cur;
        end

        // edge accumulate (simple): |cur-left| + |cur-up|
        // (you可以換成你原本更複雜的 edge 定義)
        edge_acc[tx] <= edge_acc[tx]
                      + {{(EDGE_W-(Y_W+1)){1'b0}}, abs_diff(y_cur, y_left)}
                      + {{(EDGE_W-(Y_W+1)){1'b0}}, abs_diff(y_cur, y_up)};
      end
    end
  end

  // ============================================================
  // Output FIFO (burst=2)
  // ============================================================
  localparam int unsigned OD = OUT_FIFO_DEPTH;
  localparam int unsigned O_PTR_W = (OD <= 2) ? 1 : $clog2(OD);

  logic [O_PTR_W-1:0] o_rd, o_wr;
  logic [$clog2(OD):0] o_cnt;

  wire o_full  = (o_cnt == OD);
  wire o_empty = (o_cnt == 0);

  // accept pixels only if output fifo not full when tile_last wants to push
  // easiest safe rule: stall input only when we are at tile_last AND fifo full
  // (otherwise keep accepting to not throttle raster)
  assign v2_ready = ~(o_full && v2_valid && is_tile_last);

  // push output when tile_last fires
  wire o_push = v2_valid && v2_ready && is_tile_last;

  // pop when downstream takes
  assign out_valid = ~o_empty;
  wire o_pop = out_valid && out_ready;

  // fifo payload
  logic [SUM_W-1:0]  sum_mem   [OD];
  logic [Y_W-1:0]    min_mem   [OD];
  logic [Y_W-1:0]    max_mem   [OD];
  logic [EDGE_W-1:0] edge_mem  [OD];
  logic [15:0]       tx_mem    [OD];
  logic [15:0]       ty_mem    [OD];
  logic [15:0]       fid_mem   [OD];

  // drive outputs from head
  assign sumY       = sum_mem[o_rd];
  assign minY       = min_mem[o_rd];
  assign maxY       = max_mem[o_rd];
  assign edge_sum   = edge_mem[o_rd];
  assign tile_x_o   = tx_mem[o_rd];
  assign tile_y_o   = ty_mem[o_rd];
  assign frame_id_o = fid_mem[o_rd];

  always_ff @(posedge clk) begin
    if (rst) begin
      o_rd  <= '0;
      o_wr  <= '0;
      o_cnt <= '0;
    end else if (en) begin
      case ({o_push, o_pop})
        2'b10: begin
          sum_mem[o_wr]  <= sum_acc[tx];
          min_mem[o_wr]  <= min_acc[tx];
          max_mem[o_wr]  <= max_acc[tx];
          edge_mem[o_wr] <= edge_acc[tx];
          tx_mem[o_wr]   <= tile_x_idx;
          ty_mem[o_wr]   <= ty_acc[tx];
          fid_mem[o_wr]  <= fid_acc[tx];

          o_wr  <= o_wr + 1'b1;
          o_cnt <= o_cnt + 1'b1;
        end
        2'b01: begin
          o_rd  <= o_rd + 1'b1;
          o_cnt <= o_cnt - 1'b1;
        end
        2'b11: begin
          // pop+push
          o_rd <= o_rd + 1'b1;

          sum_mem[o_wr]  <= sum_acc[tx];
          min_mem[o_wr]  <= min_acc[tx];
          max_mem[o_wr]  <= max_acc[tx];
          edge_mem[o_wr] <= edge_acc[tx];
          tx_mem[o_wr]   <= tile_x_idx;
          ty_mem[o_wr]   <= ty_acc[tx];
          fid_mem[o_wr]  <= fid_acc[tx];

          o_wr <= o_wr + 1'b1;
          // o_cnt unchanged
        end
        default: ;
      endcase
    end
  end

endmodule

`default_nettype wire
`endif
