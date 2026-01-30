`ifndef VIDEO_RGB2Y_INTEGRATED_TOP_SV
`define VIDEO_RGB2Y_INTEGRATED_TOP_SV

`include "./src/AMOLED/video_in_timing_if/video_in_timing_if_top.sv"
`include "./src/AMOLED/rgb2y_luma/rgb2y_luma_top.sv"

`timescale 1ns/1ps
`default_nettype none

// ============================================================
// 1-deep skid buffer (ready/valid)
// - 可吸收短暫 out_ready=0 的停頓
// - 不能吸收長時間 backpressure（會 drop / 或你可改成 FIFO）
// ============================================================
module rv_skid_1 #(
  parameter int unsigned DW = 32
)(
  input  logic           clk,
  input  logic           rst,
  input  logic           en,

  input  logic           in_valid,
  output logic           in_ready,
  input  logic [DW-1:0]  in_data,

  output logic           out_valid,
  input  logic           out_ready,
  output logic [DW-1:0]  out_data,

  output logic           drop_pulse,
  output logic [31:0]    drop_cnt
);

  logic          full;
  logic [DW-1:0] buf_r;  

  always @(posedge clk) begin
    if (rst) begin
      full       <= 1'b0;
      buf_r      <= '0;
      drop_pulse <= 1'b0;
      drop_cnt   <= 32'd0;
    end else begin
      drop_pulse <= 1'b0;

      if (en) begin
        // pop
        if (full && out_ready) begin
          full <= 1'b0;
        end

        // push
        if (!full) begin
          if (in_valid && !out_ready) begin
            full  <= 1'b1;
            buf_r <= in_data;
          end
        end else begin
          if (in_valid) begin
            drop_pulse <= 1'b1;
            drop_cnt   <= drop_cnt + 1;
          end
        end
      end
    end
  end

  always @* begin
    if (!en) begin
      in_ready  = 1'b0;
      out_valid = 1'b0;
      out_data  = '0;
    end else if (full) begin
      out_valid = 1'b1;
      out_data  = buf_r;
      in_ready  = 1'b0;
    end else begin
      out_valid = in_valid;
      out_data  = in_data;
      in_ready  = out_ready;
    end
  end

endmodule


// ============================================================
// Integrated Top
// ============================================================
module video_rgb2y_integrated_top #(
  // ----------------------------
  // video_in_timing_if_top params
  // ----------------------------
  parameter bit VS_INV = 1'b0,
  parameter bit HS_INV = 1'b0,
  parameter bit DE_INV = 1'b0,
  parameter bit USE_DEGLITCH = 1'b0,
  parameter int unsigned STABLE_CYCLES = 2,

  parameter bit FRAME_START_ON_VS_RISE = 1'b1,
  parameter bit USE_HSYNC_FOR_EOL      = 1'b0,

  parameter int unsigned X_W = 11,
  parameter int unsigned Y_W = 10,
  parameter int unsigned ACTIVE_W = 1280,
  parameter int unsigned ACTIVE_H = 720,

  parameter bit Y_INC_ON_DE_RISE = 1'b0,
  parameter int unsigned X_LIMIT_MODE = 2, // 0 none, 1 wrap, 2 sat
  parameter int unsigned Y_LIMIT_MODE = 2,
  parameter bit ENABLE_BOUNDS     = 1'b1,
  parameter bit GATE_BY_IN_FRAME  = 1'b1,
  parameter bit ENABLE_ASSERT     = 1'b0,

  parameter bit FEV_USE_VS_FALL   = 1'b0,
  parameter bit FEV_GATE_BY_FRAME = 1'b1,

  // ----------------------------
  // rgb2y_luma_top params
  // ----------------------------
  parameter int unsigned FORMAT = 0,      // 0 RGB888
  parameter int unsigned LAT_U  = 1,
  parameter bit          ZERO_WHEN_INVALID = 1'b1,

  parameter int unsigned LAT_M  = 1,
  parameter bit          USE_DSP = 1'b0,
  parameter bit          BYPASS  = 1'b0,

  parameter int unsigned LAT_SUM = 1,
  parameter bit          HOLD_VALID_WHEN_STALL = 1'b1,

  parameter int unsigned LAT_RND = 1,
  parameter int unsigned ROUND_MODE = 1,
  parameter int unsigned Y8_MODE = 0,

  // sideband 定義：你想帶哪些就決定 SB_W 與組包順序
  // 這裡預設：{eof,eol,sol,sof, de,hs,vs, y, x} => 4+3+Y_W+X_W
  parameter int unsigned SB_W = (4+3+Y_W+X_W),

  // out_pack
  parameter int unsigned PACK_IN_W  = 10,
  parameter int unsigned PACK_OUT_W = 10,
  parameter int unsigned PACK_LAT   = 2,

  parameter bit PACK_GATE_SIDEBAND_WITH_VALID = 1'b1,
  parameter bit PACK_USE_ROUND = 1'b0,
  parameter bit PACK_USE_SAT   = 1'b1,
  parameter bit PACK_USE_CLIP  = 1'b0,
  parameter bit PACK_COUNT_DBG = 1'b0
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 en,

  // raw video in
  input  logic                 vsync_i,
  input  logic                 hsync_i,
  input  logic                 de_i,
  input  logic [23:0]          rgb_i,

  // clip controls
  input  logic                 clip_en,
  input  logic [PACK_OUT_W-1:0] clip_min,
  input  logic [PACK_OUT_W-1:0] clip_max,

  // output stream
  output logic                 out_valid,
  input  logic                 out_ready,
  output logic [PACK_OUT_W-1:0] Y,
  output logic [SB_W-1:0]       sb_out,

  // optional debug export
  output logic                 in_frame,
  output logic                 in_line,
  output logic                 pix_valid,     // from timing_if
  output logic [X_W-1:0]        x,
  output logic [Y_W-1:0]        y,

  // skid drop debug (若 downstream backpressure 太久會掉資料)
  output logic                 skid_drop_pulse,
  output logic [31:0]          skid_drop_cnt
);

  // ============================================================
  // A) timing_if
  // ============================================================
  logic        vsync, hsync, de;
  logic        pix_valid_o;
  logic [23:0] pix_rgb_out;
  logic [X_W-1:0] x_o;
  logic [Y_W-1:0] y_o;

  logic sof, sol, eol, eof;

  logic frame_start_p, line_start_p, line_end_p;
  logic x_at_last, y_at_last, x_overflow, y_overflow;

  video_in_timing_if_top #(
    .VS_INV(VS_INV),
    .HS_INV(HS_INV),
    .DE_INV(DE_INV),
    .USE_DEGLITCH(USE_DEGLITCH),
    .STABLE_CYCLES(STABLE_CYCLES),

    .FRAME_START_ON_VS_RISE(FRAME_START_ON_VS_RISE),
    .USE_HSYNC_FOR_EOL(USE_HSYNC_FOR_EOL),

    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .Y_INC_ON_DE_RISE(Y_INC_ON_DE_RISE),
    .X_LIMIT_MODE(X_LIMIT_MODE),
    .Y_LIMIT_MODE(Y_LIMIT_MODE),

    .ENABLE_BOUNDS(ENABLE_BOUNDS),
    .GATE_BY_IN_FRAME(GATE_BY_IN_FRAME),
    .ENABLE_ASSERT(ENABLE_ASSERT),

    .FEV_USE_VS_FALL(FEV_USE_VS_FALL),
    .FEV_GATE_BY_FRAME(FEV_GATE_BY_FRAME)
  ) u_timing (
    .clk(clk),
    .rst(rst),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i   (de_i),
    .rgb_i  (rgb_i),

    .vsync(vsync),
    .hsync(hsync),
    .de   (de),

    .in_frame(in_frame),
    .in_line (in_line),
    .pix_valid(pix_valid),

    .x(x),
    .y(y),

    .frame_start_p(frame_start_p),
    .line_start_p (line_start_p),
    .line_end_p   (line_end_p),

    .x_at_last(x_at_last),
    .y_at_last(y_at_last),
    .x_overflow(x_overflow),
    .y_overflow(y_overflow),

    .sof(sof),
    .sol(sol),
    .eol(eol),
    .eof(eof),

    .pix_valid_o(pix_valid_o),
    .pix_rgb_out(pix_rgb_out),
    .x_o(x_o),
    .y_o(y_o)
  );

  // ============================================================
  // B) build sideband + pack RGB888 -> 36b bus
  // ============================================================
  logic [SB_W-1:0] sb_in;
  logic [35:0]     rgb36;

  // sb layout: {eof,eol,sol,sof, de,hs,vs, y, x}
  always @* begin
    sb_in = '0;
    sb_in[0 +: X_W] = x_o;
    sb_in[X_W +: Y_W] = y_o;

    sb_in[X_W+Y_W +: 3] = {vsync, hsync, de};  // 3 bits
    sb_in[X_W+Y_W+3 +: 4] = {sof, sol, eol, eof}; // 4 bits
  end

  // rgb36 for FORMAT=0 (RGB888): put into [23:0]
  always @* begin
    rgb36 = 36'h0;
    rgb36[23:0] = pix_rgb_out;
  end

  // ============================================================
  // C) skid buffer between timing_if and rgb2y (absorb short stalls)
  // ============================================================
  localparam int unsigned SKID_DW = (36 + SB_W);

  logic               skid_in_ready;
  logic               skid_out_valid;
  logic               skid_out_ready;
  logic [SKID_DW-1:0] skid_out_data;

  rv_skid_1 #(
    .DW(SKID_DW)
  ) u_skid (
    .clk(clk),
    .rst(rst),
    .en(en),

    .in_valid(pix_valid_o),
    .in_ready(skid_in_ready), 
    .in_data({sb_in, rgb36}),

    .out_valid(skid_out_valid),
    .out_ready(skid_out_ready),
    .out_data(skid_out_data),

    .drop_pulse(skid_drop_pulse),
    .drop_cnt(skid_drop_cnt)
  );

  // ============================================================
  // D) rgb2y_luma_top
  // ============================================================
  logic                 rgb2y_in_ready;
  logic [35:0]          rgb2y_rgb_in;
  logic [SB_W-1:0]      rgb2y_sb_in;

  assign {rgb2y_sb_in, rgb2y_rgb_in} = skid_out_data;

  // skid_out_ready 由 rgb2y_in_ready 決定
  assign skid_out_ready = rgb2y_in_ready;

  rgb2y_luma_top #(
    .FORMAT(FORMAT),
    .LAT_U(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),

    .LAT_M(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS),

    .LAT_SUM(LAT_SUM),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),

    .LAT_RND(LAT_RND),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE),

    .PACK_IN_W(PACK_IN_W),
    .PACK_OUT_W(PACK_OUT_W),
    .SB_W(SB_W),
    .PACK_LAT(PACK_LAT),

    .PACK_GATE_SIDEBAND_WITH_VALID(PACK_GATE_SIDEBAND_WITH_VALID),
    .PACK_USE_ROUND(PACK_USE_ROUND),
    .PACK_USE_SAT(PACK_USE_SAT),
    .PACK_USE_CLIP(PACK_USE_CLIP),
    .PACK_COUNT_DBG(PACK_COUNT_DBG)
  ) u_rgb2y (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(skid_out_valid),
    .rgb_in(rgb2y_rgb_in),
    .sb_in(rgb2y_sb_in),
    .in_ready(rgb2y_in_ready),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .Y(Y),
    .sb_out(sb_out),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    // 你要的話把 debug port 都接出來；不需要可留空
    .v1_valid(),
    .r8(), .g8(), .b8(),
    .v2_valid(),
    .r_term(), .g_term(), .b_term(),
    .v4_valid(),
    .sum17(),
    .y10(),
    .y8(),
    .ovf_sum(),
    .ovf_y(),

    .drop_pulse(),
    .stall_pulse(),
    .drop_cnt(),
    .stall_cnt()
  );

endmodule

`default_nettype wire
`endif
