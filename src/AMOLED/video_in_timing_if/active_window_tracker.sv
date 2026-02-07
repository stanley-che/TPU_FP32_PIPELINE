`ifndef ACTIVE_WINDOW_TRACKER_SV
`define ACTIVE_WINDOW_TRACKER_SV

`timescale 1ns/1ps
`default_nettype none

module active_window_tracker #(
  // 1: 用 vs_rise 當作 frame start；0: 用 vs_fall 當作 frame start
  parameter bit FRAME_START_ON_VS_RISE = 1'b1,
  // 1: 用 hs_rise 當作 eol (可選)；0: 不用 hsync 管 line 結束（預設不用）
  parameter bit USE_HSYNC_FOR_EOL      = 1'b0
)(
  input  logic clk,
  input  logic rst,

  // edges (建議來自 sync_edge / sanitizer)
  input  logic vs_rise,
  input  logic vs_fall,
  input  logic hs_rise,
  input  logic hs_fall,
  input  logic de,
  input  logic de_rise,
  input  logic de_fall,

  // state
  output logic in_frame,   // 當前在一幀內
  output logic in_line,    // 當前在一行的 active 區
  output logic pix_valid   // 通常 = in_frame & in_line & de
);

  // ------------------------------------------------------------
  // frame tracking
  // ------------------------------------------------------------
  wire frame_start = (FRAME_START_ON_VS_RISE) ? vs_rise : vs_fall;
  wire frame_end   = (FRAME_START_ON_VS_RISE) ? vs_fall : vs_rise;

  // ------------------------------------------------------------
  // sequential
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      in_frame  <= 1'b0;
      in_line   <= 1'b0;
      pix_valid <= 1'b0;
    end else begin
      // ---- in_frame ----
      if (frame_start) begin
        in_frame <= 1'b1;
      end else if (frame_end) begin
        in_frame <= 1'b0;
      end

      // ---- in_line ----
      // 以 de_rise 進入 active line、de_fall 離開
      // （只有在 in_frame=1 時才允許進入，避免 blanking 亂入）
      if (!in_frame) begin
        in_line <= 1'b0;
      end else begin
        if (de_rise) begin
          in_line <= 1'b1;
        end

        // line end：優先用 de_fall；若你要保險，也可讓 hsync 邊緣強制結束
        if (de_fall) begin
          in_line <= 1'b0;
        end else if (USE_HSYNC_FOR_EOL && (hs_rise || hs_fall)) begin
          in_line <= 1'b0;
        end

        // frame end 時也要清 line（避免跨帧殘留）
        if (frame_end) begin
          in_line <= 1'b0;
        end
      end

      // ---- pix_valid ----
      // 通常你 downstream 只吃 active 區，所以以 in_frame & in_line & de 最保險
      pix_valid <= (in_frame && in_line && de);
    end
  end

endmodule

`default_nettype wire
`endif
