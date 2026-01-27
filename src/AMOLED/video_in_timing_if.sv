`ifndef VIDEO_IN_TIMING_IF_SV
`define VIDEO_IN_TIMING_IF_SV
`timescale 1ns/1ps
`default_nettype none

module video_in_timing_if #(
  parameter int X_W = 11,
  parameter int Y_W = 10
)(
  input  logic           clk,
  input  logic           rst,

  input  logic           vsync,
  input  logic           hsync,
  input  logic           de,
  input  logic [23:0]    rgb,

  output logic           pix_valid,
  output logic [23:0]    pix_rgb,
  output logic [X_W-1:0] x,
  output logic [Y_W-1:0] y,
  output logic           sof,
  output logic           sol,
  output logic           eol,
  output logic           eof
);

  // ----------------------------
  // 1) input sync (1-stage) + edge detect
  //    只做「一級」同步，輸出直接用這級 => pix_valid 延遲 1 拍
  // ----------------------------
  logic vs_q, hs_q, de_q;
  logic [23:0] rgb_q;

  logic vs_d, hs_d, de_d;   // previous of *_q

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_q  <= 1'b0;
      hs_q  <= 1'b0;
      de_q  <= 1'b0;
      rgb_q <= '0;

      vs_d  <= 1'b0;
      hs_d  <= 1'b0;
      de_d  <= 1'b0;
    end else begin
      // stage-1 sync
      vs_q  <= vsync;
      hs_q  <= hsync;
      de_q  <= de;
      rgb_q <= rgb;

      // previous (for edge detect)
      vs_d  <= vs_q;
      hs_d  <= hs_q;
      de_d  <= de_q;
    end
  end

  wire de_rise = (de_q && !de_d);
  wire de_fall = (!de_q && de_d);
  wire vs_rise = (vs_q && !vs_d);

  // ----------------------------
  // 2) pixel outputs (NO extra register stage!)
  // ----------------------------
  // pix_valid/pix_rgb = 同步後的一級暫存 => de 延遲 1 拍
  always_comb begin
    pix_valid = de_q;
    pix_rgb   = rgb_q;
  end

  // ----------------------------
  // 3) x/y counters on active pixels only
  // (保留你原本行為：de_rise 時 x=0，de_q 時 x++)
  // 注意：你原本 y 在 de_rise 會 +1，所以第一行會變 y=1
  // 如果你要第一行 y=0，我下一段也給你改法
  // ----------------------------
  logic last_pix_valid;

  always_ff @(posedge clk) begin
    if (rst) begin
      x <= '0;
      y <= '0;
      last_pix_valid <= 1'b0;
    end else begin
      last_pix_valid <= de_q;

      if (vs_rise) begin
        x <= '0;
        y <= '0;
      end else begin
        if (de_rise) begin
          x <= '0;
          // 你原本行為：每行開始 y++（會讓第一行 y=1）
          y <= y + 1'b1;
        end else if (de_q) begin
          x <= x + 1'b1;
        end
      end
    end
  end

  // ----------------------------
  // 4) SOF/SOL/EOL/EOF pulses
  // (維持你原本 pulse 定義)
  // ----------------------------
  logic sof_armed;
  always_ff @(posedge clk) begin
    if (rst) begin
      sof_armed <= 1'b0;
    end else begin
      if (vs_rise) sof_armed <= 1'b1;
      else if (de_rise && sof_armed) sof_armed <= 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      sof <= 1'b0; sol <= 1'b0; eol <= 1'b0; eof <= 1'b0;
    end else begin
      sof <= (de_rise && sof_armed);
      sol <= (de_rise);

      // 你的 eol 打在 de_fall 的那拍，通常那拍 pix_valid=0（因為 de_q 已經變 0）
      // 先保持你原行為不動
      eol <= (de_fall && last_pix_valid);

      // eof：vs_rise 當拍且上一拍還在有效像素
      eof <= (vs_rise && last_pix_valid);
    end
  end

endmodule

`default_nettype wire
`endif
