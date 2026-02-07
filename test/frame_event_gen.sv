// frame_event_gen.sv
`timescale 1ns/1ps
`default_nettype none

module frame_event_gen #(
  parameter bit USE_VS_FALL   = 1'b0, // 1: eof 用 vs_fall, 0: eof 用 next vs_rise 推斷
  parameter bit GATE_BY_FRAME = 1'b1  // 1: sol/eol 只在 in_frame=1 時才有效
)(
  input  logic        clk,
  input  logic        rst,

  // pixel stream (already aligned/cleaned by your front-end)
  input  logic        pix_valid,
  input  logic [23:0] pix_rgb_in,
  input  logic [10:0] x,
  input  logic [9:0]  y,

  // edge pulses from sync_edge_top (already cleaned)
  input  logic        vs_rise,
  input  logic        vs_fall,   // 若沒有就 tie 0，並設 USE_VS_FALL=0
  input  logic        de_rise,
  input  logic        de_fall,

  // optional gating (from your tracker)
  input  logic        in_frame,

  // outputs (1-cycle pulses)
  output logic        sof,
  output logic        sol,
  output logic        eol,
  output logic        eof,

  // forwarded pixel bus
  output logic        pix_valid_o,
  output logic [23:0] pix_rgb_out,
  output logic [10:0] x_o,
  output logic [9:0]  y_o
);

  // ------------------------------------------------------------
  // register pixel bus (nice for timing)
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      pix_valid_o  <= 1'b0;
      pix_rgb_out  <= 24'h0;
      x_o          <= '0;
      y_o          <= '0;
    end else begin
      pix_valid_o <= pix_valid;
      // 只在 pix_valid 時更新 rgb，避免無效區亂跳（也可改成每拍都打拍）
      if (pix_valid) pix_rgb_out <= pix_rgb_in;
      x_o <= x;
      y_o <= y;
    end
  end

  // ------------------------------------------------------------
  // frame state (only needed for eof inference when no vs_fall)
  // ------------------------------------------------------------
  logic in_frame_seen;

  always_ff @(posedge clk) begin
    if (rst) begin
      in_frame_seen <= 1'b0;
    end else begin
      // 以 vs_rise 當 frame open
      if (vs_rise) in_frame_seen <= 1'b1;

      // 若有 vs_fall，則明確 close
      if (USE_VS_FALL && vs_fall) in_frame_seen <= 1'b0;

      // 若沒有 vs_fall：不在這裡關，留給「next vs_rise 產生 eof」的策略
      // （in_frame_seen 會一直維持 1，直到 reset；這沒關係，因為 eof 由 vs_rise+in_frame_seen 觸發）
    end
  end

  // ------------------------------------------------------------
  // event generation (combinational, then registered as 1-cycle)
  // ------------------------------------------------------------
  logic sof_n, sol_n, eol_n, eof_n;

  always_comb begin
    // default
    sof_n = 1'b0;
    sol_n = 1'b0;
    eol_n = 1'b0;
    eof_n = 1'b0;

    // sof: always vs_rise
    sof_n = vs_rise;

    // sol/eol: de edges, optional gate by in_frame
    if (GATE_BY_FRAME) begin
      sol_n = de_rise & in_frame;
      eol_n = de_fall & in_frame;
    end else begin
      sol_n = de_rise;
      eol_n = de_fall;
    end

    // eof:
    if (USE_VS_FALL) begin
      eof_n = vs_fall;
    end else begin
      // 沒有 vs_fall：用「下一次 vs_rise 結束上一幀」
      // 若上一幀還被視為 active（in_frame_seen=1），則同拍 eof=1
      // 注意：這會造成 eof 和 sof 同拍都為 1（上一幀結束 + 新一幀開始），這在很多 pipeline 是 OK 的
      eof_n = vs_rise & in_frame_seen;
    end
  end

  // register pulses to ensure single-cycle
  always_ff @(posedge clk) begin
    if (rst) begin
      sof <= 1'b0;
      sol <= 1'b0;
      eol <= 1'b0;
      eof <= 1'b0;
    end else begin
      sof <= sof_n;
      sol <= sol_n;
      eol <= eol_n;
      eof <= eof_n;
    end
  end

endmodule

`default_nettype wire
