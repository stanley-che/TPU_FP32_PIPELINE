// ============================================================
// nb_fetch_scheduler_9req.sv  (IVERILOG-SAFE, 1-bundle buffered)
// 功能:
// - 把「一組 9 個 tile 座標」轉成 9 次 SRAM read request（逐筆送）
// - 內部 1-deep buffer：收到 nb bundle 後鎖住，直到 9req 全部送完
// - token_idx = 0..8，會跟著每一筆 request 送出
//
// Ready/Valid 語意:
// - nb_valid/nb_ready: 代表整包(9 coords) 進入 scheduler 的握手
// - rd_valid/rd_ready: 代表單筆 SRAM read request 的握手
//
// rd_tag 打包建議 (TAG_W=16):
// - 需要 token_idx(0..8) => 4 bits
// - 因為 TAG_W 固定 16，建議把 tag 的低 4 bits 用來放 token_idx：
//     rd_tag = { nb_tag[TAG_W-1:4], token_idx[3:0] }
//   也就是：group tag 只剩 12 bits（你可以接受就用）
//   若你想保留完整 16-bit group tag，請把 TAG_W 拉大到 20/24。
// ============================================================

`ifndef NB_FETCH_SCHEDULER_9REQ_SV
`define NB_FETCH_SCHEDULER_9REQ_SV

`timescale 1ns/1ps
`default_nettype none

module nb_fetch_scheduler_9req #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,
  parameter int unsigned TAG_W   = 16,
  parameter int unsigned IDX_W   = 4     // token_idx bits (need >=4 for 0..8)
)(
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         en,

  // ----------------------------
  // Input bundle (9 coords)
  // ----------------------------
  input  logic                         nb_valid,
  output logic                         nb_ready,

  input  logic [$clog2(TILES_Y)-1:0]    nb_i0,
  input  logic [$clog2(TILES_X)-1:0]    nb_j0,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i1,
  input  logic [$clog2(TILES_X)-1:0]    nb_j1,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i2,
  input  logic [$clog2(TILES_X)-1:0]    nb_j2,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i3,
  input  logic [$clog2(TILES_X)-1:0]    nb_j3,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i4,
  input  logic [$clog2(TILES_X)-1:0]    nb_j4,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i5,
  input  logic [$clog2(TILES_X)-1:0]    nb_j5,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i6,
  input  logic [$clog2(TILES_X)-1:0]    nb_j6,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i7,
  input  logic [$clog2(TILES_X)-1:0]    nb_j7,
  input  logic [$clog2(TILES_Y)-1:0]    nb_i8,
  input  logic [$clog2(TILES_X)-1:0]    nb_j8,

  input  logic [TAG_W-1:0]             nb_tag,

  // ----------------------------
  // Output single SRAM read req
  // ----------------------------
  output logic                         rd_valid,
  input  logic                         rd_ready,
  output logic [$clog2(TILES_Y)-1:0]    rd_tile_i,
  output logic [$clog2(TILES_X)-1:0]    rd_tile_j,
  output logic [TAG_W-1:0]             rd_tag,

  // ----------------------------
  // Side outputs
  // ----------------------------
  output logic                         sched_busy,
  output logic                         sched_done_pulse
);

  // ----------------------------
  // guard
  // ----------------------------
  localparam int unsigned IW = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW = (TILES_X <= 1) ? 1 : $clog2(TILES_X);

  wire go = en && !rst;

  // ----------------------------
  // internal bundle regs (1-deep)
  // ----------------------------
  logic                 busy;
  logic [IDX_W-1:0]      idx;          // 0..8
  wire                  rd_fire = rd_valid && rd_ready;

  // latched bundle
  logic [IW-1:0] bi0,bi1,bi2,bi3,bi4,bi5,bi6,bi7,bi8;
  logic [JW-1:0] bj0,bj1,bj2,bj3,bj4,bj5,bj6,bj7,bj8;
  logic [TAG_W-1:0] btag;

  // accept new bundle when not busy, OR when finishing last beat (allow back-to-back)
  wire finishing_last = busy && rd_fire && (idx == IDX_W'(8));
  always @* begin
    if (!go) nb_ready = 1'b0;
    else     nb_ready = (!busy) || finishing_last;
  end
  wire nb_fire = nb_valid && nb_ready;

  // ----------------------------
  // output mux based on idx
  // ----------------------------
  always @* begin
    rd_tile_i = '0;
    rd_tile_j = '0;

    if (busy) begin
      case (idx)
        0: begin rd_tile_i = bi0; rd_tile_j = bj0; end
        1: begin rd_tile_i = bi1; rd_tile_j = bj1; end
        2: begin rd_tile_i = bi2; rd_tile_j = bj2; end
        3: begin rd_tile_i = bi3; rd_tile_j = bj3; end
        4: begin rd_tile_i = bi4; rd_tile_j = bj4; end
        5: begin rd_tile_i = bi5; rd_tile_j = bj5; end
        6: begin rd_tile_i = bi6; rd_tile_j = bj6; end
        7: begin rd_tile_i = bi7; rd_tile_j = bj7; end
        default: begin rd_tile_i = bi8; rd_tile_j = bj8; end
      endcase
    end
  end

  // rd_valid asserted when busy
  always @* begin
    if (!go) rd_valid = 1'b0;
    else     rd_valid = busy;
  end

  // rd_tag packing: {upper bits of nb_tag, token_idx}
  // token_idx uses low IDX_W bits; keep group in upper bits
  // NOTE: if IDX_W=4 and TAG_W=16 => group bits = 12
  wire [TAG_W-1:0] tag_packed =
    (TAG_W > IDX_W) ? { btag[TAG_W-1:IDX_W], idx[IDX_W-1:0] } : idx[TAG_W-1:0];

  always @* begin
    rd_tag = tag_packed;
  end

  // side outputs
  always @* begin
    if (!go) sched_busy = 1'b0;
    else     sched_busy = busy;
  end

  // done pulse (1 cycle) when last request handshaked
  // NOTE: "done" means all 9 req have been ISSUED (accepted by SRAM read port)
  always @* begin
    if (!go) sched_done_pulse = 1'b0;
    else     sched_done_pulse = (busy && rd_fire && (idx == IDX_W'(8)));
  end

  // ----------------------------
  // state update
  // ----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
      idx  <= '0;

      bi0 <= '0; bj0 <= '0;
      bi1 <= '0; bj1 <= '0;
      bi2 <= '0; bj2 <= '0;
      bi3 <= '0; bj3 <= '0;
      bi4 <= '0; bj4 <= '0;
      bi5 <= '0; bj5 <= '0;
      bi6 <= '0; bj6 <= '0;
      bi7 <= '0; bj7 <= '0;
      bi8 <= '0; bj8 <= '0;
      btag <= '0;
    end else if (en) begin
      // default: keep state

      // If we issued a request, advance idx or end busy
      if (busy && rd_fire) begin
        if (idx == IDX_W'(8)) begin
          // finished all 9
          busy <= 1'b0;
          idx  <= '0;
        end else begin
          idx <= idx + IDX_W'(1);
        end
      end

      // Accept new bundle (can be back-to-back on finishing_last)
      if (nb_fire) begin
        // latch new bundle
        bi0 <= nb_i0; bj0 <= nb_j0;
        bi1 <= nb_i1; bj1 <= nb_j1;
        bi2 <= nb_i2; bj2 <= nb_j2;
        bi3 <= nb_i3; bj3 <= nb_j3;
        bi4 <= nb_i4; bj4 <= nb_j4;
        bi5 <= nb_i5; bj5 <= nb_j5;
        bi6 <= nb_i6; bj6 <= nb_j6;
        bi7 <= nb_i7; bj7 <= nb_j7;
        bi8 <= nb_i8; bj8 <= nb_j8;
        btag <= nb_tag;

        // start issuing from idx=0
        busy <= 1'b1;
        idx  <= '0;
      end
    end
  end

endmodule

`default_nettype wire
`endif
