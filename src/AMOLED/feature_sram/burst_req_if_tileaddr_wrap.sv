`ifndef BURST_REQ_IF_TILEADDR_WRAP_SV
`define BURST_REQ_IF_TILEADDR_WRAP_SV
`include "./src/AMOLED/feature_sram/tile_addr_gen.sv"
`include "./src/AMOLED/feature_sram/burst_req_if.sv"
`timescale 1ns/1ps
`default_nettype none

module burst_req_if_tileaddr_wrap #(
  parameter int unsigned tile_x   = 320,
  parameter int unsigned tile_y   = 180,
  parameter int unsigned sram_bus = 32,
  parameter int unsigned feat_dim = 8,
  parameter int unsigned elen_W   = 32,
  parameter int unsigned tag_w    = 16,
  parameter bit          isclamp  = 1'b0,

  parameter int unsigned FEAT_BITS_P      = feat_dim * elen_W,
  parameter int unsigned WORDS_PER_FEAT_P = (FEAT_BITS_P + sram_bus - 1) / sram_bus,
  parameter int unsigned TOTAL_WORDS_P    = tile_x * tile_y * WORDS_PER_FEAT_P,
  parameter int unsigned MEM_AW           = (TOTAL_WORDS_P <= 1) ? 1 : $clog2(TOTAL_WORDS_P)
)(
  input  logic clk,
  input  logic rst,

  // write req
  input  logic                    valid_wr,
  output logic                    ready_wr,
  input  logic [$clog2(tile_y)-1:0] tile_i_wr,
  input  logic [$clog2(tile_x)-1:0] tile_j_wr,
  input  logic [feat_dim*elen_W-1:0] feat_in,

  // read req
  input  logic                    valid_rd,
  output logic                    ready_rd,
  input  logic [$clog2(tile_y)-1:0] tile_i_rd,
  input  logic [$clog2(tile_x)-1:0] tile_j_rd,
  input  logic [tag_w-1:0]        tag_rd,

  // to burst_engine
  output logic                    req_valid,
  input  logic                    req_ready,
  output logic                    req_is_wr,
  output logic [MEM_AW-1:0]       sram_addr,
  output logic [feat_dim*elen_W-1:0] sram_wdata,
  output logic [tag_w-1:0]        sram_tag
);

  // ------------------------------------------------------------
  // 1) 先做 arbitration 選出「這一拍要吃的 tile_i/tile_j」
  //    （規則要跟 burst_req_if 一致：write priority, pending_valid 時不吃新的）
  // ------------------------------------------------------------
  logic pending_valid_mirror;
  logic take_wr, take_rd;

  // 這裡用與 burst_req_if 同樣條件推導「本拍是否會 take」
  // 注意：真正 pending_valid 在 burst_req_if 裡，但 wrapper 也要知道何時選地址
  // => 最簡單：直接用 burst_req_if 的 ready_wr/ready_rd 來判斷可否 take
  // ready_wr=~pending_valid, ready_rd=~pending_valid & ~valid_wr
  // 所以：
  always_comb begin
    take_wr = valid_wr && ready_wr;
    take_rd = valid_rd && ready_rd;
  end

  logic [$clog2(tile_y)-1:0] tile_i_sel;
  logic [$clog2(tile_x)-1:0] tile_j_sel;

  always_comb begin
    if (take_wr) begin
      tile_i_sel = tile_i_wr;
      tile_j_sel = tile_j_wr;
    end else begin
      tile_i_sel = tile_i_rd;
      tile_j_sel = tile_j_rd;
    end
  end

  // ------------------------------------------------------------
  // 2) tile_addr_gen instance：算 base_addr（word address）
  // ------------------------------------------------------------
  logic [$clog2(tile_x*tile_y)-1:0] tile_id_unused;
  logic [$clog2(tile_x*tile_y*WORDS_PER_FEAT_P)-1:0] base_addr_wide;

  tile_addr_gen #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W)
  ) u_addr (
    .tile_i(tile_i_sel),
    .tile_j(tile_j_sel),
    .tile_id(tile_id_unused),
    .sram_addr(base_addr_wide)
  );

  wire [MEM_AW-1:0] base_addr = base_addr_wide[MEM_AW-1:0];

  // ------------------------------------------------------------
  // 3) burst_req_if instance：用 ext_base_addr 覆蓋內建位址計算
  // ------------------------------------------------------------
  burst_req_if #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp),
    .FEAT_BITS_P(FEAT_BITS_P),
    .WORDS_PER_FEAT_P(WORDS_PER_FEAT_P),
    .TOTAL_WORDS_P(TOTAL_WORDS_P),
    .MEM_AW(MEM_AW)
  ) u_req (
    .clk(clk),
    .rst(rst),

    .valid_wr(valid_wr),
    .ready_wr(ready_wr),
    .tile_i_wr(tile_i_wr),
    .tile_j_wr(tile_j_wr),
    .feat_in(feat_in),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .sram_addr(sram_addr),
    .sram_wdata(sram_wdata),
    .sram_tag(sram_tag),

    .use_ext_addr(1'b1),
    .ext_base_addr(base_addr)
  );

endmodule

`default_nettype wire
`endif
