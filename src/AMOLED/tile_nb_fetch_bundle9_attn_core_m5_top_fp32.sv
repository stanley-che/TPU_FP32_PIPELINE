// ============================================================
// tile_nb_fetch_bundle9_attn_core_m5_top_fp32.sv  (IVERILOG-SAFE)
// - One-module integration:
//
//   (A) tile_nb_feature_fetch_3x3_bundle9_top_inst : center -> {kv0..kv8,q_vec,out_group_tag}
//   (B) attn_core_tile_top_fp32                    : (Q,K,V,tag) -> alpha_fp32 (+ out_tag)
//
// Assumption:
// - FEAT_W == D*32  (e.g. 256 == 8*32), so each kvX is a FP32 vector of length D.
// - K and V both use kv tokens (typical "KV" feature). If you want V different, change mapping.
//
// Notes:
// - Fetch side uses rst (active-high) + en.
// - Attn side uses rst_n (active-low).
// - This top derives rst = ~rst_n and gates center_valid/out_ready with en.
// ============================================================
`include "./src/AMOLED/tile_neighborhood_fetch/tile_nb_feature_fetch_3x3_bundle9_top_inst.sv"
`include "./src/AMOLED/atten_core/attn_core_tile_top_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tile_nb_fetch_bundle9_attn_core_m5_top_fp32 #(
  // ----------------------------
  // Neighborhood fetch params
  // ----------------------------
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,

  parameter int unsigned TAG_W   = 16,
  parameter int unsigned IDX_W   = 4,

  parameter int unsigned FEAT_W         = 256,
  parameter int unsigned MEM_W          = 64,
  parameter int unsigned BASE_ADDR_WORD = 0,

  parameter int unsigned BANKS          = 1,
  parameter int unsigned ADDR_W         = 32,
  parameter int unsigned READ_LATENCY   = 1,
  parameter bit          USE_RTAG       = 0,

  parameter bit          Q_FROM_CENTER_IDX4 = 1'b1,

  // ----------------------------
  // Attention (M5) params
  // ----------------------------
  parameter int unsigned TOKENS          = 9,
  parameter int unsigned D               = 8,

  parameter int unsigned M1_PIPE_STAGES  = 2,
  parameter int unsigned M2_PIPE_STAGES  = 1,
  parameter int unsigned M3_PIPE_STG     = 1,
  parameter int unsigned M4_PIPE_STG     = 1,

  parameter int unsigned W_PIPE_STAGES   = 1,
  parameter int unsigned V_FIFO_DEPTH    = 16,

  parameter int unsigned ALPHA_MODE      = 1,
  parameter real         ALPHA_SCALE     = 1.0,
  parameter real         ALPHA_BIAS      = 0.0,
  parameter real         ALPHA_SIG_A     = 1.0,

  // Tag pass-through inside M5
  parameter bit          TAG_EN          = 1'b1,
  parameter int unsigned TAG_FIFO_DEPTH  = 32
)(
  input  wire                         clk,
  input  wire                         rst_n,
  input  wire                         en,

  // ----------------------------
  // Center tile request
  // ----------------------------
  input  wire                         center_valid,
  output wire                         center_ready,
  input  wire [$clog2(TILES_Y)-1:0]    center_i,
  input  wire [$clog2(TILES_X)-1:0]    center_j,
  input  wire [TAG_W-1:0]             center_tag,

  // ----------------------------
  // SRAM read command (per beat)
  // ----------------------------
  output wire                         mem_rd_valid,
  input  wire                         mem_rd_ready,
  output wire [ADDR_W-1:0]            mem_addr,
  output wire [((BANKS<=1)?1:$clog2(BANKS))-1:0] mem_bank,
  output wire [TAG_W-1:0]             mem_tag,

  // ----------------------------
  // SRAM read return
  // ----------------------------
  input  wire                         mem_rvalid,
  input  wire [MEM_W-1:0]             mem_rdata,
  input  wire [TAG_W-1:0]             mem_rtag,

  // ----------------------------
  // Final output (M5 alpha)
  // ----------------------------
  output wire                         out_valid,
  input  wire                         out_ready,
  output wire [31:0]                  alpha_fp32,
  output wire [D*32-1:0]              out_vec_dbg,
  output wire [TAG_W-1:0]             out_tag,

  // ----------------------------
  // Debug (optional)
  // ----------------------------
  output wire                         sched_busy,
  output wire                         sched_done_pulse,
  output wire                         bundle_done_pulse,
  output wire [8:0]                   nb_is_center,

  output wire [TOKENS*32-1:0]         score_flat_dbg,
  output wire [TOKENS*32-1:0]         w_flat_dbg
);

  // ----------------------------
  // Reset adaptation
  // ----------------------------
  wire rst = ~rst_n;
  wire go  = en && rst_n;

  // ----------------------------
  // Fetch + bundle outputs
  // ----------------------------
  wire                 b_valid;
  wire                 b_ready;

  wire [FEAT_W-1:0]     kv0,kv1,kv2,kv3,kv4,kv5,kv6,kv7,kv8;
  wire [9*FEAT_W-1:0]   kv_bus;        // waveform only
  wire [FEAT_W-1:0]     q_feat_vec;    // FEAT_W (=D*32)
  wire [TAG_W-1:IDX_W]  out_group_tag;

  // Gate center_valid into fetch by en (optional safety)
  wire center_valid_g = go && center_valid;

  tile_nb_feature_fetch_3x3_bundle9_top_inst #(
    .TILES_X(TILES_X),
    .TILES_Y(TILES_Y),
    .TAG_W(TAG_W),
    .IDX_W(IDX_W),
    .FEAT_W(FEAT_W),
    .MEM_W(MEM_W),
    .BASE_ADDR_WORD(BASE_ADDR_WORD),
    .BANKS(BANKS),
    .ADDR_W(ADDR_W),
    .READ_LATENCY(READ_LATENCY),
    .USE_RTAG(USE_RTAG),
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4)
  ) u_fetch_bundle (
    .clk(clk),
    .rst(rst),
    .en (en),

    .center_valid(center_valid_g),
    .center_ready(center_ready),
    .center_i(center_i),
    .center_j(center_j),
    .center_tag(center_tag),

    .mem_rd_valid(mem_rd_valid),
    .mem_rd_ready(mem_rd_ready),
    .mem_addr(mem_addr),
    .mem_bank(mem_bank),
    .mem_tag(mem_tag),

    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata),
    .mem_rtag(mem_rtag),

    .out_valid(b_valid),
    .out_ready(b_ready),

    .kv0(kv0), .kv1(kv1), .kv2(kv2), .kv3(kv3), .kv4(kv4),
    .kv5(kv5), .kv6(kv6), .kv7(kv7), .kv8(kv8),

    .kv_bus(kv_bus),
    .q_vec(q_feat_vec),
    .out_group_tag(out_group_tag),

    .sched_busy(sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .bundle_done_pulse(bundle_done_pulse),
    .nb_is_center(nb_is_center)
  );

  // ----------------------------
  // Map FEAT vectors -> Attention Q/K/V
  // ----------------------------
  // Expect FEAT_W == D*32
  wire [D*32-1:0] q_vec_attn = q_feat_vec[D*32-1:0];

  wire [TOKENS*D*32-1:0] k_vecs_attn;
  wire [TOKENS*D*32-1:0] v_vecs_attn;

  // token0 in LSB ... token8 in MSB
  assign k_vecs_attn[0*D*32 +: D*32] = kv0[D*32-1:0];
  assign k_vecs_attn[1*D*32 +: D*32] = kv1[D*32-1:0];
  assign k_vecs_attn[2*D*32 +: D*32] = kv2[D*32-1:0];
  assign k_vecs_attn[3*D*32 +: D*32] = kv3[D*32-1:0];
  assign k_vecs_attn[4*D*32 +: D*32] = kv4[D*32-1:0];
  assign k_vecs_attn[5*D*32 +: D*32] = kv5[D*32-1:0];
  assign k_vecs_attn[6*D*32 +: D*32] = kv6[D*32-1:0];
  assign k_vecs_attn[7*D*32 +: D*32] = kv7[D*32-1:0];
  assign k_vecs_attn[8*D*32 +: D*32] = kv8[D*32-1:0];

  // Use same kv as V by default (KV feature). Change here if needed.
  assign v_vecs_attn = k_vecs_attn;

  // Tag into M5: use group tag (shift back IDX_W zeros)
  wire [TAG_W-1:0] in_tag_attn = {out_group_tag, {IDX_W{1'b0}}};

  // ----------------------------
  // Connect bundle handshake -> attn handshake
  // ----------------------------
  // Bundle out_valid/out_ready connect to attn in_valid/in_ready
  wire attn_in_ready;
  assign b_ready = attn_in_ready;

  // ----------------------------
  // M5 Attention core
  // ----------------------------
  attn_core_tile_top_fp32 #(
    .TOKENS         (TOKENS),
    .D              (D),

    .M1_PIPE_STAGES (M1_PIPE_STAGES),
    .M2_PIPE_STAGES (M2_PIPE_STAGES),
    .M3_PIPE_STG    (M3_PIPE_STG),
    .M4_PIPE_STG    (M4_PIPE_STG),

    .W_PIPE_STAGES  (W_PIPE_STAGES),
    .V_FIFO_DEPTH   (V_FIFO_DEPTH),

    .ALPHA_MODE     (ALPHA_MODE),
    .ALPHA_SCALE    (ALPHA_SCALE),
    .ALPHA_BIAS     (ALPHA_BIAS),
    .ALPHA_SIG_A    (ALPHA_SIG_A),

    .TAG_EN         (TAG_EN),
    .TAG_W          (TAG_W),
    .TAG_FIFO_DEPTH (TAG_FIFO_DEPTH)
  ) u_m5 (
    .clk(clk),
    .rst_n(rst_n),

    .in_valid(b_valid),
    .in_ready(attn_in_ready),

    .q_vec(q_vec_attn),
    .k_vecs(k_vecs_attn),
    .v_vecs(v_vecs_attn),
    .in_tag(in_tag_attn),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .alpha_fp32(alpha_fp32),
    .out_vec_dbg(out_vec_dbg),
    .out_tag(out_tag),

    .score_flat_dbg(score_flat_dbg),
    .w_flat_dbg(w_flat_dbg)
  );

endmodule

`default_nettype wire
