`timescale 1ns/1ps
`default_nettype none

module tile_nb_fetch_bundle9_attn_core_m5_top_fp32 #(
  // ----------------------------
  // Neighborhood fetch params
  // ----------------------------
  parameter integer TILES_X = 320,
  parameter integer TILES_Y = 180,

  parameter integer TAG_W   = 16,
  parameter integer IDX_W   = 4,

  parameter integer FEAT_W         = 256,
  parameter integer MEM_W          = 64,
  parameter integer BASE_ADDR_WORD = 0,

  parameter integer BANKS          = 1,
  parameter integer ADDR_W         = 32,
  parameter integer READ_LATENCY   = 1,
  parameter integer USE_RTAG       = 0,

  parameter integer Q_FROM_CENTER_IDX4 = 1,

  // ----------------------------
  // Attention (M5) params
  // ----------------------------
  parameter integer TOKENS         = 9,
  parameter integer D              = 8,

  parameter integer M1_PIPE_STAGES = 2,
  parameter integer M2_PIPE_STAGES = 1,
  parameter integer M3_PIPE_STG    = 1,
  parameter integer M4_PIPE_STG    = 1,

  parameter integer W_PIPE_STAGES  = 1,
  parameter integer V_FIFO_DEPTH   = 16,

  parameter integer ALPHA_MODE     = 1,

  // Tag pass-through inside M5
  parameter integer TAG_EN         = 1,
  parameter integer TAG_FIFO_DEPTH = 32
)(
  input  logic clk,
  input  logic rst_n,
  input  logic en,

  // ----------------------------
  // Center tile request
  // ----------------------------
  input  logic                        center_valid,
  output logic                        center_ready,
  input  logic [$clog2(TILES_Y)-1:0]  center_i,
  input  logic [$clog2(TILES_X)-1:0]  center_j,
  input  logic [TAG_W-1:0]            center_tag,

  // ----------------------------
  // SRAM read command (per beat)
  // ----------------------------
  output logic                        mem_rd_valid,
  input  logic                        mem_rd_ready,
  output logic [ADDR_W-1:0]           mem_addr,
  output logic [((BANKS<=1)?1:$clog2(BANKS))-1:0] mem_bank,
  output logic [TAG_W-1:0]            mem_tag,

  // ----------------------------
  // SRAM read return
  // ----------------------------
  input  logic                        mem_rvalid,
  input  logic [MEM_W-1:0]            mem_rdata,
  input  logic [TAG_W-1:0]            mem_rtag,

  // ----------------------------
  // Final output (M5 alpha)
  // ----------------------------
  output logic                        out_valid,
  input  logic                        out_ready,
  output logic [31:0]                 alpha_fp32,
  output logic [D*32-1:0]             out_vec_dbg,
  output logic [TAG_W-1:0]            out_tag,

  // ----------------------------
  // Debug (optional)
  // ----------------------------
  output logic                        sched_busy,
  output logic                        sched_done_pulse,
  output logic                        bundle_done_pulse,
  output logic [8:0]                  nb_is_center,

  output logic [TOKENS*32-1:0]        score_flat_dbg,
  output logic [TOKENS*32-1:0]        w_flat_dbg
);

  // ----------------------------
  // Reset adaptation
  // ----------------------------
  logic rst;
  logic go;

  assign rst = ~rst_n;
  assign go  = en & rst_n;

  // ----------------------------
  // Fetch + bundle outputs
  // ----------------------------
  logic               b_valid;
  logic               b_ready;

  logic [FEAT_W-1:0]  kv0, kv1, kv2, kv3, kv4, kv5, kv6, kv7, kv8;
  logic [9*FEAT_W-1:0] kv_bus;
  logic [FEAT_W-1:0]   q_feat_vec;
  logic [TAG_W-IDX_W-1:0] out_group_tag;

  logic center_valid_g;
  assign center_valid_g = go & center_valid;

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
    .en(en),

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
  logic [D*32-1:0] q_vec_attn;
  logic [TOKENS*D*32-1:0] k_vecs_attn;
  logic [TOKENS*D*32-1:0] v_vecs_attn;
  logic [TAG_W-1:0] in_tag_attn;
  logic attn_in_ready;

  assign q_vec_attn = q_feat_vec[D*32-1:0];

  assign k_vecs_attn[0*D*32 +: D*32] = kv0[D*32-1:0];
  assign k_vecs_attn[1*D*32 +: D*32] = kv1[D*32-1:0];
  assign k_vecs_attn[2*D*32 +: D*32] = kv2[D*32-1:0];
  assign k_vecs_attn[3*D*32 +: D*32] = kv3[D*32-1:0];
  assign k_vecs_attn[4*D*32 +: D*32] = kv4[D*32-1:0];
  assign k_vecs_attn[5*D*32 +: D*32] = kv5[D*32-1:0];
  assign k_vecs_attn[6*D*32 +: D*32] = kv6[D*32-1:0];
  assign k_vecs_attn[7*D*32 +: D*32] = kv7[D*32-1:0];
  assign k_vecs_attn[8*D*32 +: D*32] = kv8[D*32-1:0];

  assign v_vecs_attn = k_vecs_attn;

  assign in_tag_attn = {out_group_tag, {IDX_W{1'b0}}};

  assign b_ready = attn_in_ready;

  // ----------------------------
  // M5 Attention core
  // ----------------------------
  // 注意：
  // 這裡故意不再 override real parameter
  // 讓 child module 用它自己的 default 值
  attn_core_tile_top_fp32 #(
    .TOKENS(TOKENS),
    .D(D),
    .M1_PIPE_STAGES(M1_PIPE_STAGES),
    .M2_PIPE_STAGES(M2_PIPE_STAGES),
    .M3_PIPE_STG(M3_PIPE_STG),
    .M4_PIPE_STG(M4_PIPE_STG),
    .W_PIPE_STAGES(W_PIPE_STAGES),
    .V_FIFO_DEPTH(V_FIFO_DEPTH),
    .ALPHA_MODE(ALPHA_MODE),
    .TAG_EN(TAG_EN),
    .TAG_W(TAG_W),
    .TAG_FIFO_DEPTH(TAG_FIFO_DEPTH)
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
