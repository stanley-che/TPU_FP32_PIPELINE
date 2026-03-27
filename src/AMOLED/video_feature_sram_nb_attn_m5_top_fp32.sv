`timescale 1ns/1ps
`default_nettype none

module video_sram_nb_attn_m5_top_fp32 #(
  // ----------------------------
  // Video + feature SRAM params
  // ----------------------------
  parameter integer X_W        = 11,
  parameter integer Y_W        = 10,

  parameter integer ACTIVE_W   = 1280,
  parameter integer ACTIVE_H   = 720,

  parameter integer TILE_SHIFT = 3,
  parameter integer TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter integer TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  parameter integer PACK_OUT_W = 10,
  parameter integer YPIX_W     = 8,

  parameter integer FEAT_DIM   = 8,
  parameter integer ELEN_W     = 32,
  parameter integer TAG_W      = 16,

  parameter DE_INV            = 1'b0,
  parameter USE_HSYNC_FOR_EOL = 1'b0,
  parameter GATE_BY_IN_FRAME  = 1'b0,
  parameter ENABLE_BOUNDS     = 1'b0,

  parameter ROI_EN_DEFAULT    = 1'b1,
  parameter integer ROI_X0_DEFAULT = 0,
  parameter integer ROI_Y0_DEFAULT = 0,
  parameter integer ROI_X1_DEFAULT = ACTIVE_W-1,
  parameter integer ROI_Y1_DEFAULT = ACTIVE_H-1,

  parameter USE_SOF  = 1'b1,
  parameter USE_EOL  = 1'b0,
  parameter USE_EOF  = 1'b0,

  parameter ENABLE_ASSERT   = 1'b0,
  parameter ASSERT_ON_STATS = 1'b0,
  parameter ASSERT_ON_EDGE  = 1'b0,

  // ----------------------------
  // Bundle9 + attention params
  // ----------------------------
  parameter integer TOKENS = 9,
  parameter integer D      = 8,

  parameter Q_FROM_CENTER_IDX4 = 1'b1,

  parameter integer M1_PIPE_STAGES = 2,
  parameter integer M2_PIPE_STAGES = 1,
  parameter integer M3_PIPE_STG    = 1,
  parameter integer M4_PIPE_STG    = 1,

  parameter integer W_PIPE_STAGES  = 1,
  parameter integer V_FIFO_DEPTH   = 16,

  parameter integer ALPHA_MODE     = 1,

  parameter TAG_EN                 = 1'b1,
  parameter integer TAG_FIFO_DEPTH = 32,

  // ----------------------------
  // Safety gating
  // ----------------------------
  parameter BLOCK_CENTER_WHEN_IN_FRAME = 1'b1
)(
  input  logic                         clk,
  input  logic                         rst_n,
  input  logic                         en,

  // ----------------------------
  // Video input
  // ----------------------------
  input  logic                         vsync_i,
  input  logic                         hsync_i,
  input  logic                         de_i,
  input  logic [23:0]                  rgb_i,

  input  logic                         clip_en,
  input  logic [PACK_OUT_W-1:0]        clip_min,
  input  logic [PACK_OUT_W-1:0]        clip_max,

  // ----------------------------
  // Center tile request
  // ----------------------------
  input  logic                         center_valid,
  output logic                         center_ready,
  input  logic [$clog2(TILES_Y)-1:0]   center_i,
  input  logic [$clog2(TILES_X)-1:0]   center_j,
  input  logic [TAG_W-1:0]             center_tag,

  // ----------------------------
  // Attention output
  // ----------------------------
  output logic                         out_valid,
  input  logic                         out_ready,
  output logic [31:0]                  alpha_fp32,
  output logic [D*32-1:0]              out_vec_dbg,
  output logic [TAG_W-1:0]             out_tag,

  // ----------------------------
  // Debug taps
  // ----------------------------
  output logic                         in_frame,
  output logic                         pix_valid_timing,
  output logic                         wr_fire,
  output logic [15:0]                  wr_tile_i,
  output logic [15:0]                  wr_tile_j,

  output logic                         err_mismatch_pulse,
  output logic [31:0]                  cnt_join_ok,
  output logic [31:0]                  cnt_mismatch,
  output logic [31:0]                  cnt_drop,

  output logic                         skid_drop_pulse,
  output logic [31:0]                  skid_drop_cnt
);

  localparam integer FEAT_W = FEAT_DIM * ELEN_W;
  localparam integer OUT_W  = D * 32;
  localparam integer IDX_W  = 4;

  wire rst = ~rst_n;
  wire go  = en & rst_n;

  // ---------------------------------------
  // Compile-time width guard (synth-safe)
  // ---------------------------------------
  generate
    if ((FEAT_DIM * ELEN_W) != (D * 32)) begin : g_width_mismatch
      INVALID_PARAMETER_COMBINATION__FEAT_W_MUST_EQUAL_D32 u_invalid ();
    end
    if (TOKENS != 9) begin : g_tokens_mismatch
      INVALID_PARAMETER_COMBINATION__TOKENS_MUST_BE_9 u_invalid ();
    end
  endgenerate

  // ============================================================
  // (1) Video -> feature SRAM integrated top
  // ============================================================
  logic                        valid_rd;
  logic                        ready_rd;
  logic [$clog2(TILES_Y)-1:0]  tile_i_rd;
  logic [$clog2(TILES_X)-1:0]  tile_j_rd;
  logic [TAG_W-1:0]            tag_rd;

  logic                        feat_out_valid;
  logic                        feat_out_ready;
  logic [TAG_W-1:0]            feat_out_tag;
  logic [FEAT_W-1:0]           feat_out_data;

  assign feat_out_ready = 1'b1;

  video_rgb2y_to_feature_sram_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(ELEN_W), .tag_w(TAG_W),
    .DE_INV(DE_INV),
    .USE_HSYNC_FOR_EOL(USE_HSYNC_FOR_EOL),
    .GATE_BY_IN_FRAME(GATE_BY_IN_FRAME),
    .ENABLE_BOUNDS(ENABLE_BOUNDS),
    .ROI_EN_DEFAULT(ROI_EN_DEFAULT),
    .ROI_X0_DEFAULT(ROI_X0_DEFAULT),
    .ROI_Y0_DEFAULT(ROI_Y0_DEFAULT),
    .ROI_X1_DEFAULT(ROI_X1_DEFAULT),
    .ROI_Y1_DEFAULT(ROI_Y1_DEFAULT),
    .USE_SOF(USE_SOF),
    .USE_EOL(USE_EOL),
    .USE_EOF(USE_EOF),
    .ENABLE_ASSERT(ENABLE_ASSERT),
    .ASSERT_ON_STATS(ASSERT_ON_STATS),
    .ASSERT_ON_EDGE(ASSERT_ON_EDGE)
  ) u_vid_feat (
    .clk(clk),
    .rst(rst),
    .en(en),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i(de_i),
    .rgb_i(rgb_i),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data),

    .wr_fire(wr_fire),
    .wr_tile_i(wr_tile_i),
    .wr_tile_j(wr_tile_j),

    .rgb2y_out_valid(),
    .rgb2y_out_ready(),
    .rgb2y_Y(),
    .rgb2y_sb_out(),

    .in_frame(in_frame),
    .in_line(),
    .pix_valid_timing(pix_valid_timing),
    .x(),
    .y(),

    .skid_drop_pulse(skid_drop_pulse),
    .skid_drop_cnt(skid_drop_cnt),
    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop)
  );

  // ============================================================
  // (2) Internal 3x3 bundle gather FSM
  // ============================================================
  function integer clamp_int;
    input integer v;
    input integer lo;
    input integer hi;
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  function [$clog2(TILES_Y)-1:0] calc_tile_i;
    input [$clog2(TILES_Y)-1:0] ci;
    input [3:0] token;
    integer tmp;
    begin
      case (token)
        4'd0,4'd1,4'd2: tmp = clamp_int(ci-1, 0, TILES_Y-1);
        4'd3,4'd4,4'd5: tmp = clamp_int(ci  , 0, TILES_Y-1);
        default       : tmp = clamp_int(ci+1, 0, TILES_Y-1);
      endcase
      calc_tile_i = tmp[$clog2(TILES_Y)-1:0];
    end
  endfunction

  function [$clog2(TILES_X)-1:0] calc_tile_j;
    input [$clog2(TILES_X)-1:0] cj;
    input [3:0] token;
    integer tmp;
    begin
      case (token)
        4'd0,4'd3,4'd6: tmp = clamp_int(cj-1, 0, TILES_X-1);
        4'd1,4'd4,4'd7: tmp = clamp_int(cj  , 0, TILES_X-1);
        default       : tmp = clamp_int(cj+1, 0, TILES_X-1);
      endcase
      calc_tile_j = tmp[$clog2(TILES_X)-1:0];
    end
  endfunction

  logic [FEAT_W-1:0] kv [0:8];
  logic [3:0]        tok_idx;
  logic              gathering;
  logic [TAG_W-1:0]  center_tag_latched;
  logic [$clog2(TILES_Y)-1:0] center_i_latched;
  logic [$clog2(TILES_X)-1:0] center_j_latched;

  logic              b_valid;
  logic              b_ready;

  logic [TOKENS*D*32-1:0] k_vecs_attn;
  logic [TOKENS*D*32-1:0] v_vecs_attn;
  logic [D*32-1:0]        q_vec_attn;
  logic [TAG_W-1:0]       in_tag_attn;

  wire allow_center = go & (~BLOCK_CENTER_WHEN_IN_FRAME | (in_frame == 1'b0));
  assign center_ready = allow_center & ~gathering & ~b_valid;

  localparam [1:0] S_IDLE = 2'd0;
  localparam [1:0] S_REQ  = 2'd1;
  localparam [1:0] S_WAIT = 2'd2;
  localparam [1:0] S_PACK = 2'd3;

  logic [1:0] state;
  integer t;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state <= S_IDLE;
      valid_rd <= 1'b0;
      tile_i_rd <= '0;
      tile_j_rd <= '0;
      tag_rd <= '0;
      gathering <= 1'b0;
      tok_idx <= 4'd0;
      b_valid <= 1'b0;
      center_tag_latched <= '0;
      center_i_latched <= '0;
      center_j_latched <= '0;
      k_vecs_attn <= '0;
      v_vecs_attn <= '0;
      q_vec_attn <= '0;
      in_tag_attn <= '0;
      for (t = 0; t < 9; t = t + 1)
        kv[t] <= '0;
    end else if (en) begin
      if (!go) begin
        valid_rd  <= 1'b0;
        b_valid   <= 1'b0;
        gathering <= 1'b0;
        state     <= S_IDLE;
      end else begin
        case (state)
          S_IDLE: begin
            valid_rd  <= 1'b0;
            gathering <= 1'b0;
            if (center_valid && center_ready) begin
              center_tag_latched <= center_tag;
              center_i_latched   <= center_i;
              center_j_latched   <= center_j;
              tok_idx            <= 4'd0;
              gathering          <= 1'b1;
              state              <= S_REQ;
            end
          end

          S_REQ: begin
            tile_i_rd <= calc_tile_i(center_i_latched, tok_idx);
            tile_j_rd <= calc_tile_j(center_j_latched, tok_idx);
            tag_rd    <= {center_tag_latched[TAG_W-1:IDX_W], tok_idx[IDX_W-1:0]};
            valid_rd  <= 1'b1;

            if (valid_rd && ready_rd) begin
              valid_rd <= 1'b0;
              state    <= S_WAIT;
            end
          end

          S_WAIT: begin
            if (feat_out_valid) begin
              kv[tok_idx] <= feat_out_data;
              if (tok_idx == 4'd8) begin
                state <= S_PACK;
              end else begin
                tok_idx <= tok_idx + 4'd1;
                state   <= S_REQ;
              end
            end
          end

          S_PACK: begin
            for (t = 0; t < 9; t = t + 1) begin
              k_vecs_attn[t*D*32 +: D*32] <= kv[t][D*32-1:0];
              v_vecs_attn[t*D*32 +: D*32] <= kv[t][D*32-1:0];
            end

            q_vec_attn <= kv[4][D*32-1:0];
            in_tag_attn <= {center_tag_latched[TAG_W-1:IDX_W], {IDX_W{1'b0}}};

            b_valid   <= 1'b1;
            gathering <= 1'b0;
            state     <= S_IDLE;
          end

          default: begin
            state <= S_IDLE;
          end
        endcase

        if (b_valid && b_ready)
          b_valid <= 1'b0;
      end
    end
  end

  // ============================================================
  // (3) Attention core
  // ============================================================
  // 不再 override real parameters
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
    .in_ready(b_ready),

    .q_vec(q_vec_attn),
    .k_vecs(k_vecs_attn),
    .v_vecs(v_vecs_attn),
    .in_tag(in_tag_attn),

    .out_valid(out_valid),
    .out_ready(out_ready),

    .alpha_fp32(alpha_fp32),
    .out_vec_dbg(out_vec_dbg),
    .out_tag(out_tag),

    .score_flat_dbg(),
    .w_flat_dbg()
  );

endmodule

`default_nettype wire
