// ============================================================
// video_rgb2y_feature_sram_nb_attn_m5_top_fp32.sv  (IVERILOG-SAFE)
// - Integrates:
//   (1) video_rgb2y_to_feature_sram_top  : video -> per-tile feature SRAM + tile read port
//   (2) attn_core_tile_top_fp32          : (Q,K,V,tag) -> alpha_fp32 (+ out_tag)
//
// - This top implements an internal 3x3 (bundle9) fetch FSM using the tile-read port
//   of video_rgb2y_to_feature_sram_top (NOT the external SRAM beat interface).
//
// Assumptions:
// - FEAT_DIM*elen_W == D*32 (e.g. 8*32=256) for attention input vector width.
// - Q is taken from center token (idx4) by default (like your integrated M5 top).
//
// Notes:
// - You SHOULD avoid issuing reads while the frame is being written (in_frame==1).
//   This top gates center_ready by !in_frame by default (can override with parameter).
// ============================================================

`include "./src/AMOLED/video_rgb2y_to_feature_sram_top.sv"
`include "./src/AMOLED/atten_core/attn_core_tile_top_fp32.sv"

`timescale 1ns/1ps
`default_nettype none

module video_sram_nb_attn_m5_top_fp32 #(
  // ----------------------------
  // Video + feature SRAM params
  // ----------------------------
  parameter int unsigned X_W        = 11,
  parameter int unsigned Y_W        = 10,

  parameter int unsigned ACTIVE_W   = 1280,
  parameter int unsigned ACTIVE_H   = 720,

  parameter int unsigned TILE_SHIFT = 3,   // 8x8 tiles by default
  parameter int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT),
  parameter int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT),

  parameter int unsigned PACK_OUT_W = 10,
  parameter int unsigned YPIX_W     = 8,

  parameter int unsigned FEAT_DIM   = 8,
  parameter int unsigned elen_W     = 32,
  parameter int unsigned tag_w      = 16,

  // These match your DUT variants knobs
  parameter bit DE_INV            = 1'b0,
  parameter bit USE_HSYNC_FOR_EOL = 1'b0,
  parameter bit GATE_BY_IN_FRAME  = 1'b0,
  parameter bit ENABLE_BOUNDS     = 1'b0,

  parameter bit ROI_EN_DEFAULT    = 1'b1,
  parameter int unsigned ROI_X0_DEFAULT = 0,
  parameter int unsigned ROI_Y0_DEFAULT = 0,
  parameter int unsigned ROI_X1_DEFAULT = ACTIVE_W-1,
  parameter int unsigned ROI_Y1_DEFAULT = ACTIVE_H-1,

  parameter bit USE_SOF  = 1'b1,
  parameter bit USE_EOL  = 1'b0,
  parameter bit USE_EOF  = 1'b0,

  parameter bit ENABLE_ASSERT    = 1'b0,
  parameter bit ASSERT_ON_STATS  = 1'b0,
  parameter bit ASSERT_ON_EDGE   = 1'b0,

  // ----------------------------
  // Bundle9 + attention params
  // ----------------------------
  parameter int unsigned TOKENS = 9,
  parameter int unsigned D      = 8,  // must satisfy D*32 == FEAT_DIM*elen_W

  parameter bit Q_FROM_CENTER_IDX4 = 1'b1,

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

  parameter bit          TAG_EN          = 1'b1,
  parameter int unsigned TAG_FIFO_DEPTH  = 32,

  // ----------------------------
  // Safety gating
  // ----------------------------
  parameter bit          BLOCK_CENTER_WHEN_IN_FRAME = 1'b1
)(
  input  wire                         clk,
  input  wire                         rst_n,
  input  wire                         en,

  // ----------------------------
  // Video input
  // ----------------------------
  input  wire                         vsync_i,
  input  wire                         hsync_i,
  input  wire                         de_i,
  input  wire [23:0]                  rgb_i,

  input  wire                         clip_en,
  input  wire [PACK_OUT_W-1:0]        clip_min,
  input  wire [PACK_OUT_W-1:0]        clip_max,

  // ----------------------------
  // Center tile request (one transaction => outputs one alpha)
  // ----------------------------
  input  wire                         center_valid,
  output wire                         center_ready,
  input  wire [$clog2(TILES_Y)-1:0]    center_i,
  input  wire [$clog2(TILES_X)-1:0]    center_j,
  input  wire [tag_w-1:0]             center_tag,

  // ----------------------------
  // Attention output
  // ----------------------------
  output wire                         out_valid,
  input  wire                         out_ready,
  output wire [31:0]                  alpha_fp32,
  output wire [D*32-1:0]              out_vec_dbg,
  output wire [tag_w-1:0]             out_tag,

  // ----------------------------
  // Debug taps (from video feature writer)
  // ----------------------------
  output wire                         in_frame,
  output wire                         pix_valid_timing,
  output wire                         wr_fire,
  output wire [15:0]                  wr_tile_i,
  output wire [15:0]                  wr_tile_j,

  output wire                         err_mismatch_pulse,
  output wire [31:0]                  cnt_join_ok,
  output wire [31:0]                  cnt_mismatch,
  output wire [31:0]                  cnt_drop,

  output wire                         skid_drop_pulse,
  output wire [31:0]                  skid_drop_cnt
);

  // ----------------------------
  // Local sanity: widths
  // ----------------------------
  localparam int unsigned FEAT_W = FEAT_DIM*elen_W;
  localparam int unsigned OUT_W  = D*32;
  localparam int unsigned IDX_W  = 4; // token idx width for bundle9 tags

  // IVERILOG-friendly elaboration guard (won't stop compile, but helps debug)
  initial begin
    if (FEAT_W != OUT_W) begin
      $display("[FATAL] FEAT_W(%0d) != D*32(%0d). Set D or FEAT_DIM/elen_W to match.", FEAT_W, OUT_W);
      $fatal(1);
    end
    if (TOKENS != 9) begin
      $display("[FATAL] This top expects TOKENS=9 for 3x3 bundle9.");
      $fatal(1);
    end
  end

  // ----------------------------
  // Reset adapt
  // ----------------------------
  wire rst = ~rst_n;
  wire go  = en && rst_n;

  // ============================================================
  // (1) Video -> feature SRAM integrated top
  // ============================================================
  // tile read port (we will drive)
  reg                        valid_rd;
  wire                       ready_rd;
  reg  [$clog2(TILES_Y)-1:0] tile_i_rd;
  reg  [$clog2(TILES_X)-1:0] tile_j_rd;
  reg  [tag_w-1:0]           tag_rd;

  // feature read return
  wire                       feat_out_valid;
  wire                       feat_out_ready;
  wire [tag_w-1:0]           feat_out_tag;
  wire [FEAT_W-1:0]          feat_out_data;

  assign feat_out_ready = 1'b1; // always consume

  // instantiate one variant (you can parameterize DE_INV/HSYNC/EOL etc.)
  video_rgb2y_to_feature_sram_top #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(elen_W), .tag_w(tag_w),

    .DE_INV(DE_INV),
    .USE_HSYNC_FOR_EOL(USE_HSYNC_FOR_EOL),
    .GATE_BY_IN_FRAME(GATE_BY_IN_FRAME),
    .ENABLE_BOUNDS(ENABLE_BOUNDS),

    .ROI_EN_DEFAULT(ROI_EN_DEFAULT),
    .ROI_X0_DEFAULT(ROI_X0_DEFAULT), .ROI_Y0_DEFAULT(ROI_Y0_DEFAULT),
    .ROI_X1_DEFAULT(ROI_X1_DEFAULT), .ROI_Y1_DEFAULT(ROI_Y1_DEFAULT),

    .USE_SOF(USE_SOF), .USE_EOL(USE_EOL), .USE_EOF(USE_EOF),
    .ENABLE_ASSERT(ENABLE_ASSERT), .ASSERT_ON_STATS(ASSERT_ON_STATS), .ASSERT_ON_EDGE(ASSERT_ON_EDGE)
  ) u_vid_feat (
    .clk(clk), .rst(rst), .en(en),

    .vsync_i(vsync_i), .hsync_i(hsync_i), .de_i(de_i), .rgb_i(rgb_i),
    .clip_en(clip_en), .clip_min(clip_min), .clip_max(clip_max),

    .valid_rd(valid_rd), .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd), .tile_j_rd(tile_j_rd), .tag_rd(tag_rd),

    .feat_out_valid(feat_out_valid), .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag), .feat_out_data(feat_out_data),

    .wr_fire(wr_fire), .wr_tile_i(wr_tile_i), .wr_tile_j(wr_tile_j),

    // pass-through debug
    .rgb2y_out_valid(), .rgb2y_out_ready(), .rgb2y_Y(), .rgb2y_sb_out(),
    .in_frame(in_frame), .in_line(), .pix_valid_timing(pix_valid_timing),
    .x(), .y(),

    .skid_drop_pulse(skid_drop_pulse), .skid_drop_cnt(skid_drop_cnt),
    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok), .cnt_mismatch(cnt_mismatch), .cnt_drop(cnt_drop)
  );

  // ============================================================
  // (2) Internal 3x3 bundle9 gather FSM (using tile read port)
  // ============================================================
  // neighbor mapping token order:
  // 0:(-1,-1) 1:(-1,0) 2:(-1,+1)
  // 3:(0,-1)  4:(0,0)  5:(0,+1)
  // 6:(+1,-1) 7:(+1,0) 8:(+1,+1)
  function automatic int clamp_int(input int v, input int lo, input int hi);
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  task automatic token_to_tile(
    input int ci, input int cj, input int token,
    output int ti, output int tj
  );
    begin
      case (token)
        0: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        1: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        2: begin ti = clamp_int(ci-1, 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
        3: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        4: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        5: begin ti = clamp_int(ci  , 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
        6: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj-1, 0, TILES_X-1); end
        7: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj  , 0, TILES_X-1); end
        default: begin ti = clamp_int(ci+1, 0, TILES_Y-1); tj = clamp_int(cj+1, 0, TILES_X-1); end
      endcase
    end
  endtask

  // storage for 9 tokens
  reg [FEAT_W-1:0] kv [0:8];
  reg [3:0]        tok_idx;
  reg              gathering;
  reg [tag_w-1:0]  center_tag_latched;
  reg [$clog2(TILES_Y)-1:0] center_i_latched;
  reg [$clog2(TILES_X)-1:0] center_j_latched;

  // once gathered, we assert b_valid to attention core
  reg              b_valid;
  wire             b_ready;

  // pack into k_vecs/v_vecs/q_vec
  reg [TOKENS*D*32-1:0] k_vecs_attn;
  reg [TOKENS*D*32-1:0] v_vecs_attn;
  reg [D*32-1:0]        q_vec_attn;
  reg [tag_w-1:0]       in_tag_attn;

  // center_ready policy
  wire allow_center = go && (!BLOCK_CENTER_WHEN_IN_FRAME || (in_frame == 1'b0));
  assign center_ready = allow_center && !gathering && !b_valid; // only accept new when idle

  // drive tile read request handshake:
  // - we issue one read per token, wait for ready_rd to accept request,
  //   then wait for feat_out_valid to return data, store it, move next token.
  localparam int S_IDLE   = 0;
  localparam int S_REQ    = 1;
  localparam int S_WAIT   = 2;
  localparam int S_PACK   = 3;
  reg [1:0] state;

  integer t;
  int ti, tj;

  always @(posedge clk) begin
    if (rst) begin
      state <= S_IDLE;
      valid_rd <= 1'b0;
      tile_i_rd <= '0;
      tile_j_rd <= '0;
      tag_rd    <= '0;

      gathering <= 1'b0;
      tok_idx   <= 4'd0;
      b_valid   <= 1'b0;

      center_tag_latched <= '0;
      center_i_latched   <= '0;
      center_j_latched   <= '0;

      for (t=0;t<9;t=t+1) kv[t] <= '0;
    end else if (en) begin
      // default holds
      if (!go) begin
        // if en drops, return to safe idle-ish (optional)
        valid_rd   <= 1'b0;
        b_valid    <= 1'b0;
        gathering  <= 1'b0;
        state      <= S_IDLE;
      end else begin
        case (state)
          S_IDLE: begin
            valid_rd  <= 1'b0;
            gathering <= 1'b0;

            if (center_valid && center_ready) begin
              // latch center request
              center_tag_latched <= center_tag;
              center_i_latched   <= center_i;
              center_j_latched   <= center_j;

              tok_idx   <= 4'd0;
              gathering <= 1'b1;
              state     <= S_REQ;
            end
          end

          S_REQ: begin
            // setup tile coords for current token
            token_to_tile(center_i_latched, center_j_latched, tok_idx, ti, tj);

            tile_i_rd <= ti[$clog2(TILES_Y)-1:0];
            tile_j_rd <= tj[$clog2(TILES_X)-1:0];

            tag_rd    <= {center_tag_latched[tag_w-1:IDX_W], tok_idx[IDX_W-1:0]};

            // keep valid asserted until we see a registered handshake
            valid_rd <= 1'b1;

            // IMPORTANT: only treat as accepted when valid_rd was already high
            if (valid_rd && ready_rd) begin
                valid_rd <= 1'b0;
                state    <= S_WAIT;
            end
          end


          S_WAIT: begin
            // wait for data return
            if (feat_out_valid) begin
              // store into kv[tok_idx]
              kv[tok_idx] <= feat_out_data;

              if (tok_idx == 4'd8) begin
                state <= S_PACK;
              end else begin
                tok_idx <= tok_idx + 1;
                state   <= S_REQ;
              end
            end
          end

          S_PACK: begin
            // build packed buses for attention
            // token0 in LSB ... token8 in MSB
            for (t=0; t<9; t=t+1) begin
              k_vecs_attn[t*D*32 +: D*32] <= kv[t][D*32-1:0];
              v_vecs_attn[t*D*32 +: D*32] <= kv[t][D*32-1:0];
            end

            if (Q_FROM_CENTER_IDX4) q_vec_attn <= kv[4][D*32-1:0];
            else                    q_vec_attn <= kv[4][D*32-1:0]; // fallback (can change)

            in_tag_attn <= {center_tag_latched[tag_w-1:IDX_W], {IDX_W{1'b0}}};

            b_valid   <= 1'b1;
            gathering <= 1'b0;
            state     <= S_IDLE;
          end

          default: begin
            state <= S_IDLE;
          end
        endcase

        // handshake into attention core
        if (b_valid && b_ready) begin
          b_valid <= 1'b0;
        end
      end
    end
  end

  // ============================================================
  // (3) Attention core (M5)
  // ============================================================
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
    .TAG_W          (tag_w),
    .TAG_FIFO_DEPTH (TAG_FIFO_DEPTH)
  ) u_m5 (
    .clk(clk),
    .rst_n(rst_n),

    .in_valid(b_valid),
    .in_ready(b_ready),

    .q_vec (q_vec_attn),
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
