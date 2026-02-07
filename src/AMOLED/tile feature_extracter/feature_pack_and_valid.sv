`ifndef FEATURE_PACK_AND_VALID_SV
`define FEATURE_PACK_AND_VALID_SV

`timescale 1ns/1ps
`default_nettype none

module feature_pack_and_valid #(
  parameter int unsigned TILES_X = 320,
  parameter int unsigned TILES_Y = 180,

  parameter int unsigned SUM_W   = 12,
  parameter int unsigned Y_W     = 8,
  parameter int unsigned EDGE_W  = 32,

  parameter int unsigned FEAT_W  = 16,
  parameter int unsigned FEAT_DIM= 4,        // 1..8

  parameter bit EDGE_USE_MEAN = 1'b1,

  // mean config
  parameter int unsigned TILE_PIX_SHIFT  = 4,
  parameter int unsigned TILE_PIX_N      = 16,
  parameter bit          MEAN_MODE_SHIFT = 1'b1,

  // 0: strict same-cycle join (NO internal buffering)
  // 1: buffered join (recommended)
  parameter bit MATCH_MODE_BUFFERED = 1'b1,

  // mismatch drop policy (only used when buffered)
  // 0: drop both
  // 1: drop older id (recommended)
  // 2: drop v3 only
  // 3: drop v4 only
  parameter int unsigned MISMATCH_POLICY = 1,

  parameter bit ZERO_WHEN_INVALID = 1'b0,

  // post-process
  parameter int unsigned FEAT0_SHIFT = 0,
  parameter int unsigned FEAT1_SHIFT = 0,
  parameter int unsigned FEAT2_SHIFT = 0,
  parameter int unsigned FEAT3_SHIFT = 0,
  parameter int signed   FEAT0_BIAS  = 0,
  parameter int signed   FEAT1_BIAS  = 0,
  parameter int signed   FEAT2_BIAS  = 0,
  parameter int signed   FEAT3_BIAS  = 0,
  parameter int unsigned FEAT_CLAMP_MAX = (1<<FEAT_W)-1,

  parameter int unsigned TILE_I_W  = 16,
  parameter int unsigned TILE_J_W  = 16,
  parameter int unsigned TILE_ID_W = (TILES_X*TILES_Y <= 1) ? 1 : $clog2(TILES_X*TILES_Y),

  parameter bit ENABLE_CNTS = 1'b1
)(
  input  logic                     clk,
  input  logic                     rst,

  // V3 (stats)
  input  logic                     v3_valid,
  output logic                     v3_ready,
  input  logic [SUM_W-1:0]         sumY,
  input  logic [Y_W-1:0]           minY,
  input  logic [Y_W-1:0]           maxY,
  input  logic [TILE_I_W-1:0]      tile_i,
  input  logic [TILE_J_W-1:0]      tile_j,

  // V4 (edge)
  input  logic                     v4_valid,
  output logic                     v4_ready,
  input  logic [EDGE_W-1:0]        edge_sum,
  input  logic [TILE_I_W-1:0]      v4_tile_i,
  input  logic [TILE_J_W-1:0]      v4_tile_j,

  // Output
  output logic                     feat_valid,
  input  logic                     feat_ready,

  output logic [TILE_I_W-1:0]      tile_i_o,
  output logic [TILE_J_W-1:0]      tile_j_o,
  output logic [TILE_ID_W-1:0]     tile_id_o,

  output logic [FEAT_W-1:0]        feat0,
  output logic [FEAT_W-1:0]        feat1,
  output logic [FEAT_W-1:0]        feat2,
  output logic [FEAT_W-1:0]        feat3,
  output logic [FEAT_W-1:0]        feat4,
  output logic [FEAT_W-1:0]        feat5,
  output logic [FEAT_W-1:0]        feat6,
  output logic [FEAT_W-1:0]        feat7,
  output logic [8*FEAT_W-1:0]      feat_vec,

  output logic                     err_mismatch_pulse,
  output logic [31:0]              cnt_join_ok,
  output logic [31:0]              cnt_mismatch,
  output logic [31:0]              cnt_drop
);

  // ------------------------------------------------------------
  // util
  // ------------------------------------------------------------
  function automatic logic [TILE_ID_W-1:0] calc_id(
    input logic [TILE_I_W-1:0] ti,
    input logic [TILE_J_W-1:0] tj
  );
    logic [31:0] full;
    begin
      full = (ti * TILES_X) + tj;
      calc_id = full[TILE_ID_W-1:0];
    end
  endfunction

  function automatic logic [FEAT_W-1:0] clamp_u(input integer signed x);
    integer signed y;
    begin
      y = x;
      if (y < 0) y = 0;
      if (y > integer'(FEAT_CLAMP_MAX)) y = integer'(FEAT_CLAMP_MAX);
      clamp_u = y[FEAT_W-1:0];
    end
  endfunction

  function automatic logic [FEAT_W-1:0] postproc(
    input logic [FEAT_W-1:0] in_u,
    input int unsigned       sh,
    input int signed         bias
  );
    integer signed x;
    begin
      x = $signed({1'b0, in_u}) >>> sh;
      x = x + bias;
      postproc = clamp_u(x);
    end
  endfunction

  function automatic logic [FEAT_W-1:0] y_mean_from_sum(input logic [SUM_W-1:0] s);
    integer unsigned m;
    begin
      if (MEAN_MODE_SHIFT) begin
        if (SUM_W > TILE_PIX_SHIFT) m = (s >> TILE_PIX_SHIFT);
        else                        m = 0;
      end else begin
        if (TILE_PIX_N != 0) m = s / TILE_PIX_N;
        else                 m = 0;
      end
      y_mean_from_sum = m[FEAT_W-1:0];
    end
  endfunction

  function automatic logic [FEAT_W-1:0] edge_feat_from_sum(input logic [EDGE_W-1:0] e);
    logic [EDGE_W-1:0] tmp;
    begin
      tmp = (EDGE_USE_MEAN) ? (e >> TILE_PIX_SHIFT) : e;
      edge_feat_from_sum = tmp[FEAT_W-1:0];
    end
  endfunction

  // ------------------------------------------------------------
  // Output 1-deep hold
  // ------------------------------------------------------------
  logic hold_valid;
  logic [TILE_I_W-1:0]  ti_r;
  logic [TILE_J_W-1:0]  tj_r;
  logic [TILE_ID_W-1:0] id_r;
  logic [FEAT_W-1:0] f0_r,f1_r,f2_r,f3_r,f4_r,f5_r,f6_r,f7_r;
  logic [8*FEAT_W-1:0] vec_r;

  wire out_can_load = (!hold_valid) || feat_ready;

  // output comb
  always_comb begin
    feat_valid = hold_valid;

    if (ZERO_WHEN_INVALID && !hold_valid) begin
      tile_i_o='0; tile_j_o='0; tile_id_o='0;
      feat0='0; feat1='0; feat2='0; feat3='0;
      feat4='0; feat5='0; feat6='0; feat7='0;
      feat_vec='0;
    end else begin
      tile_i_o=ti_r; tile_j_o=tj_r; tile_id_o=id_r;
      feat0=f0_r; feat1=f1_r; feat2=f2_r; feat3=f3_r;
      feat4=f4_r; feat5=f5_r; feat6=f6_r; feat7=f7_r;
      feat_vec=vec_r;
    end
  end

  // ------------------------------------------------------------
  // Buffered mode: 1-deep skid for each input
  // ------------------------------------------------------------
  logic b3_v, b4_v;

  logic [SUM_W-1:0]    sum_b;
  logic [Y_W-1:0]      min_b, max_b;
  logic [TILE_I_W-1:0] ti_b;
  logic [TILE_J_W-1:0] tj_b;

  logic [EDGE_W-1:0]   edge_b;
  logic [TILE_I_W-1:0] v4_ti_b;
  logic [TILE_J_W-1:0] v4_tj_b;

  wire [TILE_ID_W-1:0] id3_b = calc_id(ti_b,   tj_b);
  wire [TILE_ID_W-1:0] id4_b = calc_id(v4_ti_b, v4_tj_b);

  wire b_match   = b3_v && b4_v && (id3_b == id4_b);
  wire b_mis     = b3_v && b4_v && (id3_b != id4_b);

  // drop decision (only when b_mis=1)
  logic drop3, drop4;
  always_comb begin
    drop3 = 1'b0;
    drop4 = 1'b0;
    if (b_mis) begin
      case (MISMATCH_POLICY)
        0: begin drop3 = 1'b1; drop4 = 1'b1; end // drop both
        1: begin // drop older id
          if (id3_b < id4_b) begin drop3 = 1'b1; drop4 = 1'b0; end
          else               begin drop3 = 1'b0; drop4 = 1'b1; end
        end
        2: begin drop3 = 1'b1; drop4 = 1'b0; end
        default: begin drop3 = 1'b0; drop4 = 1'b1; end
      endcase
    end
  end

  // join fire
  wire do_join = MATCH_MODE_BUFFERED ? (b_match && out_can_load) : 1'b0;

  // in buffered mode: we can accept upstream if buffer will be free after this cycle
  // buffer frees on: do_join (consumes both) or drop of that buffer
  wire b3_will_free = do_join || (drop3);
  wire b4_will_free = do_join || (drop4);

  always@(*) begin
    if (MATCH_MODE_BUFFERED) begin
      v3_ready = (!b3_v) || b3_will_free; // can take new if empty or freeing
      v4_ready = (!b4_v) || b4_will_free;
    end else begin
      // strict: require same-cycle join and output space
      // ready only when we can accept the pair into output
      v3_ready = out_can_load && v4_valid && (calc_id(tile_i,tile_j)==calc_id(v4_tile_i,v4_tile_j));
      v4_ready = out_can_load && v3_valid && (calc_id(tile_i,tile_j)==calc_id(v4_tile_i,v4_tile_j));
    end
  end

  // strict join
  wire strict_match = v3_valid && v4_valid &&
                      (calc_id(tile_i,tile_j) == calc_id(v4_tile_i,v4_tile_j));
  wire strict_take  = (!MATCH_MODE_BUFFERED) && strict_match && out_can_load;

  // ------------------------------------------------------------
  // sequential: buffers + output + counters
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      hold_valid <= 1'b0;
      ti_r <= '0; tj_r <= '0; id_r <= '0;
      f0_r <= '0; f1_r <= '0; f2_r <= '0; f3_r <= '0;
      f4_r <= '0; f5_r <= '0; f6_r <= '0; f7_r <= '0;
      vec_r <= '0;

      b3_v <= 1'b0;
      b4_v <= 1'b0;

      sum_b <= '0; min_b <= '0; max_b <= '0; ti_b <= '0; tj_b <= '0;
      edge_b <= '0; v4_ti_b <= '0; v4_tj_b <= '0;

      err_mismatch_pulse <= 1'b0;
      cnt_join_ok  <= 32'd0;
      cnt_mismatch <= 32'd0;
      cnt_drop     <= 32'd0;
    end else begin
      err_mismatch_pulse <= 1'b0;

      // output consume
      if (hold_valid && feat_ready) begin
        hold_valid <= 1'b0;
      end

      // -------------------------
      // buffered mode path
      // -------------------------
      if (MATCH_MODE_BUFFERED) begin
        // load/update b3
        if (v3_valid && v3_ready) begin
          // if buffer is freeing and also taking new, just overwrite with new
          b3_v  <= 1'b1;
          sum_b <= sumY;
          min_b <= minY;
          max_b <= maxY;
          ti_b  <= tile_i;
          tj_b  <= tile_j;
        end else if (drop3) begin
          b3_v <= 1'b0;
          if (ENABLE_CNTS) cnt_drop <= cnt_drop + 1;
        end

        // load/update b4
        if (v4_valid && v4_ready) begin
          b4_v     <= 1'b1;
          edge_b   <= edge_sum;
          v4_ti_b  <= v4_tile_i;
          v4_tj_b  <= v4_tile_j;
        end else if (drop4) begin
          b4_v <= 1'b0;
          if (ENABLE_CNTS) cnt_drop <= cnt_drop + 1;
        end

        // mismatch pulse / count (when both valid and not match)
        if (b_mis) begin
          err_mismatch_pulse <= 1'b1;
          if (ENABLE_CNTS) cnt_mismatch <= cnt_mismatch + 1;
        end

        // join -> load output + clear both buffers
        if (do_join) begin
          logic [FEAT_W-1:0] f0n,f1n,f2n,f3n;
          logic [FEAT_W-1:0] ym, ed;
          begin
            ym  = y_mean_from_sum(sum_b);
            ed  = edge_feat_from_sum(edge_b);

            f0n = postproc(ym,  FEAT0_SHIFT, FEAT0_BIAS);
            f1n = postproc({{(FEAT_W-Y_W){1'b0}}, max_b}, FEAT1_SHIFT, FEAT1_BIAS);
            f2n = postproc({{(FEAT_W-Y_W){1'b0}}, min_b}, FEAT2_SHIFT, FEAT2_BIAS);
            f3n = postproc(ed,  FEAT3_SHIFT, FEAT3_BIAS);

            ti_r <= ti_b;
            tj_r <= tj_b;
            id_r <= id3_b;

            f0_r <= f0n; f1_r <= f1n; f2_r <= f2n; f3_r <= f3n;

            // FEAT_DIM 1..8: only first dims enabled; rest zero
            f4_r <= '0; f5_r <= '0; f6_r <= '0; f7_r <= '0;

            // pack {feat7..feat0}
            vec_r <= { {4{ {FEAT_W{1'b0}} }}, f3n, f2n, f1n, f0n };


            hold_valid <= 1'b1;

            b3_v <= 1'b0;
            b4_v <= 1'b0;

            if (ENABLE_CNTS) cnt_join_ok <= cnt_join_ok + 1;
          end
        end
      end

      // -------------------------
      // strict mode path
      // -------------------------
      else begin
        if (v3_valid && v4_valid && !strict_match) begin
          err_mismatch_pulse <= 1'b1;
          if (ENABLE_CNTS) cnt_mismatch <= cnt_mismatch + 1;
        end

        if (strict_take) begin
          logic [FEAT_W-1:0] f0n,f1n,f2n,f3n;
          logic [FEAT_W-1:0] ym, ed;
          begin
            ym  = y_mean_from_sum(sumY);
            ed  = edge_feat_from_sum(edge_sum);

            f0n = postproc(ym,  FEAT0_SHIFT, FEAT0_BIAS);
            f1n = postproc({{(FEAT_W-Y_W){1'b0}}, maxY}, FEAT1_SHIFT, FEAT1_BIAS);
            f2n = postproc({{(FEAT_W-Y_W){1'b0}}, minY}, FEAT2_SHIFT, FEAT2_BIAS);
            f3n = postproc(ed,  FEAT3_SHIFT, FEAT3_BIAS);

            ti_r <= tile_i;
            tj_r <= tile_j;
            id_r <= calc_id(tile_i, tile_j);

            f0_r <= f0n; f1_r <= f1n; f2_r <= f2n; f3_r <= f3n;
            f4_r <= '0; f5_r <= '0; f6_r <= '0; f7_r <= '0;

            vec_r <= { {4{ {FEAT_W{1'b0}} }}, f3n, f2n, f1n, f0n };


            hold_valid <= 1'b1;

            if (ENABLE_CNTS) cnt_join_ok <= cnt_join_ok + 1;
          end
        end
      end
    end
  end

  // (optional) elaboration checks
  initial begin
    if (FEAT_DIM < 1 || FEAT_DIM > 8) begin
      $fatal(1, "FEAT_DIM must be 1..8, got %0d", FEAT_DIM);
    end
    if ($bits(feat_vec) != 8*FEAT_W) begin
      $fatal(1, "feat_vec width mismatch");
    end
  end

endmodule

`default_nettype wire
`endif
