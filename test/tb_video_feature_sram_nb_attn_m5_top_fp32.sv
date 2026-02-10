// ============================================================
// tb_video_feature_sram_nb_attn_m5_top_fp32.sv  (IVERILOG-SAFE)
//
// DUT: video_sram_nb_attn_m5_top_fp32  (included via your src file)
// - Drive video frames to populate feature SRAM
// - Then issue center tile requests
// - TB computes golden at *bundle_fire* (DUT in_valid&&in_ready into M5)
//   and compares on out_fire.
//
// FIXED:
//  - Wait center_fire (center_valid && center_ready)
//  - SB push on bundle_fire ONLY
//  - Ignore out_fire before first bundle_fire
//  - Add gather watchdog to print DUT internal state if stuck
//
// Build:
//   iverilog -g2012 -Wall -I./src -o ./vvp/tb_vid_feat_m5.vvp ./test/tb_video_feature_sram_nb_attn_m5_top_fp32.sv
// Run:
//   vvp ./vvp/tb_vid_feat_m5.vvp
// ============================================================

`include "./src/AMOLED/video_feature_sram_nb_attn_m5_top_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_video_feature_sram_nb_attn_m5_top_fp32;

  // ----------------------------
  // Small sim params
  // ----------------------------
  localparam int unsigned ACTIVE_W   = 32;
  localparam int unsigned ACTIVE_H   = 16;
  localparam int unsigned TILE_SHIFT = 2;  // 4x4 tiles
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT);
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT);

  localparam int unsigned X_W        = 11;
  localparam int unsigned Y_W        = 10;

  localparam int unsigned PACK_OUT_W = 10;
  localparam int unsigned YPIX_W     = 8;

  localparam int unsigned FEAT_DIM   = 8;
  localparam int unsigned elen_W     = 32;
  localparam int unsigned tag_w      = 16;

  localparam int unsigned TOKENS     = 9;
  localparam int unsigned D          = 8;
  localparam int unsigned OUT_W      = D*32;
  localparam int unsigned IDX_W      = 4;

  localparam int unsigned FRAME_RD_SETTLE_CYC = 80;
  localparam int unsigned TX_GOAL = 40;
  localparam int unsigned TIMEOUT_CYC = 300000;

  // Attention params (match DUT defaults)
  localparam int unsigned ALPHA_MODE  = 1;
  localparam real         ALPHA_SCALE = 1.0;
  localparam real         ALPHA_BIAS  = 0.0;
  localparam real         ALPHA_SIG_A = 1.0;

  // M2 approx const
  localparam real LOG2E  = 1.4426950408889634;
  localparam real LN2    = 0.6931471805599453;
  localparam real XCLAMP = 8.0;

  // ----------------------------
  // Clock / reset
  // ----------------------------
  reg clk; initial begin clk=1'b0; forever #5 clk=~clk; end
  reg rst_n, en;
  initial begin
    rst_n = 1'b0; en = 1'b0;
    repeat(10) @(posedge clk);
    en = 1'b1;
    rst_n = 1'b1;
  end
  wire go = en && rst_n;

  // ----------------------------
  // Video inputs
  // ----------------------------
  reg vsync_i, hsync_i, de_i;
  reg [23:0] rgb_i;
  reg clip_en;
  reg [PACK_OUT_W-1:0] clip_min, clip_max;

  // ----------------------------
  // Center req
  // ----------------------------
  reg                         center_valid;
  wire                        center_ready;
  reg  [$clog2(TILES_Y)-1:0]   center_i;
  reg  [$clog2(TILES_X)-1:0]   center_j;
  reg  [tag_w-1:0]             center_tag;

  wire center_fire = go && center_valid && center_ready;

  // ----------------------------
  // Outputs
  // ----------------------------
  wire                        out_valid;
  reg                         out_ready;
  wire [31:0]                 alpha_fp32;
  wire [OUT_W-1:0]            out_vec_dbg;
  wire [tag_w-1:0]            out_tag;

  // debug taps
  wire in_frame;
  wire pix_valid_timing;
  wire wr_fire;
  wire [15:0] wr_tile_i, wr_tile_j;

  wire err_mismatch_pulse;
  wire [31:0] cnt_join_ok, cnt_mismatch, cnt_drop;
  wire skid_drop_pulse;
  wire [31:0] skid_drop_cnt;

  // ----------------------------
  // DUT
  // ----------------------------
  video_sram_nb_attn_m5_top_fp32 #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT), .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W), .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM), .elen_W(elen_W), .tag_w(tag_w),

    .TOKENS(TOKENS),
    .D(D),

    .BLOCK_CENTER_WHEN_IN_FRAME(1'b1)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .en(en),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i(de_i),
    .rgb_i(rgb_i),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .center_valid(center_valid),
    .center_ready(center_ready),
    .center_i(center_i),
    .center_j(center_j),
    .center_tag(center_tag),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .alpha_fp32(alpha_fp32),
    .out_vec_dbg(out_vec_dbg),
    .out_tag(out_tag),

    .in_frame(in_frame),
    .pix_valid_timing(pix_valid_timing),
    .wr_fire(wr_fire),
    .wr_tile_i(wr_tile_i),
    .wr_tile_j(wr_tile_j),

    .err_mismatch_pulse(err_mismatch_pulse),
    .cnt_join_ok(cnt_join_ok),
    .cnt_mismatch(cnt_mismatch),
    .cnt_drop(cnt_drop),

    .skid_drop_pulse(skid_drop_pulse),
    .skid_drop_cnt(skid_drop_cnt)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("./vvp/tb_video_feature_sram_nb_attn_m5_top_fp32.vcd");
    $dumpvars(0, tb_video_feature_sram_nb_attn_m5_top_fp32);
  end

  // ============================================================
  // Video generator
  // ============================================================
  localparam int unsigned HBLANK = 4;
  localparam int unsigned VBLANK = 8;
  localparam int unsigned HS_PW  = 2;
  localparam int unsigned VS_EDGE_GAP = 2;

  task drive_one_frame(input int frame_id);
    int yy, xx;
    begin
      @(negedge clk);
      de_i    = 1'b0;
      hsync_i = 1'b0;
      rgb_i   = 24'h0;

      vsync_i = 1'b1;
      repeat (VS_EDGE_GAP) @(negedge clk);

      for (yy = 0; yy < ACTIVE_H; yy++) begin
        @(negedge clk);
        de_i    = 1'b0;
        rgb_i   = 24'h0;

        hsync_i = 1'b1;
        repeat (HS_PW) @(negedge clk);

        @(negedge clk);
        hsync_i = 1'b0;

        for (xx = 0; xx < ACTIVE_W; xx++) begin
          @(negedge clk);
          de_i = 1'b1;
          rgb_i[23:16] = (xx*8) & 8'hFF;
          rgb_i[15: 8] = (yy*16) & 8'hFF;
          rgb_i[ 7: 0] = (frame_id*64 + xx) & 8'hFF;
        end

        @(negedge clk);
        de_i  = 1'b0;
        rgb_i = 24'h0;
        repeat (HBLANK) @(negedge clk);
      end

      @(negedge clk);
      de_i    = 1'b0;
      hsync_i = 1'b0;
      rgb_i   = 24'h0;

      vsync_i = 1'b0;
      repeat (VS_EDGE_GAP) @(negedge clk);

      repeat (VBLANK) @(negedge clk);
    end
  endtask

  // ============================================================
  // Frame-end settle (commit-safe)
  // ============================================================
  reg vsync_q;
  reg frame_end_pulse;
  reg [31:0] settle_cnt;

  always @(posedge clk) begin
    if (!rst_n) begin
      vsync_q         <= 1'b0;
      frame_end_pulse <= 1'b0;
      settle_cnt      <= 0;
    end else begin
      vsync_q         <= vsync_i;
      frame_end_pulse <= (vsync_q==1'b1) && (vsync_i==1'b0);

      if (frame_end_pulse) settle_cnt <= FRAME_RD_SETTLE_CYC;
      else if (settle_cnt != 0) settle_cnt <= settle_cnt - 1;
    end
  end

  wire reads_allowed = go && (settle_cnt == 0) && (vsync_i == 1'b0) && (in_frame == 1'b0);

  // ============================================================
  // Random out_ready
  // ============================================================
  integer seed;
  function automatic int urand(input int mod);
    int r;
    begin
      r = $random(seed);
      if (r < 0) r = -r;
      urand = (mod==0) ? 0 : (r % mod);
    end
  endfunction

  always @(posedge clk) begin
    if (!rst_n || !en) out_ready <= 1'b0;
    else out_ready <= (urand(100) < 70);
  end

  // ============================================================
  // Hier taps for bundle_fire + inputs
  // ============================================================
  wire b_valid_dbg = dut.b_valid;
  wire b_ready_dbg = dut.b_ready;
  wire bundle_fire = go && b_valid_dbg && b_ready_dbg;

  wire [OUT_W-1:0]          q_vec_dbg   = dut.q_vec_attn;
  wire [TOKENS*OUT_W-1:0]   k_dbg       = dut.k_vecs_attn;
  wire [TOKENS*OUT_W-1:0]   v_dbg       = dut.v_vecs_attn;
  wire [tag_w-1:0]          in_tag_dbg  = dut.in_tag_attn;

  // extra taps for stuck diagnose
  wire [1:0] dut_state      = dut.state;
  wire [3:0] dut_tok_idx    = dut.tok_idx;
  wire       dut_valid_rd   = dut.valid_rd;
  wire       dut_ready_rd   = dut.ready_rd;
  wire       dut_feat_v     = dut.feat_out_valid;

  // ============================================================
  // FP32 helpers
  // ============================================================
  function real pow2i(input integer e);
    integer i; real v;
    begin
      v=1.0;
      if(e>=0) for(i=0;i<e;i=i+1) v=v*2.0;
      else     for(i=0;i<-e;i=i+1) v=v/2.0;
      pow2i=v;
    end
  endfunction

  function real fp32_to_real(input [31:0] f);
    reg sign; integer exp, mant; real frac, val;
    begin
      sign=f[31]; exp=f[30:23]; mant=f[22:0];
      if(exp==0) begin
        if(mant==0) val=0.0;
        else val=(mant/8388608.0)*pow2i(-126);
      end else if(exp==255) begin
        val=(mant==0)?1.0e30:0.0;
      end else begin
        frac=1.0+(mant/8388608.0);
        val=frac*pow2i(exp-127);
      end
      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign; real a,v; integer e,exp_i,m; real frac;
    begin
      sign=(r<0.0); a=sign?-r:r;
      if(a==0.0) real_to_fp32=32'h0;
      else begin
        v=a; e=0;
        while(v>=2.0) begin v=v/2.0; e=e+1; end
        while(v<1.0)  begin v=v*2.0; e=e-1; end
        exp_i=e+127;
        frac=v-1.0;
        m=integer'(frac*8388608.0+0.5);
        if(m>=8388608) begin m=0; exp_i=exp_i+1; end
        real_to_fp32={sign,exp_i[7:0],m[22:0]};
      end
    end
  endfunction

  function real exp2_frac(input real F);
    real z,z2,z3;
    begin
      z=F*LN2; z2=z*z; z3=z2*z;
      exp2_frac = 1.0 + z + (z2*0.5) + (z3*(1.0/6.0));
    end
  endfunction

  function real exp2_approx(input real y);
    integer it, ifloor; real F;
    begin
      it = integer'(y);
      if ((y<0.0) && (y!=it)) ifloor = it-1;
      else                    ifloor = it;
      F = y - ifloor;
      exp2_approx = pow2i(ifloor) * exp2_frac(F);
    end
  endfunction

  function [TOKENS*32-1:0] golden_score(input [OUT_W-1:0] q, input [TOKENS*OUT_W-1:0] k_vecs);
    integer t,d;
    real acc, invs;
    reg [TOKENS*32-1:0] tmp;
    begin
      invs = 1.0 / $sqrt(D);
      tmp='0;
      for (t=0;t<TOKENS;t=t+1) begin
        acc=0.0;
        for (d=0; d<D; d=d+1) begin
          acc = acc
              + fp32_to_real(q[d*32 +: 32])
              * fp32_to_real(k_vecs[(t*OUT_W + d*32) +: 32]);
        end
        acc = acc * invs;
        tmp[t*32 +: 32] = real_to_fp32(acc);
      end
      golden_score = tmp;
    end
  endfunction

  function [TOKENS*32-1:0] golden_w(input [TOKENS*32-1:0] sflat);
    real s[0:8];
    real m,x,y,a[0:8],sum;
    integer i;
    reg [TOKENS*32-1:0] tmp;
    begin
      for (i=0;i<9;i=i+1) s[i] = fp32_to_real(sflat[i*32 +: 32]);
      m=s[0];
      for (i=1;i<9;i=i+1) if (s[i]>m) m=s[i];

      sum=0.0;
      for (i=0;i<9;i=i+1) begin
        x = s[i]-m;
        if(x<-XCLAMP) x=-XCLAMP;
        if(x>0.0)    x=0.0;
        y = x*LOG2E;
        a[i] = exp2_approx(y);
        sum = sum + a[i];
      end

      tmp='0;
      for (i=0;i<9;i=i+1)
        tmp[i*32 +: 32] = real_to_fp32(a[i]/sum);

      golden_w = tmp;
    end
  endfunction

  function [OUT_W-1:0] golden_out_vec(input [TOKENS*32-1:0] w_in, input [TOKENS*OUT_W-1:0] v_in);
    integer d,t;
    real acc, wr, vr;
    reg [OUT_W-1:0] tmp;
    begin
      tmp='0;
      for (d=0; d<D; d=d+1) begin
        acc = 0.0;
        for (t=0; t<TOKENS; t=t+1) begin
          wr = fp32_to_real(w_in[t*32 +: 32]);
          vr = fp32_to_real(v_in[(t*OUT_W + d*32) +: 32]);
          acc = acc + (wr * vr);
        end
        tmp[d*32 +: 32] = real_to_fp32(acc);
      end
      golden_out_vec = tmp;
    end
  endfunction

  function real clamp01(input real x);
    begin
      if (x<0.0) clamp01=0.0;
      else if (x>1.0) clamp01=1.0;
      else clamp01=x;
    end
  endfunction

  function real sigmoid_approx(input real x);
    real z, e;
    begin
      if (x > 8.0) sigmoid_approx = 0.999664;
      else if (x < -8.0) sigmoid_approx = 0.000335;
      else begin
        z = (-ALPHA_SIG_A * x) * LOG2E;
        e = exp2_approx(z);
        sigmoid_approx = 1.0 / (1.0 + e);
      end
    end
  endfunction

  function [31:0] golden_alpha(input [31:0] out0_bits);
    real x, a;
    begin
      x = fp32_to_real(out0_bits);
      if (ALPHA_MODE == 0)      a = sigmoid_approx(x);
      else if (ALPHA_MODE == 1) a = clamp01(x);
      else                      a = clamp01(ALPHA_SCALE*x + ALPHA_BIAS);
      golden_alpha = real_to_fp32(a);
    end
  endfunction

  // ============================================================
  // Scoreboard queues (expected out_vec/alpha/tag)
  // ============================================================
  localparam int MAXQ = 4096;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  reg [31:0]      exp_a_q   [0:MAXQ-1];
  reg [tag_w-1:0] exp_t_q   [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [OUT_W-1:0] o, input [31:0] a, input [tag_w-1:0] tg);
    begin
      if (sb_cnt >= MAXQ-1) begin
        $display("[FATAL] SB overflow");
        $fatal(1);
      end
      exp_out_q[sb_wr] = o;
      exp_a_q[sb_wr]   = a;
      exp_t_q[sb_wr]   = tg;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o, output [31:0] a, output [tag_w-1:0] tg);
    begin
      if (sb_cnt <= 0) begin
        $display("[FATAL] SB underflow");
        $fatal(1);
      end
      o  = exp_out_q[sb_rd];
      a  = exp_a_q[sb_rd];
      tg = exp_t_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // push golden on bundle_fire
  integer bundle_cnt;
  reg seen_first_bundle;

  always @(posedge clk) begin
    if (!rst_n) begin
      sb_wr <= 0; sb_rd <= 0; sb_cnt <= 0;
      bundle_cnt <= 0;
      seen_first_bundle <= 1'b0;
    end else if (en && go) begin
      if (bundle_fire) begin
        reg [TOKENS*32-1:0] sflat;
        reg [TOKENS*32-1:0] wflat;
        reg [OUT_W-1:0]     outv;
        reg [31:0]          alph;

        if ((^q_vec_dbg) === 1'bX || (^k_dbg) === 1'bX || (^v_dbg) === 1'bX || (^in_tag_dbg) === 1'bX) begin
          $display("[FATAL] X on bundle inputs at bundle_fire @%0t", $time);
          $fatal(1);
        end

        sflat = golden_score(q_vec_dbg, k_dbg);
        wflat = golden_w(sflat);
        outv  = golden_out_vec(wflat, v_dbg);
        alph  = golden_alpha(outv[0*32 +: 32]);

        sb_push(outv, alph, in_tag_dbg);

        bundle_cnt <= bundle_cnt + 1;
        seen_first_bundle <= 1'b1;

        if (bundle_cnt < 6) begin
          $display("[%0t] BUNDLE_FIRE #%0d tag=%h (sb_cnt->%0d)",
                   $time, bundle_cnt, in_tag_dbg, sb_cnt+1);
        end
      end
    end
  end

  // ============================================================
  // Center driver
  // ============================================================
  integer tx_sent;
  reg [tag_w-1:0] next_tag;

  task send_one_center();
    int ci, cj;
    int tmp;
    int k;
    begin
      if (TILES_Y > 2) ci = 1 + urand(TILES_Y-2); else ci = urand(TILES_Y);
      if (TILES_X > 2) cj = 1 + urand(TILES_X-2); else cj = urand(TILES_X);

      // group tag in upper bits, low IDX_W forced 0 (matches DUT in_tag policy)
      tmp      = urand(1 << (tag_w-IDX_W));
      next_tag = { tmp[tag_w-IDX_W-1:0], {IDX_W{1'b0}} };

      @(negedge clk);
      center_i     <= ci[$clog2(TILES_Y)-1:0];
      center_j     <= cj[$clog2(TILES_X)-1:0];
      center_tag   <= next_tag;
      center_valid <= 1'b1;

      k = 0;
      while (!center_fire) begin
        @(posedge clk);
        k++;
        if (k > 20000) begin
          $display("[FATAL] timeout waiting center_fire. center_ready=%b in_frame=%b settle_cnt=%0d",
                   center_ready, in_frame, settle_cnt);
          $display("        DUT state=%0d tok=%0d vrd=%b rrd=%b feat_v=%b",
                   dut_state, dut_tok_idx, dut_valid_rd, dut_ready_rd, dut_feat_v);
          $fatal(1);
        end
      end

      @(negedge clk);
      center_valid <= 1'b0;

      tx_sent <= tx_sent + 1;
      if (tx_sent < 8)
        $display("[%0t] TX center=(%0d,%0d) ctag=%h",
                 $time, ci, cj, next_tag);
    end
  endtask

  // ============================================================
  // Output monitor + compare + hold-under-stall
  // ============================================================
  wire out_fire = go && out_valid && out_ready;

  reg [OUT_W-1:0] hold_out_prev;
  reg [31:0]      hold_a_prev;
  reg [tag_w-1:0] hold_t_prev;
  reg             hold_v;

  integer rx_got;
  integer mismatch_pulses, skid_pulses;

  always @(posedge clk) begin
    if (!rst_n) begin
      hold_v <= 1'b0;
      rx_got <= 0;
      mismatch_pulses <= 0;
      skid_pulses <= 0;
    end else if (en) begin
      if (err_mismatch_pulse) mismatch_pulses <= mismatch_pulses + 1;
      if (skid_drop_pulse)    skid_pulses     <= skid_pulses + 1;

      // hold check
      if (out_valid && !out_ready) begin
        if (hold_v) begin
          if (out_vec_dbg !== hold_out_prev) begin
            $display("[FAIL] out_vec changed under stall @%0t", $time);
            $fatal(1);
          end
          if (alpha_fp32 !== hold_a_prev) begin
            $display("[FAIL] alpha changed under stall @%0t", $time);
            $fatal(1);
          end
          if (out_tag !== hold_t_prev) begin
            $display("[FAIL] out_tag changed under stall @%0t", $time);
            $fatal(1);
          end
        end
        hold_out_prev <= out_vec_dbg;
        hold_a_prev   <= alpha_fp32;
        hold_t_prev   <= out_tag;
        hold_v        <= 1'b1;
      end else begin
        hold_v <= 1'b0;
      end

      // ignore spurious out before any bundle_fire pushed golden
      if (out_fire && !seen_first_bundle) begin
        $display("[%0t] WARN: out_fire before first bundle_fire (ignored).", $time);
      end else if (out_fire) begin
        reg [OUT_W-1:0] exp_out;
        reg [31:0]      exp_a;
        reg [tag_w-1:0] exp_t;
        integer d;
        real er, ar;

        if ((^out_tag) === 1'bX || (^alpha_fp32) === 1'bX || (^out_vec_dbg) === 1'bX) begin
          $display("[FAIL] X on outputs at out_fire @%0t", $time);
          $fatal(1);
        end

        sb_pop(exp_out, exp_a, exp_t);

        if (out_tag !== exp_t) begin
          $display("[FAIL] out_tag mismatch @rx=%0d time=%0t exp=%h got=%h",
                   rx_got, $time, exp_t, out_tag);
          $fatal(1);
        end

        if (out_vec_dbg !== exp_out) begin
          $display("[FAIL] out_vec mismatch @rx=%0d time=%0t", rx_got, $time);
          for (d=0; d<D; d=d+1) begin
            $display("  d=%0d exp=%h got=%h exp_r=%f got_r=%f",
              d,
              exp_out[d*32 +: 32],
              out_vec_dbg[d*32 +: 32],
              fp32_to_real(exp_out[d*32 +: 32]),
              fp32_to_real(out_vec_dbg[d*32 +: 32])
            );
          end
          $fatal(1);
        end

        if (alpha_fp32 !== exp_a) begin
          er = fp32_to_real(exp_a);
          ar = fp32_to_real(alpha_fp32);
          $display("[FAIL] alpha mismatch @rx=%0d exp=%h(%f) got=%h(%f)",
                   rx_got, exp_a, er, alpha_fp32, ar);
          $fatal(1);
        end

        if (rx_got < 8) begin
          $display("[%0t] RX #%0d tag=%h alpha=%h sb_cnt=%0d",
                   $time, rx_got, out_tag, alpha_fp32, sb_cnt);
        end

        rx_got <= rx_got + 1;
      end
    end
  end

  // ============================================================
  // Gather watchdog: if stuck after first TX, print DUT state
  // ============================================================
  initial begin
    int stall;
    stall = 0;

    wait(rst_n==1'b1);
    wait(en==1'b1);

    // wait until first TX happens
    wait(tx_sent > 0);

    // after first TX, require progress: either bundle_fire increments or out fires
    while (bundle_cnt == 0 && rx_got == 0) begin
      @(posedge clk);
      stall++;

      if (stall % 2000 == 0) begin
        $display("[DBG %0t] stall=%0d state=%0d tok=%0d vrd=%b rrd=%b feat_v=%b b_v=%b b_r=%b in_frame=%b settle=%0d sb_cnt=%0d",
                 $time, stall,
                 dut_state, dut_tok_idx,
                 dut_valid_rd, dut_ready_rd, dut_feat_v,
                 b_valid_dbg, b_ready_dbg,
                 in_frame, settle_cnt, sb_cnt);
      end

      if (stall > 40000) begin
        $display("[FATAL] No bundle_fire/out after first TX => stuck in gather/read-return?");
        $display("        state=%0d tok=%0d vrd=%b rrd=%b feat_v=%b in_frame=%b settle=%0d",
                 dut_state, dut_tok_idx, dut_valid_rd, dut_ready_rd, dut_feat_v, in_frame, settle_cnt);
        $fatal(1);
      end
    end
  end

  // ============================================================
  // INIT
  // ============================================================
  initial begin
    vsync_i = 1'b0; hsync_i = 1'b0; de_i = 1'b0; rgb_i = 24'h0;
    clip_en = 1'b0; clip_min = '0; clip_max = {PACK_OUT_W{1'b1}};
    center_valid = 1'b0; center_i='0; center_j='0; center_tag='0;
    out_ready = 1'b0;
    seed = 32'h1357_2468;
    tx_sent = 0;

    wait(rst_n==1'b1);
    repeat(10) @(posedge clk);

    $display("=== TB start: populate frames then centers; SB push on bundle_fire ===");

    drive_one_frame(0);
    drive_one_frame(1);

    $display("[CHK] join_ok=%0d mismatch=%0d drop=%0d skid_drop_cnt=%0d",
             cnt_join_ok, cnt_mismatch, cnt_drop, skid_drop_cnt);

    if ((cnt_join_ok == 0) && (cnt_mismatch == 0) && (cnt_drop == 0)) begin
      $display("[FATAL] No activity in writer: check DE_INV/HSYNC/EOL/Vsync assumptions.");
      $fatal(1);
    end

    while (!reads_allowed) @(posedge clk);

    while (tx_sent < TX_GOAL) begin
      if (reads_allowed && center_ready) send_one_center();
      @(posedge clk);
    end

    // wait until scoreboard drained and pipeline quiet
    begin
      integer idle;
      idle = 0;
      while (1) begin
        @(posedge clk);
        if ((tx_sent >= TX_GOAL) && (sb_cnt == 0) && !out_valid) idle++;
        else idle = 0;

        if (idle > 300) begin
          $display("=== TB done: tx=%0d bundle_cnt=%0d rx=%0d sb_cnt=%0d mismatch_pulses=%0d skid_pulses=%0d ===",
                   tx_sent, bundle_cnt, rx_got, sb_cnt, mismatch_pulses, skid_pulses);
          $display("DUT stats: join_ok=%0d mismatch=%0d drop=%0d skid_drop_cnt=%0d",
                   cnt_join_ok, cnt_mismatch, cnt_drop, skid_drop_cnt);
          if (mismatch_pulses != 0) $fatal(1);
          $finish;
        end
      end
    end
  end

  // timeout guard
  initial begin
    int k;
    k=0;
    while (k < TIMEOUT_CYC) begin
      @(posedge clk); k++;
    end
    $display("[FATAL] TIMEOUT (likely stuck in gather or no bundle_fire)");
    $fatal(1);
  end

endmodule

`default_nettype wire
