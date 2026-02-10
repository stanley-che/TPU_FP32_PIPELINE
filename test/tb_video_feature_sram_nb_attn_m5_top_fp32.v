// ============================================================
// tb_video_feature_sram_nb_attn_m5_top_fp32.sv  (IVERILOG-SAFE)
//
// DUT: video_feature_sram_nb_attn_m5_top_fp32
// - Drive video frames to populate feature SRAM
// - Then issue center tile requests; TB taps DUT internal kv[0..8]
//   at attention input fire to compute golden and scoreboard compare.
//
// Checks:
//  1) out_vec_dbg matches golden out_vec
//  2) alpha_fp32  matches golden alpha
//  3) out_tag matches expected group tag
//  4) hold under stall: out_valid && !out_ready => out_vec/alpha/out_tag stable
//  5) no X on out_vec/alpha/tag at out_fire
// ============================================================

`include "./src/AMOLED/video_feature_sram_nb_attn_m5_top_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_video_feature_sram_nb_attn_m5_top_fp32;

  // ----------------------------
  // Small geometry for sim
  // ----------------------------
  localparam int unsigned ACTIVE_W   = 32;
  localparam int unsigned ACTIVE_H   = 16;

  localparam int unsigned TILE_SHIFT = 2; // 4x4 tiles
  localparam int unsigned TILES_X    = (ACTIVE_W >> TILE_SHIFT);
  localparam int unsigned TILES_Y    = (ACTIVE_H >> TILE_SHIFT);

  localparam int unsigned X_W        = 11;
  localparam int unsigned Y_W        = 10;

  localparam int unsigned PACK_OUT_W = 10;
  localparam int unsigned YPIX_W     = 8;

  localparam int unsigned FEAT_DIM   = 8;   // D
  localparam int unsigned elen_W     = 32;  // FP32
  localparam int unsigned tag_w      = 16;

  localparam int unsigned TOKENS     = 9;
  localparam int unsigned D          = FEAT_DIM;
  localparam int unsigned FEAT_W     = FEAT_DIM*elen_W;
  localparam int unsigned V_W        = TOKENS*D*32;
  localparam int unsigned OUT_W      = D*32;
  localparam int unsigned SCORE_W    = TOKENS*32;
  localparam int unsigned W_W        = TOKENS*32;

  // attention knobs (match DUT params)
  localparam bit          Q_FROM_CENTER_IDX4 = 1'b1;
  localparam int unsigned ALPHA_MODE  = 1;
  localparam real         ALPHA_SCALE = 1.0;
  localparam real         ALPHA_BIAS  = 0.0;
  localparam real         ALPHA_SIG_A = 1.0;
  localparam bit          TAG_EN      = 1'b1;

  // exp2 approx constants (same style as你的M5 TB)
  localparam real LOG2E  = 1.4426950408889634;
  localparam real LN2    = 0.6931471805599453;
  localparam real XCLAMP = 8.0;

  localparam int unsigned IDX_W = 4; // group tag alignment rule in DUT

  localparam int TX_GOAL   = 120;
  localparam int PRINT_N   = 10;
  localparam int TIMEOUT_C = 300000;

  // ----------------------------
  // clk/rst/en
  // ----------------------------
  reg clk, rst_n, en;
  initial begin clk=1'b0; forever #5 clk=~clk; end

  initial begin
    rst_n = 1'b0;
    en    = 1'b0;
    repeat(10) @(posedge clk);
    en    = 1'b1;
    rst_n = 1'b1;
  end

  wire go = en && rst_n;

  // ----------------------------
  // Video in
  // ----------------------------
  reg vsync_i, hsync_i, de_i;
  reg [23:0] rgb_i;

  // clip
  reg clip_en;
  reg [PACK_OUT_W-1:0] clip_min, clip_max;

  // ----------------------------
  // Center requests
  // ----------------------------
  reg                     center_valid;
  wire                    center_ready;
  reg [$clog2(TILES_Y)-1:0] center_i;
  reg [$clog2(TILES_X)-1:0] center_j;
  reg [tag_w-1:0]           center_tag;

  // ----------------------------
  // Output
  // ----------------------------
  wire                    out_valid;
  reg                     out_ready;
  wire [31:0]             alpha_fp32;
  wire [OUT_W-1:0]         out_vec_dbg;
  wire [tag_w-1:0]         out_tag;

  // video debug
  wire wr_fire;
  wire [15:0] wr_tile_i, wr_tile_j;
  wire in_frame, in_line, pix_valid_timing;

  // ----------------------------
  // DUT
  // ----------------------------
  video_feature_sram_nb_attn_m5_top_fp32 #(
    .X_W(X_W), .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W), .ACTIVE_H(ACTIVE_H),
    .TILE_SHIFT(TILE_SHIFT),
    .TILES_X(TILES_X), .TILES_Y(TILES_Y),
    .PACK_OUT_W(PACK_OUT_W),
    .YPIX_W(YPIX_W),
    .FEAT_DIM(FEAT_DIM),
    .elen_W(elen_W),
    .tag_w(tag_w),

    .TOKENS(TOKENS),
    .D(D),
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4),

    .ALPHA_MODE(ALPHA_MODE),
    .ALPHA_SCALE(ALPHA_SCALE),
    .ALPHA_BIAS(ALPHA_BIAS),
    .ALPHA_SIG_A(ALPHA_SIG_A),
    .TAG_EN(TAG_EN),

    .ALLOW_READ_DURING_FRAME(1'b0) // commit-safe: only read when not in_frame
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

    .wr_fire(wr_fire),
    .wr_tile_i(wr_tile_i),
    .wr_tile_j(wr_tile_j),
    .in_frame(in_frame),
    .in_line(in_line),
    .pix_valid_timing(pix_valid_timing)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("./vvp/tb_nb_attn.vcd");
    $dumpvars(0, tb_video_feature_sram_nb_attn_m5_top_fp32);
  end

  // ============================================================
  // Video generator (same style: VS high during frame)
  // ============================================================
  localparam int unsigned HBLANK = 4;
  localparam int unsigned VBLANK = 8;
  localparam int unsigned HS_PW  = 2;
  localparam int unsigned VS_EDGE_GAP = 2;

  task drive_one_frame(input integer frame_id);
    integer yy, xx;
    begin
      @(negedge clk);
      de_i    = 1'b0;
      hsync_i = 1'b0;
      rgb_i   = 24'h0;

      vsync_i = 1'b1;                 // VS rise (frame start)
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

      vsync_i = 1'b0;                 // VS fall (frame end)
      repeat (VS_EDGE_GAP) @(negedge clk);

      repeat (VBLANK) @(negedge clk);
    end
  endtask

  // ============================================================
  // FP32 helpers (same as你舊M5 TB)
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

  // exp2 approx
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

  // golden score
  function [SCORE_W-1:0] golden_score(input [D*32-1:0] q, input [V_W-1:0] k_vecs);
    integer t,d;
    real acc, invs;
    reg [SCORE_W-1:0] tmp;
    begin
      invs = 1.0 / $sqrt(D);
      tmp='0;
      for (t=0;t<TOKENS;t=t+1) begin
        acc=0.0;
        for (d=0; d<D; d=d+1) begin
          acc = acc
              + fp32_to_real(q[d*32 +: 32])
              * fp32_to_real(k_vecs[(t*D+d)*32 +: 32]);
        end
        acc = acc * invs;
        tmp[t*32 +: 32] = real_to_fp32(acc);
      end
      golden_score = tmp;
    end
  endfunction

  // golden softmax weights (scalar unrolled for 9 tokens)
  function [W_W-1:0] golden_w(input [SCORE_W-1:0] sflat);
    real s0,s1,s2,s3,s4,s5,s6,s7,s8;
    real m,x,y;
    real a0,a1,a2,a3,a4,a5,a6,a7,a8;
    real sum;
    reg [W_W-1:0] tmp;
    begin
      s0 = fp32_to_real(sflat[0*32 +:32]);
      s1 = fp32_to_real(sflat[1*32 +:32]);
      s2 = fp32_to_real(sflat[2*32 +:32]);
      s3 = fp32_to_real(sflat[3*32 +:32]);
      s4 = fp32_to_real(sflat[4*32 +:32]);
      s5 = fp32_to_real(sflat[5*32 +:32]);
      s6 = fp32_to_real(sflat[6*32 +:32]);
      s7 = fp32_to_real(sflat[7*32 +:32]);
      s8 = fp32_to_real(sflat[8*32 +:32]);

      m=s0;
      if(s1>m) m=s1; if(s2>m) m=s2; if(s3>m) m=s3; if(s4>m) m=s4;
      if(s5>m) m=s5; if(s6>m) m=s6; if(s7>m) m=s7; if(s8>m) m=s8;

      sum=0.0;
      x = s0-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a0=exp2_approx(y); sum=sum+a0;
      x = s1-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a1=exp2_approx(y); sum=sum+a1;
      x = s2-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a2=exp2_approx(y); sum=sum+a2;
      x = s3-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a3=exp2_approx(y); sum=sum+a3;
      x = s4-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a4=exp2_approx(y); sum=sum+a4;
      x = s5-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a5=exp2_approx(y); sum=sum+a5;
      x = s6-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a6=exp2_approx(y); sum=sum+a6;
      x = s7-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a7=exp2_approx(y); sum=sum+a7;
      x = s8-m; if(x<-XCLAMP) x=-XCLAMP; if(x>0.0) x=0.0; y=x*LOG2E; a8=exp2_approx(y); sum=sum+a8;

      tmp='0;
      tmp[0*32 +:32] = real_to_fp32(a0/sum);
      tmp[1*32 +:32] = real_to_fp32(a1/sum);
      tmp[2*32 +:32] = real_to_fp32(a2/sum);
      tmp[3*32 +:32] = real_to_fp32(a3/sum);
      tmp[4*32 +:32] = real_to_fp32(a4/sum);
      tmp[5*32 +:32] = real_to_fp32(a5/sum);
      tmp[6*32 +:32] = real_to_fp32(a6/sum);
      tmp[7*32 +:32] = real_to_fp32(a7/sum);
      tmp[8*32 +:32] = real_to_fp32(a8/sum);

      golden_w = tmp;
    end
  endfunction

  function [OUT_W-1:0] golden_out_vec(input [W_W-1:0] w_in, input [V_W-1:0] v_in);
    integer d, t;
    real acc, wr, vr;
    reg [OUT_W-1:0] tmp;
    begin
      tmp='0;
      for (d = 0; d < D; d = d + 1) begin
        acc = 0.0;
        for (t = 0; t < TOKENS; t = t + 1) begin
          wr = fp32_to_real(w_in[t*32 +: 32]);
          vr = fp32_to_real(v_in[(t*D + d)*32 +: 32]);
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
  // Scoreboard for outputs
  // ============================================================
  localparam int MAXQ = 4096;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  reg [31:0]      exp_a_q   [0:MAXQ-1];
  reg [tag_w-1:0] exp_t_q   [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [OUT_W-1:0] o, input [31:0] a, input [tag_w-1:0] tg);
    begin
      exp_out_q[sb_wr] = o;
      exp_a_q[sb_wr]   = a;
      exp_t_q[sb_wr]   = tg;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o, output [31:0] a, output [tag_w-1:0] tg);
    begin
      o  = exp_out_q[sb_rd];
      a  = exp_a_q[sb_rd];
      tg = exp_t_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // ============================================================
  // Random controls (out_ready) + drive center requests
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

  task drive_random_out_ready();
    integer warm;
    begin
      warm = 0;
      forever begin
        @(posedge clk);
        if (!rst_n || !en) begin
          out_ready <= 1'b0;
          warm      <= 0;
        end else begin
          if (warm < 200) begin
            out_ready <= 1'b1;
            warm      <= warm + 1;
          end else begin
            out_ready <= (urand(100) < 70);
          end
        end
      end
    end
  endtask

  integer tx_sent;

  task send_center_hold(input integer ci, input integer cj, input [tag_w-1:0] ctag);
    integer k;
    begin
      @(negedge clk);
      center_i     <= ci[$clog2(TILES_Y)-1:0];
      center_j     <= cj[$clog2(TILES_X)-1:0];
      center_tag   <= ctag;
      center_valid <= 1'b1;

      k = 0;
      while (!(go && center_ready)) begin
        @(posedge clk);
        k = k + 1;
        if (k > 100000) begin
          $display("[FATAL] timeout waiting center_ready");
          $fatal(1);
        end
      end

      @(negedge clk);
      center_valid <= 1'b0;
    end
  endtask

  task drive_centers();
    integer ci, cj;
    integer tmp_tag;
    reg [tag_w-1:0] ctag;
    begin
      center_valid <= 1'b0;
      tx_sent      <= 0;

      wait(go);

      // 先跑幾個 frame 讓 SRAM 有資料
      drive_one_frame(0);
      drive_one_frame(1);

      // 再送 center requests（DUT 會在 in_frame==0 時才 ready）
      repeat(50) @(posedge clk);

      while (tx_sent < TX_GOAL) begin
        ci = urand(TILES_Y);
        cj = urand(TILES_X);

        // group tag aligned (low IDX_W zeros)
        tmp_tag = urand(1 << (tag_w-IDX_W));
        ctag    = { tmp_tag[tag_w-IDX_W-1:0], {IDX_W{1'b0}} };

        send_center_hold(ci, cj, ctag);
        tx_sent <= tx_sent + 1;

        if (tx_sent <= PRINT_N)
          $display("[TB] TX_SENT=%0d center=(%0d,%0d) tag=%h time=%0t",
                   tx_sent, ci, cj, ctag, $time);
      end
    end
  endtask

  // ============================================================
  // Golden generation at attention input fire (hierarchical tap)
  // ============================================================
  wire attn_in_fire = go && dut.bundle_valid && dut.attn_in_ready;

  integer t;
  reg [V_W-1:0]   k_vecs, v_vecs;
  reg [D*32-1:0]  q_vec;
  reg [SCORE_W-1:0] sflat;
  reg [W_W-1:0]     wflat;
  reg [OUT_W-1:0]   outv;
  reg [31:0]        alph;
  reg [tag_w-1:0]   exp_tag;

  always @(posedge clk) begin
    if (!rst_n) begin
      // nothing
    end else if (attn_in_fire) begin
      // Build K/V from dut.kv[t] snapshot
      k_vecs = '0;
      v_vecs = '0;
      for (t=0; t<TOKENS; t=t+1) begin
        k_vecs[t*D*32 +: D*32] = dut.kv[t][D*32-1:0];
        v_vecs[t*D*32 +: D*32] = dut.kv[t][D*32-1:0];
      end

      // Q: center idx4
      q_vec = dut.kv[4][D*32-1:0];

      // Expected tag rule: {ctag_lat[tag_w-1:IDX_W], 0}
      exp_tag = { dut.ctag_lat[tag_w-1:IDX_W], {IDX_W{1'b0}} };

      sflat = golden_score(q_vec, k_vecs);
      wflat = golden_w(sflat);
      outv  = golden_out_vec(wflat, v_vecs);
      alph  = golden_alpha(outv[0*32 +: 32]);

      sb_push(outv, alph, exp_tag);
    end
  end

  // ============================================================
  // Output monitor + hold-under-stall + compare
  // ============================================================
  wire out_fire = go && out_valid && out_ready;

  integer rx_got, idle;
  reg [OUT_W-1:0] hold_out_prev;
  reg [31:0]      hold_a_prev;
  reg [tag_w-1:0] hold_t_prev;
  reg             hold_v;

  task monitor_and_check();
    reg [OUT_W-1:0] exp_out;
    reg [31:0]      exp_a;
    reg [tag_w-1:0] exp_t;
    integer d;
    real er, ar;
    begin
      rx_got = 0;
      idle   = 0;
      hold_v = 0;

      forever begin
        @(posedge clk);

        if (!go) begin
          hold_v <= 0;
          idle   <= 0;
        end else begin
          // hold under stall
          if (out_valid && !out_ready) begin
            if (hold_v) begin
              if (out_vec_dbg !== hold_out_prev) begin
                $display("[FAIL] out_vec changed under stall!");
                $display("  prev=%h", hold_out_prev);
                $display("  now =%h", out_vec_dbg);
                $fatal(1);
              end
              if (alpha_fp32 !== hold_a_prev) begin
                $display("[FAIL] alpha changed under stall!");
                $display("  prev=%h now=%h", hold_a_prev, alpha_fp32);
                $fatal(1);
              end
              if (TAG_EN && (out_tag !== hold_t_prev)) begin
                $display("[FAIL] out_tag changed under stall!");
                $display("  prev=%h now=%h", hold_t_prev, out_tag);
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

          if (out_fire) begin
            // X-guard
            if ((^out_vec_dbg) === 1'bX) begin
              $display("[FAIL] out_vec has X at out_fire time=%0t", $time);
              $fatal(1);
            end
            if ((^alpha_fp32) === 1'bX) begin
              $display("[FAIL] alpha has X at out_fire time=%0t", $time);
              $fatal(1);
            end
            if ((^out_tag) === 1'bX) begin
              $display("[FAIL] out_tag has X at out_fire time=%0t", $time);
              $fatal(1);
            end

            if (sb_cnt <= 0) begin
              $display("[FAIL] scoreboard underflow (unexpected output) time=%0t", $time);
              $fatal(1);
            end

            sb_pop(exp_out, exp_a, exp_t);

            if (out_vec_dbg !== exp_out) begin
              $display("[FAIL] out_vec mismatch @rx=%0d time=%0t", rx_got, $time);
              for (d=0; d<D; d=d+1) begin
                $display("  d=%0d exp=%h got=%h exp_r=%f got_r=%f",
                  d,
                  exp_out[d*32 +:32],
                  out_vec_dbg[d*32 +:32],
                  fp32_to_real(exp_out[d*32 +:32]),
                  fp32_to_real(out_vec_dbg[d*32 +:32])
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

            if (TAG_EN && (out_tag !== exp_t)) begin
              $display("[FAIL] tag mismatch @rx=%0d exp=%h got=%h", rx_got, exp_t, out_tag);
              $fatal(1);
            end

            if (rx_got < PRINT_N) begin
              $display("------------------------------------------------------------");
              $display("[TB] RX #%0d time=%0t tag=%h", rx_got, $time, out_tag);
              $display("  alpha=%f (bits=%h)", fp32_to_real(alpha_fp32), alpha_fp32);
              for (d=0; d<D; d=d+1)
                $display("  out[%0d]=%f", d, fp32_to_real(out_vec_dbg[d*32 +:32]));
            end

            rx_got <= rx_got + 1;
          end

          // done condition: all tx sent, scoreboard drained, output idle
          if ((tx_sent >= TX_GOAL) && (sb_cnt == 0) && !(out_valid))
            idle <= idle + 1;
          else
            idle <= 0;

          if (idle >= 400) begin
            $display("[TB] DONE (drained). rx=%0d wr_fire_seen=%0d", rx_got, wr_seen);
            $finish;
          end
        end
      end
    end
  endtask

  // ============================================================
  // Small monitor for wr_fire to ensure SRAM got written
  // ============================================================
  integer wr_seen;
  always @(posedge clk) begin
    if (!rst_n) wr_seen <= 0;
    else if (go && wr_fire) wr_seen <= wr_seen + 1;
  end

  // ============================================================
  // Init
  // ============================================================
  initial begin
    seed = 32'h1357_2468;

    vsync_i = 1'b0; hsync_i = 1'b0; de_i = 1'b0; rgb_i = 24'h0;
    clip_en = 1'b0;
    clip_min = '0;
    clip_max = {PACK_OUT_W{1'b1}};

    center_valid = 1'b0;
    center_i = '0; center_j = '0; center_tag = '0;

    out_ready = 1'b0;

    sb_wr = 0; sb_rd = 0; sb_cnt = 0;

    wait(rst_n==1'b1);
    repeat(10) @(posedge clk);

    fork
      drive_random_out_ready();
      drive_centers();
      monitor_and_check();
    join_any
  end

  // timeout guard
  initial begin
    integer k;
    k=0;
    while (k < TIMEOUT_C) begin
      @(posedge clk);
      k=k+1;
    end
    $display("[FATAL] TIMEOUT");
    $fatal(1);
  end

endmodule

`default_nettype wire
