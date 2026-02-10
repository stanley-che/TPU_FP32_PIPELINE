// ============================================================
// tb_tile_nb_fetch_bundle9_attn_core_m5_top_fp32.sv (IVERILOG-SAFE)
// - DUT: tile_nb_fetch_bundle9_attn_core_m5_top_fp32
//   (fetch 3x3 bundle9) -> (M5 attention core) -> alpha/out/tag
//
// - SRAM model:
//   mem_rd_valid/ready -> (READ_LATENCY pipe) -> queue -> mem_rvalid bubbles optional
//   mem_rdata = data_fn(full_word)  (same style as your fetch TB)
//
// - Golden (same as your M5 TB):
//   Build 9 tokens (kv0..kv8) from tile-neighborhood mapping + SRAM data_fn
//   Q = (Q_FROM_CENTER_IDX4 ? kv4 : q_vec from assembler)   // here we model q=kv4 when enabled
//   K = kv tokens (D*FP32 each)
//   V = kv tokens (same as K by default in integrated top)
//
//   score[t]    = dot(Q,K[t]) / sqrt(D)
//   w[t]        = softmax_exp2_approx(score)  (exp2 approx + clamp)
//   out_vec[d]  = sum_t w[t] * V[t][d]
//   alpha       = g(out_vec[0])               (MODE selectable)
//
// - Checks:
//   1) out_vec_dbg matches golden out_vec
//   2) alpha_fp32  matches golden alpha
//   3) out_tag matches expected group tag (center_tag[TAG_W-1:IDX_W] << IDX_W)
//   4) hold under stall: out_valid && !out_ready => out_vec/alpha/out_tag stable
//
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_m5_int ./test/tb_tile_nb_fetch_bundle9_attn_core_m5_top_fp32.sv
//
// Run:
// vvp ./vvp/tb_m5_int
// gtkwave tb_m5_int.vcd
// ============================================================
`include "./src/AMOLED/tile_nb_fetch_bundle9_attn_core_m5_top_fp32.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_tile_nb_fetch_bundle9_attn_core_m5_top_fp32;

  // ----------------------------
  // Small TB params
  // ----------------------------
  localparam int unsigned TILES_X        = 8;
  localparam int unsigned TILES_Y        = 4;

  localparam int unsigned TAG_W          = 16;
  localparam int unsigned IDX_W          = 4;

  localparam int unsigned FEAT_W         = 256;
  localparam int unsigned MEM_W          = 64;
  localparam int unsigned BASE_ADDR_WORD = 100;

  localparam int unsigned BANKS          = 4;
  localparam int unsigned ADDR_W         = 32;
  localparam int unsigned READ_LATENCY   = 1;
  localparam bit          USE_RTAG       = 0;

  localparam bit          Q_FROM_CENTER_IDX4 = 1'b1;

  // Attention params (must satisfy FEAT_W == D*32)
  localparam int unsigned TOKENS = 9;
  localparam int unsigned D      = 8;   // 8*32=256

  localparam int unsigned M1_PS  = 2;
  localparam int unsigned M2_PS  = 1;
  localparam int unsigned M3_PS  = 1;
  localparam int unsigned M4_PS  = 1;

  localparam int unsigned W_PIPE_STAGES = 1;
  localparam int unsigned V_FIFO_DEPTH  = 16;

  localparam int unsigned ALPHA_MODE  = 1;
  localparam real         ALPHA_SCALE = 1.0;
  localparam real         ALPHA_BIAS  = 0.0;
  localparam real         ALPHA_SIG_A = 1.0;

  localparam bit          TAG_EN         = 1'b1;
  localparam int unsigned TAG_FIFO_DEPTH = 32;

  localparam int unsigned BANK_W = (BANKS <= 1) ? 1 : $clog2(BANKS);
  localparam int unsigned IW     = (TILES_Y <= 1) ? 1 : $clog2(TILES_Y);
  localparam int unsigned JW     = (TILES_X <= 1) ? 1 : $clog2(TILES_X);

  localparam int unsigned SCORE_W = TOKENS*32;
  localparam int unsigned W_W     = TOKENS*32;
  localparam int unsigned V_W     = TOKENS*D*32;
  localparam int unsigned OUT_W   = D*32;

  localparam int unsigned BEATS   = (FEAT_W + MEM_W - 1) / MEM_W;

  localparam int TX_GOAL   = 120;
  localparam int PRINT_N   = 10;
  localparam int TIMEOUT_C = 200000;

  // M2 approx constants
  localparam real LOG2E  = 1.4426950408889634;
  localparam real LN2    = 0.6931471805599453;
  localparam real XCLAMP = 8.0;

  // ----------------------------
  // clk/rst/en
  // ----------------------------
  reg clk, rst_n, en;
  initial begin clk=1'b0; forever #5 clk=~clk; end
  initial begin
    rst_n = 1'b0;
    en    = 1'b0;
    repeat(8) @(posedge clk);
    en    = 1'b1;
    rst_n = 1'b1;
  end

  wire go = en && rst_n;

  // ----------------------------
  // DUT ports
  // ----------------------------
  reg                      center_valid;
  wire                     center_ready;
  reg  [IW-1:0]             center_i;
  reg  [JW-1:0]             center_j;
  reg  [TAG_W-1:0]          center_tag;

  wire                     mem_rd_valid;
  reg                      mem_rd_ready;
  wire [ADDR_W-1:0]         mem_addr;
  wire [BANK_W-1:0]         mem_bank;
  wire [TAG_W-1:0]          mem_tag;

  reg                      mem_rvalid;
  reg  [MEM_W-1:0]          mem_rdata;
  reg  [TAG_W-1:0]          mem_rtag;

  wire                     out_valid;
  reg                      out_ready;
  wire [31:0]               alpha_fp32;
  wire [OUT_W-1:0]          out_vec_dbg;
  wire [TAG_W-1:0]          out_tag;

  wire                     sched_busy;
  wire                     sched_done_pulse;
  wire                     bundle_done_pulse;
  wire [8:0]                nb_is_center;

  wire [TOKENS*32-1:0]      score_flat_dbg;
  wire [TOKENS*32-1:0]      w_flat_dbg;

  tile_nb_fetch_bundle9_attn_core_m5_top_fp32 #(
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
    .Q_FROM_CENTER_IDX4(Q_FROM_CENTER_IDX4),

    .TOKENS(TOKENS),
    .D(D),

    .M1_PIPE_STAGES(M1_PS),
    .M2_PIPE_STAGES(M2_PS),
    .M3_PIPE_STG(M3_PS),
    .M4_PIPE_STG(M4_PS),

    .W_PIPE_STAGES(W_PIPE_STAGES),
    .V_FIFO_DEPTH(V_FIFO_DEPTH),

    .ALPHA_MODE(ALPHA_MODE),
    .ALPHA_SCALE(ALPHA_SCALE),
    .ALPHA_BIAS(ALPHA_BIAS),
    .ALPHA_SIG_A(ALPHA_SIG_A),

    .TAG_EN(TAG_EN),
    .TAG_FIFO_DEPTH(TAG_FIFO_DEPTH)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .en(en),

    .center_valid(center_valid),
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

    .out_valid(out_valid),
    .out_ready(out_ready),
    .alpha_fp32(alpha_fp32),
    .out_vec_dbg(out_vec_dbg),
    .out_tag(out_tag),

    .sched_busy(sched_busy),
    .sched_done_pulse(sched_done_pulse),
    .bundle_done_pulse(bundle_done_pulse),
    .nb_is_center(nb_is_center),

    .score_flat_dbg(score_flat_dbg),
    .w_flat_dbg(w_flat_dbg)
  );

  // ----------------------------
  // VCD
  // ----------------------------
  initial begin
    $dumpfile("tb_m5_int.vcd");
    $dumpvars(0, tb_tile_nb_fetch_bundle9_attn_core_m5_top_fp32);
  end

  // ----------------------------
  // Helpers: clamp + neighbor mapping
  // ----------------------------
  function integer clamp_int(input integer v, input integer lo, input integer hi);
    begin
      if (v < lo) clamp_int = lo;
      else if (v > hi) clamp_int = hi;
      else clamp_int = v;
    end
  endfunction

  task token_to_tile(
    input integer ci, input integer cj, input integer token,
    output integer ti, output integer tj
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

  function [TAG_W-1:0] exp_out_tag_from_center(input [TAG_W-1:0] ctag);
    begin
      exp_out_tag_from_center = {ctag[TAG_W-1:IDX_W], {IDX_W{1'b0}}};
    end
  endfunction

  // ----------------------------
  // FP32 helpers
  // ----------------------------
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

  // ----------------------------
  // SRAM model: address/data (FP32)
  // ----------------------------
  function [63:0] full_word_from_bank_addr(input [BANK_W-1:0] b, input [ADDR_W-1:0] a);
    begin
      if (BANKS <= 1) full_word_from_bank_addr = 64'(a);
      else            full_word_from_bank_addr = (64'(a) << BANK_W) | 64'(b);
    end
  endfunction

  function [63:0] base_full_word_for_tile(input integer ti, input integer tj);
    integer linear;
    begin
      linear = ti*TILES_X + tj;
      base_full_word_for_tile = 64'(BASE_ADDR_WORD) + 64'(linear) * 64'(BEATS);
    end
  endfunction

  // 32-bit hash
  function automatic [31:0] hash32(input [63:0] x);
    reg [31:0] h;
    begin
      h = x[31:0] ^ x[63:32] ^ 32'h9E37_79B9;
      h = h ^ (h >> 16);
      h = h * 32'h7FEB_352D;
      h = h ^ (h >> 15);
      h = h * 32'h846C_A68B;
      h = h ^ (h >> 16);
      hash32 = h;
    end
  endfunction

  function automatic real u32_to_unit_signed(input [31:0] u);
    real v;
    begin
      v = u / 4294967296.0;   // [0,1)
      u32_to_unit_signed = (v * 2.0) - 1.0; // [-1,1)
    end
  endfunction

  function automatic [31:0] fp32_from_seed(input [31:0] s);
    real r;
    begin
      r = u32_to_unit_signed(s);
      fp32_from_seed = real_to_fp32(r);
    end
  endfunction

  function automatic [MEM_W-1:0] data_fn_fp32(input [63:0] full_word);
    integer lane;
    reg [MEM_W-1:0] w;
    reg [31:0] h;
    begin
      w = '0;
      for (lane = 0; lane < (MEM_W/32); lane = lane + 1) begin
        h = hash32(full_word ^ 64'(lane) ^ 64'hA5A5_0000_0000_5A5A);
        w[lane*32 +: 32] = fp32_from_seed(h);
      end
      data_fn_fp32 = w;
    end
  endfunction

  // build one FEAT_W vector from base_full (uses SAME data_fn_fp32)
  task build_expected_feat(input [63:0] base_full, output reg [FEAT_W-1:0] exp);
    integer b;
    begin
      exp = '0;
      for (b = 0; b < BEATS; b = b + 1)
        exp[b*MEM_W +: MEM_W] = data_fn_fp32(base_full + 64'(b));
    end
  endtask

  // ----------------------------
  // exp2 approx
  // ----------------------------
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

  // ----------------------------
  // golden attention
  // ----------------------------
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

  // (IVERILOG-safe scalar version)
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

  // ----------------------------
  // Build golden bundle (Q/K/V) from (center_i,j)
  // ----------------------------
  task build_kv_from_center(
    input integer ci, input integer cj,
    output reg [V_W-1:0] k_vecs,
    output reg [V_W-1:0] v_vecs,
    output reg [D*32-1:0] q_vec
  );
    integer t, ti, tj;
    reg [FEAT_W-1:0] feat;
    begin
      k_vecs = '0;
      v_vecs = '0;
      q_vec  = '0;

      for (t=0;t<TOKENS;t=t+1) begin
        token_to_tile(ci,cj,t,ti,tj);
        build_expected_feat(base_full_word_for_tile(ti,tj), feat);
        k_vecs[t*D*32 +: D*32] = feat[D*32-1:0];
        v_vecs[t*D*32 +: D*32] = feat[D*32-1:0];
        if (Q_FROM_CENTER_IDX4 && (t==4)) q_vec = feat[D*32-1:0];
      end

      if (!Q_FROM_CENTER_IDX4) begin
        token_to_tile(ci,cj,4,ti,tj);
        build_expected_feat(base_full_word_for_tile(ti,tj), feat);
        q_vec = feat[D*32-1:0];
      end
    end
  endtask

  // ----------------------------
  // Scoreboard
  // ----------------------------
  localparam int MAXQ = 4096;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  reg [31:0]      exp_a_q   [0:MAXQ-1];
  reg [TAG_W-1:0] exp_t_q   [0:MAXQ-1];
  integer sb_wr, sb_rd, sb_cnt;

  task sb_push(input [OUT_W-1:0] o, input [31:0] a, input [TAG_W-1:0] tg);
    begin
      exp_out_q[sb_wr] = o;
      exp_a_q[sb_wr]   = a;
      exp_t_q[sb_wr]   = tg;
      sb_wr  = (sb_wr + 1) % MAXQ;
      sb_cnt = sb_cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o, output [31:0] a, output [TAG_W-1:0] tg);
    begin
      o  = exp_out_q[sb_rd];
      a  = exp_a_q[sb_rd];
      tg = exp_t_q[sb_rd];
      sb_rd  = (sb_rd + 1) % MAXQ;
      sb_cnt = sb_cnt - 1;
    end
  endtask

  // ----------------------------
  // Handshakes + stall-hold checker
  // ----------------------------
  wire center_fire = go && center_valid && center_ready;
  wire out_fire    = go && out_valid && out_ready;

  reg [OUT_W-1:0] hold_out_prev;
  reg [31:0]      hold_a_prev;
  reg [TAG_W-1:0] hold_t_prev;
  reg             hold_v;

  // ----------------------------
  // Random control
  // ----------------------------
  integer seed;
  function automatic int urand(input int mod);
    int r;
    begin
      r = $random(seed);
      if (r < 0) r = -r;
      urand = (mod==0) ? 0 : (r % mod);
    end
  endfunction

  // ----------------------------
  // SRAM queue + latency pipe
  // ----------------------------
  reg [63:0] addr_pipe [0:READ_LATENCY];
  reg        v_pipe    [0:READ_LATENCY];
  integer    ii;

  localparam int QDEP = 1024;
  reg [63:0] q_full [0:QDEP-1];
  integer q_wptr, q_rptr, q_count;

  task q_push(input [63:0] fw);
    begin
      if (q_count >= QDEP) begin
        $display("[FATAL] SRAM Q overflow");
        $fatal(1);
      end
      q_full[q_wptr] = fw;
      q_wptr = (q_wptr + 1) % QDEP;
      q_count = q_count + 1;
    end
  endtask

  task q_pop(output [63:0] fw);
    begin
      if (q_count <= 0) begin
        $display("[FATAL] SRAM Q underflow");
        $fatal(1);
      end
      fw = q_full[q_rptr];
      q_rptr = (q_rptr + 1) % QDEP;
      q_count = q_count - 1;
    end
  endtask

  reg bubble_en;
  reg [63:0] pop_fw;

  wire mem_fire = go && mem_rd_valid && mem_rd_ready;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (ii=0; ii<=READ_LATENCY; ii=ii+1) begin
        addr_pipe[ii] <= 64'd0;
        v_pipe[ii]    <= 1'b0;
      end
      q_wptr  <= 0;
      q_rptr  <= 0;
      q_count <= 0;

      mem_rvalid <= 1'b0;
      mem_rdata  <= '0;
      mem_rtag   <= '0;
    end else if (en) begin
      for (ii=READ_LATENCY; ii>0; ii=ii-1) begin
        addr_pipe[ii] <= addr_pipe[ii-1];
        v_pipe[ii]    <= v_pipe[ii-1];
      end

      addr_pipe[0] <= full_word_from_bank_addr(mem_bank, mem_addr);
      v_pipe[0]    <= mem_fire;

      if (v_pipe[READ_LATENCY]) q_push(addr_pipe[READ_LATENCY]);

      mem_rvalid <= 1'b0;

      if (q_count > 0) begin
        if (bubble_en && (urand(4)==0)) begin
          mem_rvalid <= 1'b0;
        end else begin
          q_pop(pop_fw);
          mem_rvalid <= 1'b1;
          mem_rdata  <= data_fn_fp32(pop_fw);
          mem_rtag   <= mem_tag; // ok when USE_RTAG=0
        end
      end
    end
  end

  // ----------------------------
  // Drive mem_rd_ready / out_ready randomness
  // ----------------------------
  task drive_random_controls();
    integer warm;
    begin
      warm = 0;
      forever begin
        @(posedge clk);
        if (!rst_n || !en) begin
          mem_rd_ready <= 1'b0;
          out_ready    <= 1'b0;
          bubble_en    <= 1'b0;
          warm         <= 0;
        end else begin
          if (warm < 200) begin
            mem_rd_ready <= 1'b1;
            out_ready    <= 1'b1;
            bubble_en    <= 1'b0;
            warm         <= warm + 1;
          end else begin
            mem_rd_ready <= (urand(100) < 75);
            out_ready    <= (urand(100) < 70);
            bubble_en    <= (urand(100) < 35);
          end
        end
      end
    end
  endtask

  // ----------------------------
  // Drive center requests + push golden into scoreboard
  // ----------------------------
  integer tx_sent;

  task send_center_hold(input integer ci, input integer cj, input [TAG_W-1:0] ctag);
    integer k;
    begin
      @(negedge clk);
      center_i     <= ci[IW-1:0];
      center_j     <= cj[JW-1:0];
      center_tag   <= ctag;
      center_valid <= 1'b1;

      k = 0;
      while (!(go && center_ready)) begin
        @(posedge clk);
        k = k + 1;
        if (k > 50000) begin
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
    reg [TAG_W-1:0] ctag;
    integer tmp_tag;

    reg [V_W-1:0]     k_vecs, v_vecs;
    reg [D*32-1:0]    q_vec;
    reg [SCORE_W-1:0] sflat;
    reg [W_W-1:0]     wflat;
    reg [OUT_W-1:0]   outv;
    reg [31:0]        alph;
    begin
      center_valid <= 1'b0;
      tx_sent      <= 0;

      wait(go);
      repeat(20) @(posedge clk);

      while (tx_sent < TX_GOAL) begin
        ci = urand(TILES_Y);
        cj = urand(TILES_X);

        tmp_tag = urand(1 << (TAG_W-IDX_W));
        ctag    = { tmp_tag[TAG_W-IDX_W-1:0], {IDX_W{1'b0}} };

        send_center_hold(ci, cj, ctag);

        build_kv_from_center(ci, cj, k_vecs, v_vecs, q_vec);
        sflat = golden_score(q_vec, k_vecs);
        wflat = golden_w(sflat);
        outv  = golden_out_vec(wflat, v_vecs);
        alph  = golden_alpha(outv[0*32 +: 32]);

        sb_push(outv, alph, exp_out_tag_from_center(ctag));
        tx_sent <= tx_sent + 1;

        if (tx_sent < PRINT_N)
          $display("[TB] TX_SENT=%0d center=(%0d,%0d) tag=%h time=%0t", tx_sent, ci, cj, ctag, $time);
      end
    end
  endtask

  // ----------------------------
  // Monitor outputs + check
  // ----------------------------
  integer rx_got, idle;

  task monitor_and_check();
    reg [OUT_W-1:0] exp_out;
    reg [31:0]      exp_a;
    reg [TAG_W-1:0] exp_t;
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

          if ((tx_sent >= TX_GOAL) && (sb_cnt == 0) && !(out_valid))
            idle <= idle + 1;
          else
            idle <= 0;

          if (idle >= 200) begin
            $display("[TB] DONE (drained). rx=%0d", rx_got);
            $finish;
          end
        end
      end
    end
  endtask

  // ----------------------------
  // Init + run
  // ----------------------------
  initial begin
    seed = 32'h1357_2468;

    center_valid = 1'b0;
    center_i     = '0;
    center_j     = '0;
    center_tag   = '0;

    mem_rd_ready = 1'b0;
    out_ready    = 1'b0;
    bubble_en    = 1'b0;

    sb_wr=0; sb_rd=0; sb_cnt=0;

    wait(rst_n==1'b1);
    repeat(10) @(posedge clk);
    if (nb_is_center !== 9'b000010000) begin
      $display("[WARN] nb_is_center=%b (expected 000010000)", nb_is_center);
    end

    fork
      drive_random_controls();
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
