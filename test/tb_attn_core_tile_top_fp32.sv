// ============================================================
// tb_attn_core_tile_top_fp32.sv  (IVERILOG-SAFE / NO automatic)
// - Test: attn_core_tile_top_fp32 (M5 top: M1->M2->M3->M4 + tag fifo)
// - Golden (same as tb_attn_m1_m4_chain_top_pipe):
//    score[t]    = dot(Q,K[t]) / sqrt(D)
//    w[t]        = softmax_exp2_approx(score)   (exp2 approx + clamp)
//    out_vec[d]  = sum_t w[t] * V[t][d]
//    alpha       = g(out_vec[0])                (MODE selectable)
//
// - Checks:
//   1) out_vec_dbg matches golden out_vec
//   2) alpha_fp32  matches golden alpha
//   3) out_tag     matches input tag order (TAG_EN=1)
//   4) hold under stall: out_valid && !out_ready => out_vec_dbg/alpha/out_tag stable
//
// Build example (adjust paths):
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_m5 ./test/tb_attn_core_tile_top_fp32.sv
//
// vvp ./vvp/tb_m5
// gtkwave tb_attn_core_tile_top_fp32.vcd
// ============================================================
`include "./src/AMOLED/atten_core/attn_core_tile_top_fp32.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_attn_core_tile_top_fp32;

  // ----------------------------
  // Params (match DUT defaults)
  // ----------------------------
  localparam int unsigned TOKENS = 9;
  localparam int unsigned D      = 8;

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
  localparam int unsigned TAG_W          = 16;
  localparam int unsigned TAG_FIFO_DEPTH = 32;

  localparam int unsigned SCORE_W = TOKENS*32;
  localparam int unsigned W_W     = TOKENS*32;
  localparam int unsigned V_W     = TOKENS*D*32;
  localparam int unsigned OUT_W   = D*32;

  localparam int TX_GOAL = 300;
  localparam int PRINT_N = 8;

  // M2 approx constants (match your M2 TB)
  localparam real LOG2E  = 1.4426950408889634;
  localparam real LN2    = 0.6931471805599453;
  localparam real XCLAMP = 8.0;

  // ----------------------------
  // clk/rst
  // ----------------------------
  reg clk, rst_n;
  initial begin clk=1'b0; forever #5 clk=~clk; end
  initial begin rst_n=1'b0; repeat(6) @(posedge clk); rst_n=1'b1; end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                      in_valid;
  wire                     in_ready;
  reg  [D*32-1:0]           q_vec;
  reg  [TOKENS*D*32-1:0]    k_vecs;
  reg  [TOKENS*D*32-1:0]    v_vecs;
  reg  [TAG_W-1:0]          in_tag;

  wire                     out_valid;
  reg                      out_ready;
  wire [31:0]               alpha_fp32;
  wire [OUT_W-1:0]          out_vec_dbg;
  wire [TAG_W-1:0]          out_tag;

  wire [SCORE_W-1:0]        score_flat_dbg;
  wire [W_W-1:0]            w_flat_dbg;

  attn_core_tile_top_fp32 #(
    .TOKENS         (TOKENS),
    .D              (D),
    .M1_PIPE_STAGES (M1_PS),
    .M2_PIPE_STAGES (M2_PS),
    .M3_PIPE_STG    (M3_PS),
    .M4_PIPE_STG    (M4_PS),
    .W_PIPE_STAGES  (W_PIPE_STAGES),
    .V_FIFO_DEPTH   (V_FIFO_DEPTH),
    .ALPHA_MODE     (ALPHA_MODE),
    .ALPHA_SCALE    (ALPHA_SCALE),
    .ALPHA_BIAS     (ALPHA_BIAS),
    .ALPHA_SIG_A    (ALPHA_SIG_A),
    .TAG_EN         (TAG_EN),
    .TAG_W          (TAG_W),
    .TAG_FIFO_DEPTH (TAG_FIFO_DEPTH)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .in_valid(in_valid),
    .in_ready(in_ready),
    .q_vec(q_vec),
    .k_vecs(k_vecs),
    .v_vecs(v_vecs),
    .in_tag(in_tag),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .alpha_fp32(alpha_fp32),
    .out_vec_dbg(out_vec_dbg),
    .out_tag(out_tag),

    .score_flat_dbg(score_flat_dbg),
    .w_flat_dbg(w_flat_dbg)
  );

  // ----------------------------
  // pow2i + fp32 <-> real
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
  // exp2 approx (same as M2)
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
  // golden: score, w, out_vec, alpha
  // ----------------------------
  function [SCORE_W-1:0] golden_score(
    input [D*32-1:0] q,
    input [TOKENS*D*32-1:0] k
  );
    integer t,d;
    real acc, invs;
    reg [SCORE_W-1:0] tmp;
    begin
      invs = 1.0 / $sqrt(D);
      tmp  = '0;
      for(t=0;t<TOKENS;t=t+1) begin
        acc=0.0;
        for(d=0;d<D;d=d+1)
          acc = acc
              + fp32_to_real(q[d*32 +:32])
              * fp32_to_real(k[(t*D+d)*32 +:32]);
        acc = acc * invs;
        tmp[t*32 +:32] = real_to_fp32(acc);
      end
      golden_score = tmp;
    end
  endfunction

  function [W_W-1:0] golden_w(input [SCORE_W-1:0] sflat);
    integer t;
    real s0, s1, s2, s3, s4, s5, s6, s7, s8;
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
      tmp = '0;
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
      if (x < 0.0) clamp01 = 0.0;
      else if (x > 1.0) clamp01 = 1.0;
      else clamp01 = x;
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
  // Scoreboard: expected {out_vec, alpha, tag}
  // push on input fire, pop on output consume
  // ----------------------------
  localparam int MAXQ=4096;
  reg [OUT_W-1:0] exp_out_q [0:MAXQ-1];
  reg [31:0]      exp_a_q   [0:MAXQ-1];
  reg [TAG_W-1:0] exp_tag_q [0:MAXQ-1];
  integer wr, rd, cnt;

  task sb_push(input [OUT_W-1:0] o, input [31:0] a, input [TAG_W-1:0] tg);
    begin
      exp_out_q[wr] = o;
      exp_a_q[wr]   = a;
      exp_tag_q[wr] = tg;
      wr  = (wr + 1) % MAXQ;
      cnt = cnt + 1;
    end
  endtask

  task sb_pop(output [OUT_W-1:0] o, output [31:0] a, output [TAG_W-1:0] tg);
    begin
      o  = exp_out_q[rd];
      a  = exp_a_q[rd];
      tg = exp_tag_q[rd];
      rd  = (rd + 1) % MAXQ;
      cnt = cnt - 1;
    end
  endtask

  // ----------------------------
  // Hold-under-stall check
  // ----------------------------
  reg [OUT_W-1:0] hold_out_prev;
  reg [31:0]      hold_a_prev;
  reg [TAG_W-1:0] hold_tag_prev;
  reg             hold_v;

  // ----------------------------
  // Main control
  // ----------------------------
  integer tx, rx, idle;

  // temps for golden
  reg [SCORE_W-1:0] s_tmp;
  reg [W_W-1:0]     w_tmp;
  reg [OUT_W-1:0]   out_tmp;
  reg [31:0]        a_tmp;

  // tag counter
  reg [TAG_W-1:0] tag_ctr;

  // ----------------------------
  // Random generators
  // ----------------------------
  function [31:0] rand_fp32_range(input integer lo100, input integer hi100);
    integer r; real x;
    begin
      r = lo100 + ($urandom() % (hi100-lo100+1));
      x = r / 100.0;
      rand_fp32_range = real_to_fp32(x);
    end
  endfunction

  // ----------------------------
  // Test init
  // ----------------------------
  initial begin
    in_valid  = 1'b0;
    out_ready = 1'b1;
    q_vec     = '0;
    k_vecs    = '0;
    v_vecs    = '0;
    in_tag    = '0;

    wr=0; rd=0; cnt=0;
    tx=0; rx=0; idle=0;
    hold_out_prev='0; hold_a_prev=32'h0; hold_tag_prev='0; hold_v=1'b0;
    tag_ctr = '0;

    $dumpfile("tb_attn_core_tile_top_fp32.vcd");
    $dumpvars(0, tb_attn_core_tile_top_fp32);

    @(posedge rst_n);
    repeat(3) @(posedge clk);

    fork
      drive_in();
      drive_out_ready();
      monitor();
    join_any
    disable fork;

    $display("[TB] DONE");
    $finish;
  end

  // ----------------------------
  // Drive input (respect in_ready)
  // ----------------------------
  task drive_in();
    integer t,d;
    begin
      while (tx < TX_GOAL) begin
        @(negedge clk);

        if (in_ready && (($urandom()%100) < 70)) begin
          // Q, K, V in moderate range [-2..2]
          for (d=0; d<D; d=d+1)
            q_vec[d*32 +:32] <= rand_fp32_range(-200, 200);

          for (t=0; t<TOKENS; t=t+1) begin
            for (d=0; d<D; d=d+1) begin
              k_vecs[(t*D+d)*32 +:32] <= rand_fp32_range(-200, 200);
              // V: let dim0 be slightly wider sometimes [-6..6]
              if (d==0) v_vecs[(t*D+d)*32 +:32] <= rand_fp32_range(-600, 600);
              else      v_vecs[(t*D+d)*32 +:32] <= rand_fp32_range(-200, 200);
            end
          end

          in_tag   <= tag_ctr;
          in_valid <= 1'b1;
        end else begin
          in_valid <= 1'b0;
        end

        @(posedge clk);
        if (rst_n && in_valid && in_ready) begin
          s_tmp   = golden_score(q_vec, k_vecs);
          w_tmp   = golden_w(s_tmp);
          out_tmp = golden_out_vec(w_tmp, v_vecs);
          a_tmp   = golden_alpha(out_tmp[0*32 +:32]);
          sb_push(out_tmp, a_tmp, in_tag);

          tx      = tx + 1;
          tag_ctr = tag_ctr + 1;
        end
      end

      in_valid <= 1'b0;
    end
  endtask

  // ----------------------------
  // Random backpressure on output
  // ----------------------------
  task drive_out_ready();
    begin
      forever begin
        @(posedge clk);
        if (!rst_n) out_ready <= 1'b0;
        else       out_ready <= (($urandom()%100) < 70);
      end
    end
  endtask

  // ----------------------------
  // Monitor/check
  // ----------------------------
  task monitor();
    reg [OUT_W-1:0] exp_out;
    reg [31:0]      exp_a;
    reg [TAG_W-1:0] exp_tg;
    integer d;
    real ar, er;
    begin
      forever begin
        @(posedge clk);

        if (!rst_n) begin
          hold_v <= 1'b0;
          idle   <= 0;
        end else begin
          // hold under stall
          if (out_valid && !out_ready) begin
            if (hold_v) begin
              if (out_vec_dbg !== hold_out_prev) begin
                $display("[TB][FAIL] out_vec_dbg changed under stall!");
                $display("  prev=%h", hold_out_prev);
                $display("  now =%h", out_vec_dbg);
                $finish;
              end
              if (alpha_fp32 !== hold_a_prev) begin
                $display("[TB][FAIL] alpha_fp32 changed under stall!");
                $display("  prev=%h now=%h", hold_a_prev, alpha_fp32);
                $finish;
              end
              if (TAG_EN && (out_tag !== hold_tag_prev)) begin
                $display("[TB][FAIL] out_tag changed under stall!");
                $display("  prev=%h now=%h", hold_tag_prev, out_tag);
                $finish;
              end
            end
            hold_out_prev <= out_vec_dbg;
            hold_a_prev   <= alpha_fp32;
            hold_tag_prev <= out_tag;
            hold_v        <= 1'b1;
          end else begin
            hold_v <= 1'b0;
          end

          // consume
          if (out_valid && out_ready) begin
            sb_pop(exp_out, exp_a, exp_tg);

            if (out_vec_dbg !== exp_out) begin
              $display("[TB][FAIL] out_vec mismatch!");
              for (d=0; d<D; d=d+1) begin
                $display("  d=%0d exp=%h got=%h exp_r=%f got_r=%f",
                  d,
                  exp_out[d*32 +:32],
                  out_vec_dbg[d*32 +:32],
                  fp32_to_real(exp_out[d*32 +:32]),
                  fp32_to_real(out_vec_dbg[d*32 +:32])
                );
              end
              $finish;
            end

            if (alpha_fp32 !== exp_a) begin
              er = fp32_to_real(exp_a);
              ar = fp32_to_real(alpha_fp32);
              $display("[TB][FAIL] alpha mismatch! exp=%h (%f) got=%h (%f)",
                       exp_a, er, alpha_fp32, ar);
              $finish;
            end

            if (TAG_EN && (out_tag !== exp_tg)) begin
              $display("[TB][FAIL] tag mismatch! exp=%h got=%h", exp_tg, out_tag);
              $finish;
            end

            if (rx < PRINT_N) begin
              $display("------------------------------------------------------------");
              $display("[TB] RX #%0d time=%0t tag=%h", rx, $time, out_tag);
              $display("  alpha=%f (bits=%h)", fp32_to_real(alpha_fp32), alpha_fp32);
              for (d=0; d<D; d=d+1)
                $display("  out[%0d]=%f", d, fp32_to_real(out_vec_dbg[d*32 +:32]));
            end

            rx = rx + 1;
          end

          // stop when drained
          if ((tx >= TX_GOAL) && (cnt == 0) && !(in_valid && !in_ready))
            idle = idle + 1;
          else
            idle = 0;

          if (idle >= 80) begin
            $display("[TB] DONE");
            $finish;
          end
        end
      end
    end
  endtask

endmodule

`default_nettype wire
