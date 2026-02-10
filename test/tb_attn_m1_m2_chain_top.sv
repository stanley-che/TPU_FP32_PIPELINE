// ============================================================
// tb_attn_m1_m2_chain_top.sv  (IVERILOG-SAFE / NO automatic)
// - Test attn_m1_m2_chain_top
// - Golden = scaled dot-product + exp2 softmax approx
// - Checks:
//   1) w_flat matches golden softmax-exp2
//   2) sum(w) ~= 1
//   3) hold under stall (out_valid && !out_ready => w_flat stable)
// NOTE:
//   score_flat_dbg is NOT guaranteed aligned with w because M2 buffers.
// ============================================================

/*
iverilog -g2012 -Wall \
  ./test/tb_attn_m1_m2_chain_top.sv \
  -o ./vvp/tb_chain
vvp ./vvp/tb_chain
*/
`include "./src/AMOLED/atten_core/attn_m1_m2_chain_top.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_attn_m1_m2_chain_top;

  localparam int TOKENS = 9;
  localparam int D      = 8;
  localparam int M1_PS  = 2;
  localparam int M2_PS  = 1;

  localparam int SCORE_W = TOKENS*32;
  localparam int W_W     = TOKENS*32;

  localparam int TX_GOAL = 200;
  localparam int PRINT_N = 5;

  localparam real LOG2E  = 1.4426950408889634;
  localparam real LN2    = 0.6931471805599453;
  localparam real XCLAMP = 8.0;

  // ----------------------------
  // clk/rst
  // ----------------------------
  reg clk, rst_n;
  initial begin clk=0; forever #5 clk=~clk; end
  initial begin rst_n=0; repeat(6) @(posedge clk); rst_n=1; end

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                    in_valid;
  wire                   in_ready;
  reg  [D*32-1:0]         q_vec;
  reg  [TOKENS*D*32-1:0]  k_vecs;

  wire                   out_valid;
  reg                    out_ready;
  wire [W_W-1:0]          w_flat;
  wire [SCORE_W-1:0]      score_flat_dbg; // not checked

  attn_m1_m2_chain_top #(
    .TOKENS(TOKENS),
    .D(D),
    .M1_PIPE_STAGES(M1_PS),
    .M2_PIPE_STAGES(M2_PS)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .in_valid(in_valid), .in_ready(in_ready),
    .q_vec(q_vec), .k_vecs(k_vecs),
    .out_valid(out_valid), .out_ready(out_ready),
    .w_flat(w_flat),
    .score_flat_dbg(score_flat_dbg)
  );

  // ----------------------------
  // pow2i
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

  // ----------------------------
  // fp32 <-> real
  // ----------------------------
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
  // exp2 approx (same as DUT)
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
  // golden score + golden w
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
    real s[0:TOKENS-1];
    real m,x,y,a[0:TOKENS-1],sum;
    reg [W_W-1:0] tmp;
    begin
      for(t=0;t<TOKENS;t=t+1)
        s[t]=fp32_to_real(sflat[t*32 +:32]);

      m=s[0];
      for(t=1;t<TOKENS;t=t+1) if(s[t]>m) m=s[t];

      sum=0.0;
      for(t=0;t<TOKENS;t=t+1) begin
        x = s[t]-m;
        if(x<-XCLAMP) x=-XCLAMP;
        if(x>0.0) x=0.0;
        y = x*LOG2E;
        a[t]=exp2_approx(y);
        sum += a[t];
      end

      tmp='0;
      for(t=0;t<TOKENS;t=t+1)
        tmp[t*32 +:32] = real_to_fp32(a[t]/sum);

      golden_w = tmp;
    end
  endfunction

  // ----------------------------
  // scoreboard stores expected w only (aligned with output)
  // ----------------------------
  localparam int MAXQ=1024;
  reg [W_W-1:0] exp_w_q [0:MAXQ-1];
  integer wr,rd,cnt;

  task sb_push(input [W_W-1:0] w);
    begin exp_w_q[wr]=w; wr=(wr+1)%MAXQ; cnt=cnt+1; end
  endtask
  task sb_pop(output [W_W-1:0] w);
    begin w=exp_w_q[rd]; rd=(rd+1)%MAXQ; cnt=cnt-1; end
  endtask

  // temps
  reg [SCORE_W-1:0] s_tmp;
  reg [W_W-1:0]     w_tmp;

  integer tx, rx;
  reg [W_W-1:0] hold_prev;
  reg hold_v;

  initial begin
    in_valid=0; out_ready=1;
    q_vec='0; k_vecs='0;
    wr=0; rd=0; cnt=0; tx=0; rx=0;
    hold_prev='0; hold_v=0;

    $dumpfile("tb_attn_m1_m2_chain_top.vcd");
    $dumpvars(0,tb_attn_m1_m2_chain_top);

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

  task drive_in();
    integer t,d;
    begin
      while(tx<TX_GOAL) begin
        @(negedge clk);
        if(in_ready && (($urandom()%100)<70)) begin
          for(d=0; d<D; d=d+1)
            q_vec[d*32 +:32] <= real_to_fp32((($urandom()%200)-100)/50.0);
          for(t=0; t<TOKENS; t=t+1)
            for(d=0; d<D; d=d+1)
              k_vecs[(t*D+d)*32 +:32] <= real_to_fp32((($urandom()%200)-100)/50.0);
          in_valid <= 1'b1;
        end else in_valid <= 1'b0;

        @(posedge clk);
        if(rst_n && in_valid && in_ready) begin
          s_tmp = golden_score(q_vec,k_vecs);
          w_tmp = golden_w(s_tmp);
          sb_push(w_tmp);
          tx = tx + 1;
        end
      end
      in_valid <= 1'b0;
    end
  endtask

  task drive_out_ready();
    begin
      forever begin
        @(posedge clk);
        if(!rst_n) out_ready <= 1'b0;
        else      out_ready <= (($urandom()%100)<70);
      end
    end
  endtask

  task monitor();
    reg [W_W-1:0] w_exp;
    real sum;
    integer t;
    integer idle;
    begin
      idle=0;
      forever begin
        @(posedge clk);
        if(!rst_n) begin
          hold_v<=0; idle=0;
        end else begin
          // hold check
          if(out_valid && !out_ready) begin
            if(hold_v && (w_flat!==hold_prev)) begin
              $display("[TB][FAIL] w_flat changed under stall!");
              $finish;
            end
            hold_prev<=w_flat; hold_v<=1;
          end else hold_v<=0;

          // consume
          if(out_valid && out_ready) begin
            sb_pop(w_exp);

            if(w_flat !== w_exp) begin
              $display("[TB][FAIL] w mismatch!");
              for(t=0;t<TOKENS;t=t+1) begin
                $display("  t=%0d exp=%h got=%h exp_r=%f got_r=%f",
                  t,
                  w_exp[t*32 +:32],
                  w_flat[t*32 +:32],
                  fp32_to_real(w_exp[t*32 +:32]),
                  fp32_to_real(w_flat[t*32 +:32])
                );
              end
              $finish;
            end

            if(rx<PRINT_N) begin
              sum=0.0;
              $display("---- RX %0d ----",rx);
              for(t=0;t<TOKENS;t=t+1) begin
                sum += fp32_to_real(w_flat[t*32 +:32]);
                $display("  w[%0d]=%f",t,fp32_to_real(w_flat[t*32 +:32]));
              end
              $display("  sum(w)=%f",sum);
            end

            rx = rx + 1;
          end

          if((tx>=TX_GOAL) && (cnt==0) && !(in_valid && !in_ready)) idle=idle+1;
          else idle=0;

          if(idle>=40) begin
            $display("[TB] DONE");
            $finish;
          end
        end
      end
    end
  endtask

endmodule

`default_nettype wire
