// ============================================================
// attn_m3_m4_top.sv  (IVERILOG-SAFE, single file)
// - Contains:
//   * rv_pipe_stage / rv_pipe
//   * attn_weighted_sum_fp32   (M3)
//   * attn_alpha_fp32          (M4)  **FIXED: pipes {vec,alpha} as one payload**
//   * attn_m3_m4_top           (M3->M4 chain)
//
// M3: out_vec[d] = sum_t w[t] * V[t][d]
// M4: alpha = g(out_vec[0])  (MODE selectable)
//
// Notes:
// - full ready/valid propagation
// - hold under backpressure guaranteed by elastic regs
// - FIX: out_vec_dbg is now aligned with out_valid/alpha_fp32
//        by piping {out_vec,alpha} as one bundle inside M4.
// ============================================================
`ifndef RV_PIPE_STAGE
`define RV_PIPE_STAGE
`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// Generic 1-deep elastic stage (ready/valid, holds on stall)
// ------------------------------------------------------------
module rv_pipe_stage #(
  parameter int unsigned WIDTH = 32
)(
  input  wire                  clk,
  input  wire                  rst_n,
  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [WIDTH-1:0]      in_data,
  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [WIDTH-1:0]      out_data
);
  reg                 vld;
  reg [WIDTH-1:0]     dat;

  assign in_ready  = (~vld) | out_ready;
  assign out_valid = vld;
  assign out_data  = dat;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vld <= 1'b0;
      dat <= '0;
    end else begin
      if (in_valid && in_ready) begin
        dat <= in_data;
        vld <= 1'b1;
      end else if (vld && out_ready) begin
        vld <= 1'b0;
      end
      // else hold
    end
  end
endmodule
`endif
// ------------------------------------------------------------
// N-stage elastic pipe generator (STAGES can be 0 for bypass)
// ------------------------------------------------------------
`ifndef RV_PIPE
`define RV_PIPE
module rv_pipe #(
  parameter int unsigned WIDTH  = 32,
  parameter int unsigned STAGES = 1
)(
  input  wire                  clk,
  input  wire                  rst_n,
  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [WIDTH-1:0]      in_data,
  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [WIDTH-1:0]      out_data
);
  generate
    if (STAGES == 0) begin : g_bypass
      assign in_ready  = out_ready;
      assign out_valid = in_valid;
      assign out_data  = in_data;
    end else begin : g_pipe
      wire [STAGES:0]  vld;
      wire [STAGES:0]  rdy;
      wire [WIDTH-1:0] dat [0:STAGES];

      assign vld[0]   = in_valid;
      assign dat[0]   = in_data;
      assign in_ready = rdy[0];

      assign out_valid   = vld[STAGES];
      assign out_data    = dat[STAGES];
      assign rdy[STAGES] = out_ready;

      genvar i;
      for (i = 0; i < STAGES; i = i + 1) begin : g_stage
        rv_pipe_stage #(.WIDTH(WIDTH)) u_stage (
          .clk       (clk),
          .rst_n     (rst_n),
          .in_valid  (vld[i]),
          .in_ready  (rdy[i]),
          .in_data   (dat[i]),
          .out_valid (vld[i+1]),
          .out_ready (rdy[i+1]),
          .out_data  (dat[i+1])
        );
      end
    end
  endgenerate
endmodule
`endif
// ------------------------------------------------------------
// M3: attn_weighted_sum_fp32
// ------------------------------------------------------------
module attn_weighted_sum_fp32 #(
  parameter int unsigned TOKENS      = 9,
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 1   // >=1 (extra elastic stages)
)(
  input  wire                       clk,
  input  wire                       rst_n,

  input  wire                       in_valid,
  output wire                       in_ready,
  input  wire [TOKENS*32-1:0]       w_flat,
  input  wire [TOKENS*D*32-1:0]     v_vecs,

  output wire                       out_valid,
  input  wire                       out_ready,
  output wire [D*32-1:0]            out_vec
);

  localparam int unsigned OUT_W = D*32;

  function real pow2i(input integer e);
    integer i; real v;
    begin
      v = 1.0;
      if (e >= 0) for (i=0;i<e;i=i+1) v = v * 2.0;
      else        for (i=0;i<-e;i=i+1) v = v / 2.0;
      pow2i = v;
    end
  endfunction

  function real fp32_to_real(input [31:0] f);
    reg sign; integer exp, mant; real frac, val;
    begin
      sign=f[31]; exp=f[30:23]; mant=f[22:0];
      if (exp==0) begin
        if (mant==0) val=0.0;
        else begin frac=mant/8388608.0; val=frac*pow2i(-126); end
      end else if (exp==255) begin
        val=(mant==0)?1.0e30:0.0;
      end else begin
        frac=1.0+(mant/8388608.0);
        val=frac*pow2i(exp-127);
      end
      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign; real a,v; integer e, exp_i, mant_i; real frac; reg [31:0] out;
    begin
      sign=(r<0.0);
      a=sign?-r:r;
      out=32'h0;
      if (a==0.0) out=32'h0;
      else begin
        v=a; e=0;
        while (v>=2.0) begin v=v/2.0; e=e+1; end
        while (v< 1.0) begin v=v*2.0; e=e-1; end
        exp_i=e+127;
        frac=v-1.0;
        mant_i=integer'(frac*8388608.0+0.5);
        if (mant_i>=8388608) begin mant_i=0; exp_i=exp_i+1; end
        out={sign,exp_i[7:0],mant_i[22:0]};
      end
      real_to_fp32=out;
    end
  endfunction

  reg              s0_vld;
  reg [OUT_W-1:0]   s0_dat;
  wire             s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  function [31:0] get_w(input int unsigned ti);
    begin get_w = w_flat[ti*32 +: 32]; end
  endfunction

  function [31:0] get_v(input int unsigned ti, input int unsigned di);
    int unsigned idx;
    begin idx = (ti*D + di); get_v = v_vecs[idx*32 +: 32]; end
  endfunction

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= '0;
    end else begin
      if (do_accept) begin
        integer d, t;
        reg [OUT_W-1:0] tmp;
        tmp = '0;

        for (d = 0; d < D; d = d + 1) begin
          real acc, wr, vr;
          acc = 0.0;
          for (t = 0; t < TOKENS; t = t + 1) begin
            wr = fp32_to_real(get_w(t));
            vr = fp32_to_real(get_v(t,d));
            acc = acc + (wr * vr);
          end
          tmp[d*32 +: 32] = real_to_fp32(acc);
        end

        s0_dat <= tmp;
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid    = s0_vld;
      assign s0_out_ready = out_ready;
      assign out_vec      = s0_dat;
    end else begin : g_pipe
      rv_pipe #(
        .WIDTH (OUT_W),
        .STAGES(PIPE_STAGES-1)
      ) u_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_dat),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (out_vec)
      );
    end
  endgenerate

endmodule

// ------------------------------------------------------------
// M4: attn_alpha_fp32
// FIX: pipeline {out_vec, alpha} as one bundle so vector is aligned
// ------------------------------------------------------------
module attn_alpha_fp32 #(
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 1,   // >=1
  parameter int unsigned MODE        = 1,   // 0=sigmoid,1=clamp,2=linear+clamp
  parameter real         SCALE       = 1.0,
  parameter real         BIAS        = 0.0,
  parameter real         SIG_A       = 1.0
)(
  input  wire                  clk,
  input  wire                  rst_n,

  input  wire                  in_valid,
  output wire                  in_ready,
  input  wire [D*32-1:0]       out_vec,

  output wire                  out_valid,
  input  wire                  out_ready,
  output wire [31:0]           alpha_fp32,

  // passthrough vector aligned with out_valid/alpha_fp32
  output wire [D*32-1:0]       out_vec_aligned
);

  localparam int unsigned VEC_W    = D*32;
  localparam int unsigned BUNDLE_W = VEC_W + 32;

  function real pow2i(input integer e);
    integer i; real v;
    begin
      v = 1.0;
      if (e >= 0) for (i=0;i<e;i=i+1) v = v * 2.0;
      else        for (i=0;i<-e;i=i+1) v = v / 2.0;
      pow2i = v;
    end
  endfunction

  function real fp32_to_real(input [31:0] f);
    reg sign; integer exp, mant; real frac, val;
    begin
      sign=f[31]; exp=f[30:23]; mant=f[22:0];
      if (exp==0) begin
        if (mant==0) val=0.0;
        else begin frac=mant/8388608.0; val=frac*pow2i(-126); end
      end else if (exp==255) begin
        val=(mant==0)?1.0e30:0.0;
      end else begin
        frac=1.0+(mant/8388608.0);
        val=frac*pow2i(exp-127);
      end
      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign; real a,v; integer e, exp_i, mant_i; real frac; reg [31:0] out;
    begin
      sign=(r<0.0);
      a=sign?-r:r;
      out=32'h0;
      if (a==0.0) out=32'h0;
      else begin
        v=a; e=0;
        while (v>=2.0) begin v=v/2.0; e=e+1; end
        while (v< 1.0) begin v=v*2.0; e=e-1; end
        exp_i=e+127;
        frac=v-1.0;
        mant_i=integer'(frac*8388608.0+0.5);
        if (mant_i>=8388608) begin mant_i=0; exp_i=exp_i+1; end
        out={sign,exp_i[7:0],mant_i[22:0]};
      end
      real_to_fp32=out;
    end
  endfunction

  function real clamp01(input real x);
    begin
      if (x < 0.0) clamp01 = 0.0;
      else if (x > 1.0) clamp01 = 1.0;
      else clamp01 = x;
    end
  endfunction

  localparam real LN2   = 0.6931471805599453;
  localparam real LOG2E = 1.4426950408889634;

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
      if ((y < 0.0) && (y != it)) ifloor = it - 1;
      else                        ifloor = it;
      F = y - ifloor;
      exp2_approx = pow2i(ifloor) * exp2_frac(F);
    end
  endfunction

  function real sigmoid_approx(input real x);
    real z, e;
    begin
      if (x > 8.0) sigmoid_approx = 0.999664;
      else if (x < -8.0) sigmoid_approx = 0.000335;
      else begin
        z = (-SIG_A * x) * LOG2E;
        e = exp2_approx(z);
        sigmoid_approx = 1.0 / (1.0 + e);
      end
    end
  endfunction

  reg                 s0_vld;
  reg [BUNDLE_W-1:0]  s0_bundle;
  wire                s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  wire [31:0] out0_bits = out_vec[0*32 +: 32];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld    <= 1'b0;
      s0_bundle <= '0;
    end else begin
      if (do_accept) begin
        real x, a;
        reg [31:0] a_bits;

        x = fp32_to_real(out0_bits);

        if (MODE == 0)      a = sigmoid_approx(x);
        else if (MODE == 1) a = clamp01(x);
        else                a = clamp01(SCALE*x + BIAS);

        a_bits   = real_to_fp32(a);
        s0_bundle <= {out_vec, a_bits};   // [VEC_W +: VEC_W] | [31:0]
        s0_vld    <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid       = s0_vld;
      assign s0_out_ready    = out_ready;
      assign out_vec_aligned = s0_bundle[BUNDLE_W-1 -: VEC_W];
      assign alpha_fp32      = s0_bundle[31:0];
    end else begin : g_pipe
      wire [BUNDLE_W-1:0] pipe_bundle;

      rv_pipe #(
        .WIDTH (BUNDLE_W),
        .STAGES(PIPE_STAGES-1)
      ) u_pipe (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_bundle),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (pipe_bundle)
      );

      assign out_vec_aligned = pipe_bundle[BUNDLE_W-1 -: VEC_W];
      assign alpha_fp32      = pipe_bundle[31:0];
    end
  endgenerate

endmodule

// ------------------------------------------------------------
// TOP: M3 -> M4 chain
// ------------------------------------------------------------
`ifndef ATTN_M3_M4_TOP
`define ATTN_M3_M4_TOP
module attn_m3_m4_top #(
  parameter int unsigned TOKENS        = 9,
  parameter int unsigned D             = 8,
  parameter int unsigned M3_PIPE_STG   = 1,
  parameter int unsigned M4_PIPE_STG   = 1,

  parameter int unsigned ALPHA_MODE    = 1,   // 0/1/2
  parameter real         ALPHA_SCALE   = 1.0,
  parameter real         ALPHA_BIAS    = 0.0,
  parameter real         ALPHA_SIG_A   = 1.0
)(
  input  wire                       clk,
  input  wire                       rst_n,

  input  wire                       in_valid,
  output wire                       in_ready,
  input  wire [TOKENS*32-1:0]       w_flat,
  input  wire [TOKENS*D*32-1:0]     v_vecs,

  output wire                       out_valid,
  input  wire                       out_ready,
  output wire [31:0]                alpha_fp32,

  // debug tap (NOW ALIGNED with out_valid/alpha_fp32)
  output wire [D*32-1:0]            out_vec_dbg
);

  wire              m3_out_valid;
  wire              m3_out_ready;
  wire [D*32-1:0]   m3_out_vec;

  wire [D*32-1:0]   m4_vec_aligned;

  // M3
  attn_weighted_sum_fp32 #(
    .TOKENS(TOKENS),
    .D(D),
    .PIPE_STAGES(M3_PIPE_STG)
  ) u_m3 (
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .in_ready(in_ready),
    .w_flat(w_flat),
    .v_vecs(v_vecs),
    .out_valid(m3_out_valid),
    .out_ready(m3_out_ready),
    .out_vec(m3_out_vec)
  );

  // M4 (FIXED)
  attn_alpha_fp32 #(
    .D(D),
    .PIPE_STAGES(M4_PIPE_STG),
    .MODE(ALPHA_MODE),
    .SCALE(ALPHA_SCALE),
    .BIAS(ALPHA_BIAS),
    .SIG_A(ALPHA_SIG_A)
  ) u_m4 (
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(m3_out_valid),
    .in_ready(m3_out_ready),
    .out_vec(m3_out_vec),
    .out_valid(out_valid),
    .out_ready(out_ready),
    .alpha_fp32(alpha_fp32),
    .out_vec_aligned(m4_vec_aligned)
  );

  // aligned debug vector
  assign out_vec_dbg = m4_vec_aligned;

endmodule

`default_nettype wire
`endif
