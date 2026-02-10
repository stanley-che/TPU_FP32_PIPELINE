// ============================================================
// attn_weighted_sum_fp32.sv  (IVERILOG-SAFE)
// - M3: weighted sum / attention output vector
//
//   out[d] = sum_{t=0..TOKENS-1} w[t] * V[t][d],  d=0..D-1
//
// - Inputs:
//   in_valid/in_ready
//   w_flat  : TOKENS * FP32
//   v_vecs  : TOKENS * D * FP32   (layout: t-major then d)
// - Outputs:
//   out_valid/out_ready
//   out_vec : D * FP32
//
// - Notes:
//   * ready/valid elastic
//   * out_vec holds stable under backpressure (out_valid && !out_ready)
//   * No shortreal, use real + local FP32 pack/unpack
// ============================================================

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

// ------------------------------------------------------------
// N-stage elastic pipe generator (STAGES can be 0 for bypass)
// ------------------------------------------------------------
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

  // ----------------------------------------------------------
  // pow2i (iverilog-safe)
  // ----------------------------------------------------------
  function real pow2i(input integer e);
    integer i;
    real v;
    begin
      v = 1.0;
      if (e >= 0)
        for (i = 0; i < e; i = i + 1) v = v * 2.0;
      else
        for (i = 0; i < (-e); i = i + 1) v = v / 2.0;
      pow2i = v;
    end
  endfunction

  // ----------------------------------------------------------
  // FP32 <-> real (local)
  // ----------------------------------------------------------
  function real fp32_to_real(input [31:0] f);
    reg sign;
    integer exp, mant;
    real frac, val;
    begin
      sign = f[31];
      exp  = f[30:23];
      mant = f[22:0];

      if (exp == 0) begin
        if (mant == 0) val = 0.0;
        else begin
          frac = mant / 8388608.0;
          val  = frac * pow2i(-126);
        end
      end else if (exp == 255) begin
        val = (mant == 0) ? 1.0e30 : 0.0;
      end else begin
        frac = 1.0 + (mant / 8388608.0);
        val  = frac * pow2i(exp - 127);
      end

      fp32_to_real = sign ? -val : val;
    end
  endfunction

  function [31:0] real_to_fp32(input real r);
    reg sign;
    real a, v;
    integer e, exp_i, mant_i;
    real frac;
    reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;
      out  = 32'h0;

      if (a == 0.0) begin
        out = 32'h0;
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end
        exp_i = e + 127;

        frac   = v - 1.0;
        mant_i = integer'(frac * 8388608.0 + 0.5);
        if (mant_i >= 8388608) begin
          mant_i = 0;
          exp_i  = exp_i + 1;
        end
        out = {sign, exp_i[7:0], mant_i[22:0]};
      end

      real_to_fp32 = out;
    end
  endfunction

  // ----------------------------------------------------------
  // Stage0: compute-on-accept 1-deep elastic buffer
  // ----------------------------------------------------------
  reg              s0_vld;
  reg [OUT_W-1:0]   s0_dat;
  wire             s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept  = in_valid && in_ready;

  // helper: slice w[t], v[t][d]
  function automatic [31:0] get_w(input int unsigned ti);
    begin
      get_w = w_flat[ti*32 +: 32];
    end
  endfunction

  function automatic [31:0] get_v(input int unsigned ti, input int unsigned di);
    int unsigned idx;
    begin
      idx = (ti*D + di);
      get_v = v_vecs[idx*32 +: 32];
    end
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
          real acc;
          real wr, vr;
          acc = 0.0;
          for (t = 0; t < TOKENS; t = t + 1) begin
            wr  = fp32_to_real(get_w(t));
            vr  = fp32_to_real(get_v(t, d));
            acc = acc + (wr * vr);
          end
          tmp[d*32 +: 32] = real_to_fp32(acc);
        end

        s0_dat <= tmp;
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
      // else hold
    end
  end

  // ----------------------------------------------------------
  // Optional extra elastic stages
  // ----------------------------------------------------------
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

`default_nettype wire
