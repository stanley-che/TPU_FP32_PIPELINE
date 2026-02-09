// ============================================================
// attn_score_9xD_fp32_pipe.sv  (IVERILOG-SAFE)
// - NO shortreal, NO $bitstoshortreal/$shortrealtobits
// - Use real + local FP32 pack/unpack
// - Avoid real power operator (**): use pow2i() loop
// - Keeps ready/valid elastic behavior and holds under stall
//
// score[t] = dot(Q, K[t]) , t=0..TOKENS-1
// ============================================================

`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// Generic 1-deep elastic stage (ready/valid, holds data on stall)
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
// M1: attn_score_9xD_fp32 (PIPELINED, IVERILOG FRIENDLY)
// ------------------------------------------------------------
module attn_score_9xD_fp32 #(
  parameter int unsigned TOKENS      = 9,
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 2  // >=1
)(
  input  wire                       clk,
  input  wire                       rst_n,
  input  wire                       in_valid,
  output wire                       in_ready,
  input  wire [D*32-1:0]            q_vec,
  input  wire [TOKENS*D*32-1:0]     k_vecs,
  output wire                       out_valid,
  input  wire                       out_ready,
  output wire [TOKENS*32-1:0]       score_flat
);

  localparam int unsigned SCORE_W = TOKENS*32;

  // ----------------------------------------------------------
  // pow2i: avoid real '**' operator (iverilog-safe)
  // ----------------------------------------------------------
  function real pow2i(input integer e);
    integer i;
    real v;
    begin
      v = 1.0;
      if (e >= 0) begin
        for (i = 0; i < e; i = i + 1) v = v * 2.0;
      end else begin
        for (i = 0; i < (-e); i = i + 1) v = v / 2.0;
      end
      pow2i = v;
    end
  endfunction

  // ----------------------------
  // FP32 <-> real (local, no shortreal)
  // ----------------------------
  function real fp32_to_real(input [31:0] f);
    reg sign;
    integer exp;
    integer mant;
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
        val = (mant == 0) ? 1.0e30 : 0.0; // clamp (inf/nan)
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
    integer e;
    integer exp_i;
    integer mant_i;
    real frac;
    reg [31:0] out;
    begin
      sign = (r < 0.0);
      a    = sign ? -r : r;

      out = 32'h0000_0000;

      if (a == 0.0) begin
        out = 32'h0000_0000;
      end else if (a > 3.4e38) begin
        out = {sign, 8'hFF, 23'h0}; // Inf
      end else begin
        v = a; e = 0;
        while (v >= 2.0) begin v = v / 2.0; e = e + 1; end
        while (v <  1.0) begin v = v * 2.0; e = e - 1; end

        exp_i = e + 127;

        if (exp_i <= 0) begin
          // subnormal
          frac   = a / pow2i(-126);
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i < 0) mant_i = 0;
          if (mant_i > 8388607) mant_i = 8388607;
          out = {sign, 8'h00, mant_i[22:0]};
        end else if (exp_i >= 255) begin
          out = {sign, 8'hFF, 23'h0};
        end else begin
          frac   = v - 1.0;
          mant_i = integer'(frac * 8388608.0 + 0.5);
          if (mant_i >= 8388608) begin
            mant_i = 0;
            exp_i  = exp_i + 1;
            if (exp_i >= 255) out = {sign, 8'hFF, 23'h0};
            else              out = {sign, exp_i[7:0], mant_i[22:0]};
          end else begin
            out = {sign, exp_i[7:0], mant_i[22:0]};
          end
        end
      end

      real_to_fp32 = out;
    end
  endfunction

  // slicing helpers
  function automatic [31:0] get_q(input int unsigned di);
    get_q = q_vec[di*32 +: 32];
  endfunction

  function automatic [31:0] get_k(input int unsigned ti, input int unsigned di);
    int unsigned idx;
    begin
      idx = (ti*D + di);
      get_k = k_vecs[idx*32 +: 32];
    end
  endfunction

  // ----------------------------
  // stage0: compute-on-accept 1-deep elastic buffer (safe)
  // ----------------------------
  reg                s0_vld;
  reg [SCORE_W-1:0]   s0_dat;

  wire s0_out_ready;
  assign in_ready  = (~s0_vld) | s0_out_ready;
  wire do_accept   = in_valid && in_ready;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= '0;
    end else begin
      if (do_accept) begin
        integer t, d;
        reg [SCORE_W-1:0] score_tmp;
        score_tmp = '0;

        for (t = 0; t < TOKENS; t = t + 1) begin
          real acc;        // <-- automatic per token (key!)
          acc = 0.0;
          for (d = 0; d < D; d = d + 1) begin
            acc = acc + fp32_to_real(get_q(d)) * fp32_to_real(get_k(t,d));
          end
          score_tmp[t*32 +: 32] = real_to_fp32(acc);
        end

        s0_dat <= score_tmp;
        s0_vld <= 1'b1;
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
      // else hold
    end
  end

  // ----------------------------
  // extra stages
  // ----------------------------
  generate
    if (PIPE_STAGES <= 1) begin : g_no_extra
      assign out_valid    = s0_vld;
      assign s0_out_ready = out_ready;
      assign score_flat   = s0_dat;
    end else begin : g_extra
      rv_pipe #(
        .WIDTH (SCORE_W),
        .STAGES(PIPE_STAGES-1)
      ) u_extra (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (s0_vld),
        .in_ready  (s0_out_ready),
        .in_data   (s0_dat),
        .out_valid (out_valid),
        .out_ready (out_ready),
        .out_data  (score_flat)
      );
    end
  endgenerate

endmodule

`default_nettype wire
