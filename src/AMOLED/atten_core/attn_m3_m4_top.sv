`ifndef ATTN_M3_M4_FIXED_SV
`define ATTN_M3_M4_FIXED_SV

`timescale 1ns/1ps
`default_nettype none

// ============================================================
// 1-deep elastic ready/valid stage
// ============================================================
module rv_pipe_stage #(
  parameter integer WIDTH = 32
)(
  input  wire                 clk,
  input  wire                 rst_n,
  input  wire                 in_valid,
  output wire                 in_ready,
  input  wire [WIDTH-1:0]     in_data,
  output wire                 out_valid,
  input  wire                 out_ready,
  output wire [WIDTH-1:0]     out_data
);

  reg                 vld_r;
  reg [WIDTH-1:0]     dat_r;

  assign in_ready  = (~vld_r) | out_ready;
  assign out_valid = vld_r;
  assign out_data  = dat_r;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vld_r <= 1'b0;
      dat_r <= {WIDTH{1'b0}};
    end else begin
      if (in_valid && in_ready) begin
        vld_r <= 1'b1;
        dat_r <= in_data;
      end else if (vld_r && out_ready) begin
        vld_r <= 1'b0;
      end
    end
  end

endmodule

// ============================================================
// N-stage elastic pipe
// ============================================================
module rv_pipe #(
  parameter integer WIDTH  = 32,
  parameter integer STAGES = 1
)(
  input  wire                 clk,
  input  wire                 rst_n,
  input  wire                 in_valid,
  output wire                 in_ready,
  input  wire [WIDTH-1:0]     in_data,
  output wire                 out_valid,
  input  wire                 out_ready,
  output wire [WIDTH-1:0]     out_data
);

  generate
    if (STAGES == 0) begin : g_bypass
      assign in_ready  = out_ready;
      assign out_valid = in_valid;
      assign out_data  = in_data;
    end else begin : g_pipe
      wire [STAGES:0] vld;
      wire [STAGES:0] rdy;
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

// ============================================================
// M3: fixed-point weighted sum
// out_vec[d] = sum_t w[t] * V[t][d]
//
// signed Q(I).FRAC_W format
// ============================================================
module attn_weighted_sum_fx #(
  parameter integer TOKENS      = 9,
  parameter integer D           = 8,
  parameter integer ELEM_W      = 16,
  parameter integer FRAC_W      = 12,
  parameter integer ACC_W       = 40,
  parameter integer PIPE_STAGES = 1
)(
  input  wire                               clk,
  input  wire                               rst_n,

  input  wire                               in_valid,
  output wire                               in_ready,

  input  wire [TOKENS*ELEM_W-1:0]           w_flat,
  input  wire [TOKENS*D*ELEM_W-1:0]         v_vecs,

  output wire                               out_valid,
  input  wire                               out_ready,
  output wire [D*ELEM_W-1:0]                out_vec
);

  localparam integer OUT_W = D*ELEM_W;

  reg  [OUT_W-1:0] comb_out_vec;
  reg  [OUT_W-1:0] s0_dat;
  reg              s0_vld;
  wire             s0_out_ready;

  integer d, t;
  reg signed [ELEM_W-1:0] w_s, v_s;
  reg signed [2*ELEM_W-1:0] mult_s;
  reg signed [ACC_W-1:0] acc_s;
  reg signed [ELEM_W-1:0] sat_s;

  assign in_ready = (~s0_vld) | s0_out_ready;

  // combinational MAC
  always @* begin
    comb_out_vec = {OUT_W{1'b0}};

    for (d = 0; d < D; d = d + 1) begin
      acc_s = {ACC_W{1'b0}};

      for (t = 0; t < TOKENS; t = t + 1) begin
        w_s    = $signed(w_flat[t*ELEM_W +: ELEM_W]);
        v_s    = $signed(v_vecs[((t*D)+d)*ELEM_W +: ELEM_W]);
        mult_s = w_s * v_s;
        acc_s  = acc_s + $signed(mult_s) >>> FRAC_W;
      end

      // saturation to ELEM_W signed
      if (acc_s > $signed({1'b0, {(ELEM_W-1){1'b1}}})) begin
        sat_s = {1'b0, {(ELEM_W-1){1'b1}}};
      end else if (acc_s < $signed({1'b1, {(ELEM_W-1){1'b0}}})) begin
        sat_s = {1'b1, {(ELEM_W-1){1'b0}}};
      end else begin
        sat_s = acc_s[ELEM_W-1:0];
      end

      comb_out_vec[d*ELEM_W +: ELEM_W] = sat_s;
    end
  end

  // stage 0 register
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= {OUT_W{1'b0}};
    end else begin
      if (in_valid && in_ready) begin
        s0_vld <= 1'b1;
        s0_dat <= comb_out_vec;
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

// ============================================================
// M4: alpha from out_vec[0]
// piecewise linear sigmoid approximation
//
// x <= -4.0 => 0
// x >= +4.0 => 1
// else      => x/8 + 0.5
//
// all fixed-point signed Q(I).FRAC_W
// ============================================================
module attn_alpha_fx #(
  parameter integer D           = 8,
  parameter integer ELEM_W      = 16,
  parameter integer FRAC_W      = 12,
  parameter integer PIPE_STAGES = 1
)(
  input  wire                               clk,
  input  wire                               rst_n,

  input  wire                               in_valid,
  output wire                               in_ready,
  input  wire [D*ELEM_W-1:0]                out_vec,

  output wire                               out_valid,
  input  wire                               out_ready,
  output wire [ELEM_W-1:0]                  alpha_fx,

  output wire [D*ELEM_W-1:0]                out_vec_aligned
);

  localparam integer VEC_W    = D*ELEM_W;
  localparam integer BUNDLE_W = VEC_W + ELEM_W;

  localparam signed [ELEM_W-1:0] ONE_Q  = (1 <<< FRAC_W);
  localparam signed [ELEM_W-1:0] HALF_Q = (1 <<< (FRAC_W-1));
  localparam signed [ELEM_W-1:0] POS4_Q = (4 <<< FRAC_W);
  localparam signed [ELEM_W-1:0] NEG4_Q = -(4 <<< FRAC_W);

  reg  [BUNDLE_W-1:0] s0_bundle;
  reg                 s0_vld;
  wire                s0_out_ready;

  reg signed [ELEM_W-1:0] x;
  reg signed [ELEM_W-1:0] y;

  assign in_ready = (~s0_vld) | s0_out_ready;

  always @* begin
    x = $signed(out_vec[0 +: ELEM_W]);

    if (x <= NEG4_Q) begin
      y = {ELEM_W{1'b0}};
    end else if (x >= POS4_Q) begin
      y = ONE_Q;
    end else begin
      // y = x/8 + 0.5
      y = (x >>> 3) + HALF_Q;

      // extra clamp
      if (y < 0)
        y = 0;
      else if (y > ONE_Q)
        y = ONE_Q;
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld    <= 1'b0;
      s0_bundle <= {BUNDLE_W{1'b0}};
    end else begin
      if (in_valid && in_ready) begin
        s0_vld    <= 1'b1;
        s0_bundle <= {out_vec, y};
      end else if (s0_vld && s0_out_ready) begin
        s0_vld <= 1'b0;
      end
    end
  end

  generate
    if (PIPE_STAGES <= 1) begin : g_no_pipe
      assign out_valid       = s0_vld;
      assign s0_out_ready    = out_ready;
      assign out_vec_aligned = s0_bundle[BUNDLE_W-1:ELEM_W];
      assign alpha_fx        = s0_bundle[ELEM_W-1:0];
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

      assign out_vec_aligned = pipe_bundle[BUNDLE_W-1:ELEM_W];
      assign alpha_fx        = pipe_bundle[ELEM_W-1:0];
    end
  endgenerate

endmodule

// ============================================================
// TOP: M3 -> M4 fixed-point chain
// ============================================================
module attn_m3_m4_top #(
  parameter integer TOKENS      = 9,
  parameter integer D           = 8,
  parameter integer ELEM_W      = 16,
  parameter integer FRAC_W      = 12,
  parameter integer ACC_W       = 40,
  parameter integer M3_PIPE_STG = 1,
  parameter integer M4_PIPE_STG = 1
)(
  input  wire                               clk,
  input  wire                               rst_n,

  input  wire                               in_valid,
  output wire                               in_ready,

  input  wire [TOKENS*ELEM_W-1:0]           w_flat,
  input  wire [TOKENS*D*ELEM_W-1:0]         v_vecs,

  output wire                               out_valid,
  input  wire                               out_ready,
  output wire [ELEM_W-1:0]                  alpha_fp32,
  output wire [D*ELEM_W-1:0]                out_vec_dbg
);

  wire                        m3_out_valid;
  wire                        m3_out_ready;
  wire [D*ELEM_W-1:0]         m3_out_vec;
  wire [D*ELEM_W-1:0]         m4_vec_aligned;

  attn_weighted_sum_fp32 #(
    .TOKENS      (TOKENS),
    .D           (D),
    .ELEM_W      (ELEM_W),
    .FRAC_W      (FRAC_W),
    .ACC_W       (ACC_W),
    .PIPE_STAGES (M3_PIPE_STG)
  ) u_m3 (
    .clk       (clk),
    .rst_n     (rst_n),
    .in_valid  (in_valid),
    .in_ready  (in_ready),
    .w_flat    (w_flat),
    .v_vecs    (v_vecs),
    .out_valid (m3_out_valid),
    .out_ready (m3_out_ready),
    .out_vec   (m3_out_vec)
  );

  attn_alpha_fp32 #(
    .D           (D),
    .ELEM_W      (ELEM_W),
    .FRAC_W      (FRAC_W),
    .PIPE_STAGES (M4_PIPE_STG)
  ) u_m4 (
    .clk             (clk),
    .rst_n           (rst_n),
    .in_valid        (m3_out_valid),
    .in_ready        (m3_out_ready),
    .out_vec         (m3_out_vec),
    .out_valid       (out_valid),
    .out_ready       (out_ready),
    .alpha_fp32        (alpha_fp32)
  );

  assign out_vec_dbg = m4_vec_aligned;

endmodule

`default_nettype wire
`endif
