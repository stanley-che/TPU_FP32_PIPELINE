`timescale 1ns/1ps
`default_nettype none
module DW_fp32_add (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] z
);
    reg        sign_a, sign_b, sign_z;
    reg [7:0]  exp_a, exp_b, exp_big, exp_small, exp_z;
    reg [22:0] frac_a, frac_b;
    reg [23:0] mant_a, mant_b;
    reg [23:0] mant_big, mant_small;
    reg        sign_big, sign_small;

    reg [7:0]  exp_diff;
    reg [24:0] mant_small_shifted;
    reg [24:0] mant_sum;
    reg [24:0] mant_diff;
    reg [24:0] mant_norm;

    integer i;

    always @(*) begin
        sign_a = a[31];
        sign_b = b[31];
        exp_a  = a[30:23];
        exp_b  = b[30:23];
        frac_a = a[22:0];
        frac_b = b[22:0];

        // zero shortcut
        if (exp_a == 8'd0 && frac_a == 23'd0) begin
            z = b;
        end
        else if (exp_b == 8'd0 && frac_b == 23'd0) begin
            z = a;
        end
        else begin
            // simplified: assume normal numbers only
            mant_a = {1'b1, frac_a};
            mant_b = {1'b1, frac_b};

            // compare magnitude
            if ({exp_a, mant_a} >= {exp_b, mant_b}) begin
                exp_big   = exp_a;
                exp_small = exp_b;
                mant_big  = mant_a;
                mant_small= mant_b;
                sign_big  = sign_a;
                sign_small= sign_b;
            end
            else begin
                exp_big   = exp_b;
                exp_small = exp_a;
                mant_big  = mant_b;
                mant_small= mant_a;
                sign_big  = sign_b;
                sign_small= sign_a;
            end

            exp_diff = exp_big - exp_small;

            if (exp_diff >= 8'd24)
                mant_small_shifted = 25'd0;
            else
                mant_small_shifted = {1'b0, mant_small} >> exp_diff;

            if (sign_big == sign_small) begin
                // same sign => add
                mant_sum = {1'b0, mant_big} + mant_small_shifted;

                if (mant_sum[24]) begin
                    exp_z    = exp_big + 8'd1;
                    mant_norm= mant_sum >> 1;
                end
                else begin
                    exp_z    = exp_big;
                    mant_norm= mant_sum;
                end

                sign_z = sign_big;

                if (exp_z >= 8'hFF)
                    z = {sign_z, 8'hFE, 23'h7FFFFF};
                else
                    z = {sign_z, exp_z, mant_norm[22:0]};
            end
            else begin
                // different sign => subtract
                mant_diff = {1'b0, mant_big} - mant_small_shifted;
                sign_z    = sign_big;

                if (mant_diff == 25'd0) begin
                    z = 32'd0;
                end
                else begin
                    exp_z    = exp_big;
                    mant_norm= mant_diff;

                    // normalize left
                    for (i = 0; i < 24; i = i + 1) begin
                        if (mant_norm[23] == 1'b0 && exp_z > 8'd0) begin
                            mant_norm = mant_norm << 1;
                            exp_z     = exp_z - 8'd1;
                        end
                    end

                    if (exp_z == 8'd0)
                        z = 32'd0;
                    else
                        z = {sign_z, exp_z, mant_norm[22:0]};
                end
            end
        end
    end

endmodule

module DW_fp32_mult (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] z
);
    reg        sign_a, sign_b, sign_z;
    reg [7:0]  exp_a, exp_b;
    reg [22:0] frac_a, frac_b;

    reg [23:0] mant_a, mant_b;
    reg [47:0] mant_prod;

    reg [8:0]  exp_sum;
    reg [7:0]  exp_z;
    reg [22:0] frac_z;

    always @(*) begin
        sign_a = a[31];
        sign_b = b[31];
        exp_a  = a[30:23];
        exp_b  = b[30:23];
        frac_a = a[22:0];
        frac_b = b[22:0];

        sign_z = sign_a ^ sign_b;

        // zero handling
        if ((exp_a == 8'd0 && frac_a == 23'd0) ||
            (exp_b == 8'd0 && frac_b == 23'd0)) begin
            z = 32'd0;
        end
        else begin
            // simplified: assume normal numbers only
            mant_a = {1'b1, frac_a};
            mant_b = {1'b1, frac_b};

            mant_prod = mant_a * mant_b;   // 24x24 -> 48 bits
            exp_sum   = exp_a + exp_b - 8'd127;

            // normalization
            if (mant_prod[47]) begin
                // product in [2.0, 4.0)
                exp_z  = exp_sum[7:0] + 8'd1;
                frac_z = mant_prod[46:24]; // drop hidden 1
            end
            else begin
                // product in [1.0, 2.0)
                exp_z  = exp_sum[7:0];
                frac_z = mant_prod[45:23]; // drop hidden 1
            end

            // simplified overflow / underflow
            if (exp_sum[8] == 1'b1) begin
                z = 32'd0; // underflow => zero
            end
            else if (exp_z >= 8'hFF) begin
                z = {sign_z, 8'hFE, 23'h7FFFFF}; // saturate max finite
            end
            else begin
                z = {sign_z, exp_z, frac_z};
            end
        end
    end

endmodule

module rv_pipe_stage #(
  parameter int unsigned WIDTH = 32
)(
  input  wire              clk,
  input  wire              rst_n,
  input  wire              in_valid,
  output wire              in_ready,
  input  wire [WIDTH-1:0]  in_data,
  output wire              out_valid,
  input  wire              out_ready,
  output wire [WIDTH-1:0]  out_data
);
  reg                 vld;
  reg [WIDTH-1:0]     dat;

  assign in_ready  = (~vld) | out_ready;
  assign out_valid = vld;
  assign out_data  = dat;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vld <= 1'b0;
      dat <= {WIDTH{1'b0}};
    end else begin
      if (in_valid && in_ready) begin
        dat <= in_data;
        vld <= 1'b1;
      end else if (vld && out_ready) begin
        vld <= 1'b0;
      end
    end
  end
endmodule

module rv_pipe #(
  parameter int unsigned WIDTH  = 32,
  parameter int unsigned STAGES = 1
)(
  input  wire              clk,
  input  wire              rst_n,
  input  wire              in_valid,
  output wire              in_ready,
  input  wire [WIDTH-1:0]  in_data,
  output wire              out_valid,
  input  wire              out_ready,
  output wire [WIDTH-1:0]  out_data
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

module attn_weighted_sum_fp32 #(
  parameter int unsigned TOKENS      = 9,
  parameter int unsigned D           = 8,
  parameter int unsigned PIPE_STAGES = 1
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

  // ========= Stage0 elastic =========
  reg               s0_vld;
  reg [OUT_W-1:0]   s0_dat;
  wire              s0_out_ready;

  assign in_ready = (~s0_vld) | s0_out_ready;
  wire do_accept = in_valid && in_ready;

  function automatic [31:0] get_w(input int unsigned ti);
    begin
      get_w = w_flat[ti*32 +: 32];
    end
  endfunction

  function automatic [31:0] get_v(input int unsigned ti, input int unsigned di);
    int unsigned idx;
    begin
      idx   = ti*D + di;
      get_v = v_vecs[idx*32 +: 32];
    end
  endfunction

  // ========= combinational FP32 MAC =========
  // Requires Synopsys DesignWare
  // ========= combinational FP32 MAC (no Synopsys DW) =========
wire [OUT_W-1:0] mac_out_comb;

genvar d, t;
generate
  for (d = 0; d < D; d = d + 1) begin : G_D
    wire [31:0] prod [0:TOKENS-1];
    wire [31:0] sum  [0:TOKENS];

    assign sum[0] = 32'h00000000; // FP32 0.0

    for (t = 0; t < TOKENS; t = t + 1) begin : G_T
      DW_fp32_mult u_mul (
        .a(get_w(t)),
        .b(get_v(t, d)),
        .z(prod[t])
      );

      DW_fp32_add u_add (
        .a(sum[t]),
        .b(prod[t]),
        .z(sum[t+1])
      );
    end

    assign mac_out_comb[d*32 +: 32] = sum[TOKENS];
  end
endgenerate

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s0_vld <= 1'b0;
      s0_dat <= {OUT_W{1'b0}};
    end else begin
      if (do_accept) begin
        s0_dat <= mac_out_comb;
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

`default_nettype wire
