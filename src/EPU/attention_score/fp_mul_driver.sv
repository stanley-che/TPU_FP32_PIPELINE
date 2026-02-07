`ifndef FP_MUL_DRIVER_SV
`define FP_MUL_DRIVER_SV

`timescale 1ns/1ps
`default_nettype none

// ============================================================
// fp_mul_driver : fixed-latency pipelined FP32 multiplier
// - Interface identical to original fp_mul_driver
// - iverilog-safe (NO block declarations, NO comma-assign)
// ============================================================
module fp_mul_driver #(
  parameter integer LAT = 8
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        start,
  input  logic [31:0] a_bits,
  input  logic [31:0] b_bits,

  output logic        busy,
  output logic        done,
  output logic [31:0] z_bits
);

  logic running;
  logic [$clog2(LAT+1)-1:0] cnt;

  assign busy = running;

  // pipeline core wires
  logic        core_in_valid;
  logic [31:0] core_a, core_b;
  logic        core_out_valid;
  logic [31:0] core_z;

  assign core_in_valid = start && !running;
  assign core_a = a_bits;
  assign core_b = b_bits;

  fp32_mul_pipe_core #(.LAT(LAT)) u_core (
    .clk(clk),
    .rst_n(!rst),
    .in_valid(core_in_valid),
    .a_bits(core_a),
    .b_bits(core_b),
    .out_valid(core_out_valid),
    .z_bits(core_z)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      running <= 1'b0;
      cnt     <= '0;
      done    <= 1'b0;
      z_bits  <= 32'd0;
    end else begin
      if (core_in_valid) begin
        running <= 1'b1;
        cnt     <= LAT[$clog2(LAT+1)-1:0];
        done    <= 1'b0;
      end

      if (running && cnt != 0)
        cnt <= cnt - 1'b1;

      if (core_out_valid) begin
        z_bits  <= core_z;
        done    <= 1'b1;
        running <= 1'b0;
        cnt     <= '0;
      end
    end
  end

endmodule


// ============================================================
// fp32_mul_pipe_core (iverilog-safe, FTZ)
// ============================================================
module fp32_mul_pipe_core #(
  parameter integer LAT = 8
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] a_bits,
  input  logic [31:0] b_bits,

  output logic        out_valid,
  output logic [31:0] z_bits
);

  // valid pipeline
  logic [LAT-1:0] vpipe;
  always_ff @(posedge clk)
    if (!rst_n) vpipe <= '0;
    else        vpipe <= {vpipe[LAT-2:0], in_valid};

  assign out_valid = vpipe[LAT-1];

  // stage registers
  logic        s0_special;
  logic [31:0] s0_special_z;
  logic        s0_sign;
  logic signed [9:0] s0_exp;
  logic [23:0] s0_ma, s0_mb;

  logic        s1_special;
  logic [31:0] s1_special_z;
  logic        s1_sign;
  logic signed [9:0] s1_exp;
  logic [47:0] s1_prod;
  logic [7:0] exp_biased;

  // special detect
  logic a_nan, b_nan, a_inf, b_inf, a_zero, b_zero;

  // ---------------- Stage0 ----------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s0_special   <= 1'b0;
      s0_special_z <= 32'd0;
      s0_sign      <= 1'b0;
      s0_exp       <= '0;
      s0_ma        <= '0;
      s0_mb        <= '0;
    end else begin
      a_nan  <= (a_bits[30:23]==8'hFF && a_bits[22:0]!=0);
      b_nan  <= (b_bits[30:23]==8'hFF && b_bits[22:0]!=0);
      a_inf  <= (a_bits[30:23]==8'hFF && a_bits[22:0]==0);
      b_inf  <= (b_bits[30:23]==8'hFF && b_bits[22:0]==0);
      a_zero <= (a_bits[30:0]==0);
      b_zero <= (b_bits[30:0]==0);

      s0_special   <= 1'b0;
      s0_special_z <= 32'd0;

      if (a_nan || b_nan) begin
        s0_special   <= 1'b1;
        s0_special_z <= 32'h7FC00000;
      end else if ((a_inf && b_zero) || (b_inf && a_zero)) begin
        s0_special   <= 1'b1;
        s0_special_z <= 32'h7FC00000;
      end else if (a_inf || b_inf) begin
        s0_special   <= 1'b1;
        s0_special_z <= {a_bits[31]^b_bits[31],8'hFF,23'd0};
      end else if (a_zero || b_zero) begin
        s0_special   <= 1'b1;
        s0_special_z <= 32'd0;
      end

      s0_sign <= a_bits[31] ^ b_bits[31];
      s0_exp  <= (a_bits[30:23]-127) + (b_bits[30:23]-127);

      s0_ma <= (a_bits[30:23]==0) ? {1'b0,a_bits[22:0]} : {1'b1,a_bits[22:0]};
      s0_mb <= (b_bits[30:23]==0) ? {1'b0,b_bits[22:0]} : {1'b1,b_bits[22:0]};
    end
  end

  // ---------------- Stage1 ----------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s1_special   <= 1'b0;
      s1_special_z <= 32'd0;
      s1_sign      <= 1'b0;
      s1_exp       <= '0;
      s1_prod      <= '0;
    end else begin
      s1_special   <= s0_special;
      s1_special_z <= s0_special_z;
      s1_sign      <= s0_sign;
      s1_exp       <= s0_exp;
      s1_prod      <= s0_ma * s0_mb;
    end
  end

  // ---------------- Stage2 (pack) ----------------
always_ff @(posedge clk) begin
  if (!rst_n) begin
    z_bits <= 32'd0;
  end else begin
    if (s1_special) begin
      z_bits <= s1_special_z;
    end else begin
      // case: product MSB = 1
      if (s1_prod[47]) begin
        if (s1_exp + 1 + 127 >= 255) begin
          z_bits <= {s1_sign,8'hFF,23'd0};
        end else if (s1_exp + 1 + 127 <= 0) begin
          z_bits <= 32'd0;
        end else begin
          exp_biased = s1_exp + 1 + 127;
          z_bits <= {s1_sign, exp_biased, s1_prod[46:24]};
        end
      end
      // case: product MSB = 0
      else begin
        if (s1_exp + 127 >= 255) begin
          z_bits <= {s1_sign,8'hFF,23'd0};
        end else if (s1_exp + 127 <= 0) begin
          z_bits <= 32'd0;
        end else begin
          exp_biased = s1_exp + 127;
          z_bits <= {s1_sign, exp_biased, s1_prod[45:23]};
        end
      end
    end
  end
end


endmodule

`endif
