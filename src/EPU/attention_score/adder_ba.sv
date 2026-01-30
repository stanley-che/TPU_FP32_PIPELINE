
`ifndef FP32_ADD_PIPE_FIXED_BA
`define FP32_ADD_PIPE_FIXED_BA
`include "./src/EPU/attention_score/adder_ba.sv"
`timescale 1ns/1ps
module fp32_add_pipe_fixed_ba #(
  parameter integer LAT = 6
)(
  input             clk,
  input             rst_n,

  input             in_valid,
  input      [31:0] a_bits,
  input      [31:0] b_bits,

  output            out_valid,
  output reg [31:0] z_bits
);

  // ---------- CLZ ----------
  function integer clz28;
    input [27:0] x;
    integer i;
    begin
      clz28 = 28;
      for (i = 27; i >= 0; i = i - 1) begin
        if (x[i]) begin
          clz28 = 27 - i;
          i = -1;
        end
      end
    end
  endfunction

  // ---------- valid pipeline ----------
  reg [LAT-1:0] vpipe;
  assign out_valid = vpipe[LAT-1];

  always @(posedge clk) begin
    if (!rst_n)
      vpipe <= {LAT{1'b0}};
    else
      vpipe <= {vpipe[LAT-2:0], in_valid};
  end

  // ---------- stage registers ----------
  reg        s0_special;
  reg [31:0] s0_special_z;
  reg        s0_a_s, s0_b_s;
  reg [7:0]  s0_a_e, s0_b_e;
  reg [23:0] s0_a_m24, s0_b_m24;

  reg        s1_special;
  reg [31:0] s1_special_z;
  reg        s1_z_s;
  reg        s1_do_sub;
  reg signed [10:0] s1_e_big;
  reg [27:0] s1_m_big;
  reg [27:0] s1_m_sml;

  reg        s2_special;
  reg [31:0] s2_special_z;
  reg        s2_z_s;
  reg signed [10:0] s2_z_e;
  reg [28:0] s2_sum;

  reg        s3_special;
  reg [31:0] s3_special_z;
  reg        s3_z_s;
  reg signed [10:0] s3_z_e;
  reg [27:0] s3_z_m;

  reg        s4_special;
  reg [31:0] s4_special_z;
  reg        s4_z_s;
  reg signed [10:0] s4_z_e;
  reg [27:0] s4_z_m;

  // ---------- Stage0 ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      s0_special <= 1'b0;
      s0_special_z <= 32'd0;
      s0_a_s <= 0; s0_b_s <= 0;
      s0_a_e <= 0; s0_b_e <= 0;
      s0_a_m24 <= 0; s0_b_m24 <= 0;
    end else begin
      s0_a_s <= a_bits[31];
      s0_b_s <= b_bits[31];
      s0_a_e <= a_bits[30:23];
      s0_b_e <= b_bits[30:23];

      s0_a_m24 <= (a_bits[30:23]==0) ? {1'b0,a_bits[22:0]} : {1'b1,a_bits[22:0]};
      s0_b_m24 <= (b_bits[30:23]==0) ? {1'b0,b_bits[22:0]} : {1'b1,b_bits[22:0]};

      s0_special <= 1'b0;
      s0_special_z <= 32'd0;
      if ((a_bits[30:23]==8'hFF && a_bits[22:0]!=0) ||
          (b_bits[30:23]==8'hFF && b_bits[22:0]!=0))
      begin
        s0_special <= 1'b1;
        s0_special_z <= 32'h7FC00000;
      end
    end
  end

  // ---------- Stage1 ALIGN (FIXED HERE) ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      s1_special <= 0; s1_special_z <= 0;
      s1_z_s <= 0; s1_do_sub <= 0;
      s1_e_big <= 0; s1_m_big <= 0; s1_m_sml <= 0;
    end else begin
      s1_special   <= s0_special;
      s1_special_z <= s0_special_z;

      if (!s0_special) begin
        integer ea, eb, e_big, e_sml;
        integer shift, i;
        reg [27:0] ma_ext, mb_ext;
        reg [27:0] big_m, sml_m;
        reg big_s, sml_s;
        reg [27:0] shifted;
        reg sticky;

        ea = (s0_a_e==0) ? (1-127) : (s0_a_e-127);
        eb = (s0_b_e==0) ? (1-127) : (s0_b_e-127);

        ma_ext = {s0_a_m24,4'b0000};
        mb_ext = {s0_b_m24,4'b0000};

        if ((ea>eb) || ((ea==eb)&&(ma_ext>=mb_ext))) begin
          big_m = ma_ext; big_s = s0_a_s; e_big = ea;
          sml_m = mb_ext; sml_s = s0_b_s; e_sml = eb;
        end else begin
          big_m = mb_ext; big_s = s0_b_s; e_big = eb;
          sml_m = ma_ext; sml_s = s0_a_s; e_sml = ea;
        end

        shift = e_big - e_sml;
        if (shift > 27) shift = 28;

        sticky = 1'b0;
        if (shift == 0) begin
          shifted = sml_m;
        end else if (shift >= 28) begin
          shifted = 28'd0;
          sticky  = |sml_m;
        end else begin
          shifted = sml_m >> shift;
          sticky = 1'b0;
          for (i = 0; i < shift; i = i + 1)
            sticky = sticky | sml_m[i];
        end
        shifted[0] = shifted[0] | sticky;

        s1_e_big  <= e_big;
        s1_m_big  <= big_m;
        s1_m_sml  <= shifted;
        s1_z_s    <= big_s;
        s1_do_sub <= big_s ^ sml_s;
      end
    end
  end

  // ---------- Stage2 ADD ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      s2_special <= 0; s2_special_z <= 0;
      s2_z_s <= 0; s2_z_e <= 0; s2_sum <= 0;
    end else begin
      s2_special <= s1_special;
      s2_special_z <= s1_special_z;
      if (!s1_special) begin
        s2_z_s <= s1_z_s;
        s2_z_e <= s1_e_big;
        s2_sum <= s1_do_sub ? ({1'b0,s1_m_big}-{1'b0,s1_m_sml})
                            : ({1'b0,s1_m_big}+{1'b0,s1_m_sml});
      end
    end
  end

  // ---------- Stage3 NORMALIZE ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      s3_special <= 0; s3_special_z <= 0;
      s3_z_s <= 0; s3_z_e <= 0; s3_z_m <= 0;
    end else begin
      s3_special <= s2_special;
      s3_special_z <= s2_special_z;
      if (!s2_special) begin
        s3_z_s <= s2_z_s;
        if (s2_sum == 0) begin
          s3_z_e <= -126;
          s3_z_m <= 0;
        end else if (s2_sum[28]) begin
          s3_z_e <= s2_z_e + 1;
          s3_z_m <= s2_sum[28:1];
        end else begin
          integer lz;
          lz = clz28(s2_sum[27:0]);
          s3_z_e <= s2_z_e - lz;
          s3_z_m <= s2_sum[27:0] << lz;
        end
      end
    end
  end

  // ---------- Stage4 PASS ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      s4_special <= 0; s4_special_z <= 0;
      s4_z_s <= 0; s4_z_e <= 0; s4_z_m <= 0;
    end else begin
      s4_special <= s3_special;
      s4_special_z <= s3_special_z;
      s4_z_s <= s3_z_s;
      s4_z_e <= s3_z_e;
      s4_z_m <= s3_z_m;
    end
  end

  // ---------- Stage5 PACK ----------
  always @(posedge clk) begin
    if (!rst_n) begin
      z_bits <= 0;
    end else begin
      if (s4_special)
        z_bits <= s4_special_z;
      else begin
        integer eb;
        eb = s4_z_e + 127;
        if (eb >= 255)
          z_bits <= {s4_z_s,8'hFF,23'd0};
        else if (eb <= 0)
          z_bits <= 32'd0;
        else
          z_bits <= {s4_z_s,eb[7:0],s4_z_m[26:4]};
        if (s4_z_m == 0)
          z_bits <= 32'd0;
      end
    end
  end

endmodule
`endif