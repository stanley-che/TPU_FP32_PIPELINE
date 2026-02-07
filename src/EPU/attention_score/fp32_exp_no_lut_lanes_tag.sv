/*
fp32_exp_no_lut_lanes_tag
*/
`include "./src/EPU/attention_score/fp32_exp_no_lut.sv"
module fp32_exp_no_lut_vec #(
  parameter int unsigned LANES = 4
)(
  input  logic                    clk,
  input  logic                    rst_n,
  input  logic [LANES-1:0]        in_valid,
  input  logic [LANES-1:0][31:0]  in_fp32,
  output logic [LANES-1:0]        out_valid,
  output logic [LANES-1:0][31:0]  out_fp32
);

  genvar i;
  generate
    for (i=0; i<LANES; i++) begin : G
      fp32_exp_no_lut u_exp (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid[i]),
        .in_fp32(in_fp32[i]),
        .out_valid(out_valid[i]),
        .out_fp32(out_fp32[i])
      );
    end
  endgenerate
endmodule
