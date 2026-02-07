`ifndef ROUND_ROBIN_SV
`define ROUND_ROBIN_SV
`timescale 1ns/1ps
`default_nettype none

module RR_single #(
  parameter integer N = 8
)(
  input  wire              clk,
  input  wire              rst_n,
  input  wire [N-1:0]      req,
  output wire [N-1:0]      gnt,
  output wire              gnt_flag
);

  reg  [N-1:0] post_base, pre_base;

  reg  [2*N-1:0] dbl_req;
  reg  [2*N-1:0] dbl_base;
  reg  [2*N-1:0] pick;

  wire [N-1:0] gnt_pick;

  // base pointer
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      post_base <= {{(N-1){1'b0}}, 1'b1};
    end else begin
      post_base <= pre_base;
    end
  end

  // combinational pick
  always @* begin
    dbl_req  = {req, req};
    dbl_base = {{(N){1'b0}}, post_base};
    pick     = dbl_req & ~(dbl_req - dbl_base);
  end

  assign gnt_pick = pick[N-1:0] | pick[2*N-1:N];
  assign gnt      = gnt_pick;
  assign gnt_flag = |gnt_pick;

  // next base
  generate
    if (N <= 1) begin : gen_n1
      always @* begin
        if (gnt_flag) pre_base = {N{1'b1}}; // N=1 -> 1'b1
        else          pre_base = post_base;
      end
    end else begin : gen_ngt1
      always @* begin
        if (gnt_flag) begin
          pre_base = {gnt_pick[N-2:0], gnt_pick[N-1]};
        end else begin
          pre_base = post_base;
        end
      end
    end
  endgenerate

endmodule

`default_nettype wire
`endif