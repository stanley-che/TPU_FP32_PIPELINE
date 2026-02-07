// attention_mask_readout.sv
// ------------------------------------------------------------
// Readout-time masking wrapper for attention score matrix SC.
//
// - Padding mask: if pad_valid[key]==0 => output -INF
// - Causal mask : if causal_en && (k > q) => output -INF
//
// This module does NOT modify SRAM contents.
// It only masks data on readout.
//
// ------------------------------------------------------------
`timescale 1ns/1ps
`default_nettype none

module attention_mask_readout #(
  parameter int unsigned T      = 8,
  parameter int unsigned DATA_W = 32,
  localparam int unsigned T_W   = (T<=1)?1:$clog2(T)
)(
  input  logic clk,
  input  logic rst_n,

  // mask control
  input  logic [T-1:0] pad_valid,   // 1=valid token, 0=padding
  input  logic         causal_en,    // 1=enable causal mask

  // upstream read interface (from scaling tile / SC SRAM read port)
  input  logic              in_rvalid,
  input  logic [DATA_W-1:0]  in_rdata,
  input  logic [T_W-1:0]     in_q,      // query index (row)
  input  logic [T_W-1:0]     in_k,      // key index (col)

  // downstream masked readout
  output logic              out_rvalid,
  output logic [DATA_W-1:0] out_rdata
);

  // FP32 -Infinity
  localparam logic [31:0] FP32_NINF = 32'hFF800000;

  logic masked;

  always_comb begin
    masked = 1'b0;

    // padding mask: mask if key token invalid
    // (注意：這裡是用 "key index" 來做 padding mask，符合一般 attention mask)
    if (pad_valid[in_k] == 1'b0)
      masked = 1'b1;

    // causal mask: upper triangle masked (k > q)
    if (causal_en && (in_k > in_q))
      masked = 1'b1;

    out_rvalid = in_rvalid;
    out_rdata  = (masked) ? FP32_NINF : in_rdata;
  end

endmodule

`default_nettype wire
