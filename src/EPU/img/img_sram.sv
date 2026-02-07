`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// img_sram: simulation-friendly framebuffer SRAM
// ------------------------------------------------------------
// * 1 write port (sync)
// * 1 read port (async combinational)
// NOTE: synthesis 時可換成 sync-read macro wrapper
module img_sram #(
  parameter int unsigned DEPTH  = 255*255,
  parameter int unsigned ADDR_W = 16,          // enough for 65025
  parameter int unsigned DATA_W = 32
)(
  input  logic                 clk,
  input  logic                 we,
  input  logic [ADDR_W-1:0]    waddr,
  input  logic [DATA_W-1:0]    wdata,
  input  logic [ADDR_W-1:0]    raddr,
  output logic [DATA_W-1:0]    rdata
);

  logic [DATA_W-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
  end

  always_comb begin
    if (raddr < DEPTH) rdata = mem[raddr];
    else               rdata = '0;
  end

endmodule

`default_nettype wire
