`timescale 1ns/1ps
`default_nettype none

module single_bank_sram #(
  parameter integer ADDR_W = 12,
  parameter integer Data_W = 16
)(
  input  wire                  clk,
  input  wire                  rst_n,
  input  wire                  we,      // write enable
  input  wire                  cs,      // chip select
  input  wire [ADDR_W-1:0]     addr,
  output reg  [Data_W-1:0]     rdata,   // read data (sync 1-cycle)
  input  wire [Data_W-1:0]     wdata    // write data
);

  // Optional: make depth explicit for some tools
  localparam integer DEPTH = (1 << ADDR_W);

  reg [Data_W-1:0] Mem [0:DEPTH-1];

  always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdata <= {Data_W{1'b0}};
    end else if (cs) begin
      if (we) begin
        Mem[addr] <= wdata;        // write
      end
      // read-first behavior on write/read same addr
      rdata <= Mem[addr];          // sync read (1-cycle latency)
    end
    // else: hold rdata
  end

endmodule

`default_nettype wire

