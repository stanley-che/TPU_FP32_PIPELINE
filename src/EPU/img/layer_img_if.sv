`include "./src/EPU/img/img_sram.sv"
`timescale 1ns/1ps
`default_nettype none

module layer_img_if #(
  parameter int unsigned IMG_H   = 255,
  parameter int unsigned IMG_W   = 255,
  parameter int unsigned ADDR_W  = 16,  // >= $clog2(IMG_H*IMG_W)
  parameter int unsigned DATA_W  = 32
)(
  input  logic                 clk,
  input  logic                 rst,

  // CPU read port
  input  logic                 cpu_rd_req,
  input  logic [15:0]          cpu_row,
  input  logic [15:0]          cpu_col,
  output logic [DATA_W-1:0]    cpu_rdata,

  // Sequencer write port
  input  logic                 seq_we,
  input  logic [15:0]          seq_row,
  input  logic [15:0]          seq_col,
  input  logic [DATA_W-1:0]    seq_wdata
);

  localparam int unsigned DEPTH = IMG_H*IMG_W;

  function automatic logic [ADDR_W-1:0] addr_of(input logic [15:0] r, input logic [15:0] c);
    int unsigned a;
    begin
      a = (r*IMG_W) + c;
      addr_of = a[ADDR_W-1:0];
    end
  endfunction

  logic [ADDR_W-1:0] waddr, raddr;

  assign waddr = addr_of(seq_row, seq_col);
  assign raddr = addr_of(cpu_row, cpu_col);

  img_sram #(
    .DEPTH(DEPTH),
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W)
  ) u_img (
    .clk  (clk),
    .we   (seq_we),
    .waddr(waddr),
    .wdata(seq_wdata),
    .raddr(raddr),
    .rdata(cpu_rdata)
  );

  // cpu_rd_req 先保留給未來 sync-read 的版本（現在 async read 不用 state）
  always_ff @(posedge clk) begin
    if (rst) begin end
  end

endmodule

`default_nettype wire
