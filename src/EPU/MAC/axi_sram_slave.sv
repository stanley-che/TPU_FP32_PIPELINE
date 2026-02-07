`include "./src/EPU/MAC/axil_2p_sram.sv"
`timescale 1ns/1ps
`default_nettype none

module sram_mem_mn #(
  parameter int unsigned M    = 8,
  parameter int unsigned KMAX = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY = 1,

  // auto widths
  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX),
  parameter int unsigned DEPTH  = M*KMAX,
  parameter int unsigned ADDR_W = (DEPTH<=1)?1:$clog2(DEPTH)
)(
  input  wire                   clk,
  input  wire                   rst,

  // SRAM-like row/k access
  input  wire                   w_en,
  input  wire                   w_re,
  input  wire                   w_we,
  input  wire [ROW_W-1:0]        w_row,
  input  wire [K_W-1:0]          w_k,
  input  wire [DATA_W-1:0]       w_wdata,
  input  wire [BYTE_W-1:0]       w_wmask,

  output wire [DATA_W-1:0]       w_rdata,
  output wire                    w_rvalid
);
    wire a_en_i = w_en && w_re;  
    wire b_en_i = w_en && w_we;
  // row-major linear address: addr = row*KMAX + k
  function automatic [ADDR_W-1:0] lin_addr;
    input [ROW_W-1:0] row;
    input [K_W-1:0]   kk;
    begin
      lin_addr = row * KMAX + kk;
    end
  endfunction

  wire [ADDR_W-1:0] a_addr = lin_addr(w_row, w_k);
  wire [ADDR_W-1:0] b_addr = lin_addr(w_row, w_k);

  // Use your SRAM model
  sram_word_ab #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_mem (
    .clk(clk),
    .rst(rst),

    // Port A read-only
    .a_en    (a_en_i),
    .a_re   (w_re),
    .a_addr (a_addr),
    .a_rdata(w_rdata),
    .a_rvalid(w_rvalid),

    // Port B write-only
    .b_en    (b_en_i),
    .b_we   (w_we),
    .b_addr (b_addr),
    .b_wdata(w_wdata),
    .b_wmask(w_wmask)
  );

endmodule

`default_nettype wire
