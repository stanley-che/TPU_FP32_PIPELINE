`include "./src/EPU/attention_score/sram_word_ab1.sv"
`timescale 1ns/1ps
`default_nettype none

module sram_mem_kn #(
  parameter int unsigned KMAX = 1024,
  parameter int unsigned N    = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY = 1,

  // auto widths
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX),
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned DEPTH  = KMAX*N,
  parameter int unsigned ADDR_W = (DEPTH<=1)?1:$clog2(DEPTH)
)(
  input  wire                   clk,
  input  wire                   rst,

  // SRAM-like k/n access
  input  wire                   x_en,
  input  wire                   x_re,
  input  wire                   x_we,
  input  wire [K_W-1:0]          x_k,
  input  wire [N_W-1:0]          x_n,
  input  wire [DATA_W-1:0]       x_wdata,
  input  wire [BYTE_W-1:0]       x_wmask,

  output wire [DATA_W-1:0]       x_rdata,
  output wire                    x_rvalid
);

  wire a_en_i = x_en && x_re;  
  wire b_en_i = x_en && x_we;

  // k-major linear address: addr = k*N + n
  function automatic [ADDR_W-1:0] lin_addr;
    input [K_W-1:0] kk;
    input [N_W-1:0] nn;
    begin
      lin_addr = kk * N + nn;
    end
  endfunction

  wire [ADDR_W-1:0] a_addr = lin_addr(x_k, x_n);
  wire [ADDR_W-1:0] b_addr = lin_addr(x_k, x_n);

  sram_word_ab1 #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_mem1 (
    .clk      (clk),
    .rst      (rst),

    // Port A read-only
    .a_en     (a_en_i),
    .a_re     (x_re),
    .a_addr   (a_addr),
    .a_rdata  (x_rdata),
    .a_rvalid (x_rvalid),

    // Port B write-only
    .b_en     (b_en_i),
    .b_we     (x_we),
    .b_addr   (b_addr),
    .b_wdata  (x_wdata),
    .b_wmask  (x_wmask)
  );

endmodule

`default_nettype wire
