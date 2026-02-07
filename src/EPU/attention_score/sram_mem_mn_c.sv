`ifndef SRAM_MEM__MN_C_SV
`define SRAM_MEM__MN_C_SV
`include "./src/EPU/attention_score/sram_word_abD.sv"
`timescale 1ns/1ps
`default_nettype none

module sram_mem_mn_c #(
  parameter int unsigned M    = 8,
  parameter int unsigned N    = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY = 1,

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N),
  parameter int unsigned DEPTH  = M*N,
  parameter int unsigned ADDR_W = (DEPTH<=1)?1:$clog2(DEPTH)
)(
  input  wire                   clk,
  input  wire                   rst,

  // ---- read port (A) ----
  input  wire                   c_en,
  input  wire                   c_re,
  input  wire [ROW_W-1:0]       c_row,
  input  wire [COL_W-1:0]       c_col,
  output wire [DATA_W-1:0]      c_rdata,
  output wire                   c_rvalid,

  // ---- write port (B) ----
  input  wire                   c_we_en,
  input  wire                   c_we,
  input  wire [ROW_W-1:0]       c_wrow,
  input  wire [COL_W-1:0]       c_wcol,
  input  wire [DATA_W-1:0]      c_wdata,
  input  wire [BYTE_W-1:0]      c_wmask
);

  function automatic [ADDR_W-1:0] lin_addr;
    input [ROW_W-1:0] row;
    input [COL_W-1:0] col;
    begin
      lin_addr = row * N + col; // row-major
    end
  endfunction

  wire [ADDR_W-1:0] a_addr = lin_addr(c_row,  c_col);
  wire [ADDR_W-1:0] b_addr = lin_addr(c_wrow, c_wcol);

  wire a_en_i = c_en && c_re;
  wire b_en_i = c_we_en && c_we;

  // 你現有的 SRAM model：sram_word_ab
  sram_word_abD #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_mem2 (
    .clk(clk),
    .rst(rst),

    // Port A read-only
    .a_en     (a_en_i),
    .a_re     (c_re),
    .a_addr   (a_addr),
    .a_rdata  (c_rdata),
    .a_rvalid (c_rvalid),

    // Port B write-only
    .b_en     (b_en_i),
    .b_we     (c_we),
    .b_addr   (b_addr),
    .b_wdata  (c_wdata),
    .b_wmask  (c_wmask)
  );

endmodule

`default_nettype wire
`endif