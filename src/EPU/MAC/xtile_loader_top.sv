`include "./src/EPU/MAC/x_sram_to_Xtile_row.sv"
`include "./src/EPU/MAC/sram_mem_kn.sv"
`timescale 1ns/1ps
`default_nettype none

module xtile_loader_top #(
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = DATA_W/8,
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX),
  parameter int unsigned CONFLICT_POLICY = 1
)(
  input  logic clk,
  input  logic rst,

  input  logic           start_k,
  input  logic [K_W-1:0] k_idx,
  output logic           row_valid,
  input  logic           row_accept,

  // CPU/DRAM -> X SRAM write port
  input  logic               cpu_x_we,
  input  logic [K_W-1:0]     cpu_x_k,
  input  logic [N_W-1:0]     cpu_x_n,
  input  logic [DATA_W-1:0]  cpu_x_wdata,
  input  logic [BYTE_W-1:0]  cpu_x_wmask,

  // output tile (FLAT): idx=(k*N+n)*DATA_W
  output logic [KMAX*N*DATA_W-1:0] X_tile_flat
);

  // loader raw outputs
  logic                 x_en_l, x_re_l, x_we_l;
  logic [K_W-1:0]       x_k_l;
  logic [N_W-1:0]       x_n_l;
  logic [DATA_W-1:0]    x_wdata_l;
  logic [BYTE_W-1:0]    x_wmask_l;

  // muxed to SRAM
  logic                 x_en, x_re, x_we;
  logic [K_W-1:0]       x_k;
  logic [N_W-1:0]       x_n;
  logic [DATA_W-1:0]    x_wdata;
  logic [BYTE_W-1:0]    x_wmask;

  logic [DATA_W-1:0]    x_rdata;
  logic                 x_rvalid;

  // CPU priority
  assign x_en    = cpu_x_we ? 1'b1        : x_en_l;
  assign x_re    = cpu_x_we ? 1'b0        : x_re_l;
  assign x_we    = cpu_x_we ? 1'b1        : x_we_l;
  assign x_k     = cpu_x_we ? cpu_x_k     : x_k_l;
  assign x_n     = cpu_x_we ? cpu_x_n     : x_n_l;
  assign x_wdata = cpu_x_we ? cpu_x_wdata : x_wdata_l;
  assign x_wmask = cpu_x_we ? cpu_x_wmask : x_wmask_l;

  // Loader: SRAM -> X_tile_flat
  x_sram_to_Xtile_row #(
    .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .N_W(N_W), .K_W(K_W)
  ) u_loader (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .row_valid(row_valid),
    .row_accept(row_accept),

    .x_en(x_en_l),
    .x_re(x_re_l),
    .x_we(x_we_l),
    .x_k(x_k_l),
    .x_n(x_n_l),
    .x_wdata(x_wdata_l),
    .x_wmask(x_wmask_l),

    .x_rdata(x_rdata),
    .x_rvalid(x_rvalid),

    .X_tile_flat(X_tile_flat)
  );

  // SRAM: k/n
  sram_mem_kn #(
    .KMAX(KMAX), .N(N), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_sram1 (
    .clk(clk),
    .rst(rst),

    .x_en(x_en),
    .x_re(x_re),
    .x_we(x_we),
    .x_k(x_k),
    .x_n(x_n),
    .x_wdata(x_wdata),
    .x_wmask(x_wmask),

    .x_rdata(x_rdata),
    .x_rvalid(x_rvalid)
  );

endmodule

`default_nettype wire
