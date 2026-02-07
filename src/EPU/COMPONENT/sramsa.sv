// tile_compute_system_top.sv
`include "./src/EPU/attention_score/tile_system_top.sv"
`include "./src/EPU/attention_score/systolic_wrap_c_sram.sv"
`timescale 1ns/1ps
`default_nettype none

module tile_compute_system_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned CONFLICT_POLICY_C = 1,

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N),
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // ---------------- command ----------------
  input  logic        start,
  input  logic [15:0] K_len,
  output logic        busy,
  output logic        done,

  // ---------------- CPU write W SRAM ----------------
  input  logic                 cpu_w_we,
  input  logic [ROW_W-1:0]     cpu_w_row,
  input  logic [K_W-1:0]       cpu_w_k,
  input  logic [DATA_W-1:0]    cpu_w_wdata,
  input  logic [BYTE_W-1:0]    cpu_w_wmask,

  // ---------------- CPU write X SRAM ----------------
  input  logic                 cpu_x_we,
  input  logic [K_W-1:0]       cpu_x_k,
  input  logic [N_W-1:0]       cpu_x_n,
  input  logic [DATA_W-1:0]    cpu_x_wdata,
  input  logic [BYTE_W-1:0]    cpu_x_wmask,

  // ---------------- C SRAM CPU read ----------------
  input  logic              c_rd_en,
  input  logic              c_rd_re,
  input  logic [ROW_W-1:0]  c_rd_row,
  input  logic [COL_W-1:0]  c_rd_col,
  output logic [DATA_W-1:0] c_rd_rdata,
  output logic              c_rd_rvalid,

  // ---------------- debug exports ----------------
  output logic [M*N*DATA_W-1:0] c_out_flat_o,
  output logic [M*N-1:0]        c_valid_flat_o,
  output logic                  C_valid,

  // optional: expose tiles (handy for waveform)
  output logic [M*KMAX*DATA_W-1:0] W_tile_flat_dbg,
  output logic [KMAX*N*DATA_W-1:0] X_tile_flat_dbg
);

  // tiles from tile_system_top
  wire [M*KMAX*DATA_W-1:0] W_tile_flat;
  wire [KMAX*N*DATA_W-1:0] X_tile_flat;

  // ---------------- tile loader system ----------------
  tile_system_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .N_W(N_W), .K_W(K_W)
  ) u_tile (
    .clk(clk),
    .rst(rst),

    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done),

    .cpu_w_we(cpu_w_we),
    .cpu_w_row(cpu_w_row),
    .cpu_w_k(cpu_w_k),
    .cpu_w_wdata(cpu_w_wdata),
    .cpu_w_wmask(cpu_w_wmask),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat)
  );

  // ---------------- systolic + C SRAM ctrl ----------------
  systolic_wrap_c_sram #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .ROW_W(ROW_W), .COL_W(COL_W)
  ) u_wrap (
    .clk(clk),
    .rst(rst),

    .start(done),
    .K_len(K_len),
    .busy(/* unused: tile already has busy */),
    .done(/* unused: tile already has done */),

    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .c_out_flat_o(c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid(C_valid)
  );

  // debug mirrors
  always_comb begin
    W_tile_flat_dbg = W_tile_flat;
    X_tile_flat_dbg = X_tile_flat;
  end

endmodule

`default_nettype wire
