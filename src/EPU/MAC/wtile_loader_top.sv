`include "./src/EPU/MAC/w_sram_to_Wtile_col.sv"
`include "./src/EPU/MAC/axi_sram_slave.sv"
`timescale 1ns/1ps
`default_nettype none


module wtile_loader_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = DATA_W/8,
  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX),
  parameter int unsigned CONFLICT_POLICY = 1
)(
  input  logic clk,
  input  logic rst,

  // -------- loader control --------
  input  logic           start_k,
  input  logic [K_W-1:0] k_idx,
  output logic           col_valid,
  input  logic           col_accept,

  // CPU/DRAM -> SRAM write port
  input  logic               cpu_w_we,
  input  logic [ROW_W-1:0]   cpu_w_row,
  input  logic [K_W-1:0]     cpu_w_k,
  input  logic [DATA_W-1:0]  cpu_w_wdata,
  input  logic [BYTE_W-1:0]  cpu_w_wmask,

  // -------- output tile (FLAT) --------
  output logic [M*KMAX*DATA_W-1:0] W_tile_flat
);

  // loader raw outputs (to be muxed)
  logic                 w_en_l, w_re_l, w_we_l;
  logic [ROW_W-1:0]     w_row_l;
  logic [K_W-1:0]       w_k_l;
  logic [DATA_W-1:0]    w_wdata_l;
  logic [BYTE_W-1:0]    w_wmask_l;

  // muxed to SRAM
  logic                 w_en, w_re, w_we;
  logic [ROW_W-1:0]     w_row;
  logic [K_W-1:0]       w_k;
  logic [DATA_W-1:0]    w_wdata;
  logic [BYTE_W-1:0]    w_wmask;

  logic [DATA_W-1:0]    w_rdata;
  logic                 w_rvalid;

  // CPU priority when cpu_w_we=1
  always_comb begin
    if (cpu_w_we) begin
      w_en    = 1'b1;
      w_re    = 1'b0;
      w_we    = 1'b1;
      w_row   = cpu_w_row;
      w_k     = cpu_w_k;
      w_wdata = cpu_w_wdata;
      w_wmask = cpu_w_wmask;
    end else begin
      w_en    = w_en_l;
      w_re    = w_re_l;
      w_we    = w_we_l;
      w_row   = w_row_l;
      w_k     = w_k_l;
      w_wdata = w_wdata_l;
      w_wmask = w_wmask_l;
    end
  end

  // Loader: SRAM -> W_tile_flat
  w_sram_to_Wtile_col #(
    .M(M), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .K_W(K_W)
  ) u_loader (
    .clk(clk),
    .rst(rst),

    .start_k(start_k),
    .k_idx(k_idx),
    .col_valid(col_valid),
    .col_accept(col_accept),

    .w_en   (w_en_l),
    .w_re   (w_re_l),
    .w_we   (w_we_l),
    .w_row  (w_row_l),
    .w_k    (w_k_l),
    .w_wdata(w_wdata_l),
    .w_wmask(w_wmask_l),

    .w_rdata (w_rdata),
    .w_rvalid(w_rvalid),

    .W_tile_flat(W_tile_flat)
  );

  // SRAM: row/k
  sram_mem_mn #(
    .M(M), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_sram (
    .clk(clk),
    .rst(rst),

    .w_en   (w_en),
    .w_re   (w_re),
    .w_we   (w_we),
    .w_row  (w_row),
    .w_k    (w_k),
    .w_wdata(w_wdata),
    .w_wmask(w_wmask),

    .w_rdata (w_rdata),
    .w_rvalid(w_rvalid)
  );

endmodule

`default_nettype wire
