`include "./src/EPU/transpose/single_bank_sram.sv"
`include "./src/EPU/transpose/Round_robin.sv"
`timescale 1ns/1ps
`default_nettype none

module bank_sram #(
  parameter integer NB      = 8,   // number of banks
  parameter integer ADDR_W  = 12,
  parameter integer Data_W  = 16,
  parameter integer M       = 8    // number of request streams
)(
  input  wire                         clk,
  input  wire                         rst_n,

  // M streams (flattened for Verilog-2001)
  input  wire [M-1:0]                 req_v,        // request valid
  input  wire [M-1:0]                 req_we,       // request write enable
  input  wire [M*ADDR_W-1:0]          Req_addr,     // [m*ADDR_W +: ADDR_W]
  input  wire [M*Data_W-1:0]          Req_wData,    // [m*Data_W +: Data_W]
  output reg  [M-1:0]                 req_ready,    // OR of per-bank grants (same-cycle)
  output reg  [M*Data_W-1:0]          Rsp_rData,    // read data (aligned to 1-cycle SRAM)
  output reg  [M-1:0]                 rsp_v         // response valid (1-cycle after grant)
);

  // ==============================================================
  // utilities
  // ==============================================================
  function integer clog2;
    input integer value;
    integer i;
    begin
      clog2 = 0;
      for (i = value-1; i > 0; i = i >> 1) clog2 = clog2 + 1;
    end
  endfunction

  localparam integer BANK_W = clog2(NB);   // low bits select bank

  // slice helpers
  function [ADDR_W-1:0] addr_m;
    input [M*ADDR_W-1:0] bus; input integer m;
    begin addr_m = bus[m*ADDR_W +: ADDR_W]; end
  endfunction

  function [Data_W-1:0] data_m;
    input [M*Data_W-1:0] bus; input integer m;
    begin data_m = bus[m*Data_W +: Data_W]; end
  endfunction

  // ==============================================================
  // stitch matrices from all banks so we can OR / mux outside
  // gnt_now_mat : [b*M +: M]  = current-cycle grants from bank b
  // gnt_d_mat   : [b*M +: M]  = 1-cycle delayed grants from bank b
  // rdb_mat     : [b*Data_W +: Data_W] = rdata from bank b (SRAM 1-cycle)
  // ==============================================================
  wire [NB*M-1:0]         gnt_now_mat;
  wire [NB*M-1:0]         gnt_d_mat;
  wire [NB*Data_W-1:0]    rdb_mat;

  // ==============================================================
  // per-bank blocks
  // ==============================================================
  genvar b, m;
  generate
    for (b = 0; b < NB; b = b + 1) begin : G_BANK
      // collect requests targeted to bank b
      wire [M-1:0] to_bank_req;
      for (m = 0; m < M; m = m + 1) begin : G_TO_BANK
        wire [ADDR_W-1:0] a_m = addr_m(Req_addr, m);
        assign to_bank_req[m] = req_v[m] & (a_m[BANK_W-1:0] == b[BANK_W-1:0]);
      end

      // RR arbiter M -> 1 at bank b
      wire [M-1:0] gnt_bank_now;
      wire         any_gnt_bank;
      RR_single #(.N(M)) u_rr (
        .clk      (clk),
        .rst_n    (rst_n),
        .req      (to_bank_req),
        .gnt      (gnt_bank_now),
        .gnt_flag (any_gnt_bank)
      );
		
      // M-to-1 select for the SRAM port
      reg                  we_bank;
      reg  [ADDR_W-1:0]    addr_bank;
      reg  [Data_W-1:0]    wData_bank;
      wire [Data_W-1:0]    rData_bank;

      integer k;
      always @* begin
        we_bank   = 1'b0;
        addr_bank = {ADDR_W{1'b0}};
        wData_bank= {Data_W{1'b0}};
        for (k = 0; k < M; k = k + 1) begin
          if (gnt_bank_now[k]) begin
            we_bank    = req_we[k];
            // bank address = remove low BANK_W bits
            addr_bank  = addr_m(Req_addr, k) >> BANK_W;
            wData_bank = data_m(Req_wData, k);
          end
        end
      end

      // single-bank SRAM (sync read, 1-cycle)
      single_bank_sram #(
        .ADDR_W (ADDR_W - BANK_W),
        .Data_W (Data_W)
      ) u_sram_single (
        .clk   (clk),
        .rst_n (rst_n),
        .we    (we_bank),
        .cs    (any_gnt_bank),
        .addr  (addr_bank[ADDR_W-BANK_W-1:0]),
        .rdata (rData_bank),
        .wdata (wData_bank)
      );

      // 1-cycle delay of grants to align with rData_bank
      reg [M-1:0] gnt_bank_d;
      always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) gnt_bank_d <= {M{1'b0}};
        else        gnt_bank_d <= gnt_bank_now;
      end

      // stitch into global matrices
      assign gnt_now_mat[b*M +: M]     = gnt_bank_now;
      assign gnt_d_mat  [b*M +: M]     = gnt_bank_d;
      assign rdb_mat    [b*Data_W +: Data_W] = rData_bank;
    end
  endgenerate

  // ==============================================================
  // Fold per-bank signals into per-master outputs
  // req_ready[m] : OR of same-cycle grants from any bank
  // rsp_v[m]     : OR of 1-cycle-delayed grants (at most one bank true)
  // Rsp_rData[m] : mux rData from the bank that had gnt_d=1 for this m
  // ==============================================================
  integer mi, bi;
  reg [Data_W-1:0] rdata_mux;

  always @* begin
    // defaults
    req_ready  = {M{1'b0}};
    rsp_v      = {M{1'b0}};
    Rsp_rData  = {M*Data_W{1'b0}};

    for (mi = 0; mi < M; mi = mi + 1) begin
      // OR of current grants for req_ready
      for (bi = 0; bi < NB; bi = bi + 1) begin
        req_ready[mi] = req_ready[mi] | gnt_now_mat[bi*M + mi];
      end

      // 1-cycle response valid + data mux
      rdata_mux = {Data_W{1'b0}};
      for (bi = 0; bi < NB; bi = bi + 1) begin
        if (gnt_d_mat[bi*M + mi]) begin
          rsp_v[mi]   = 1'b1;
          rdata_mux   = rdb_mat[bi*Data_W +: Data_W];
        end
      end
      Rsp_rData[mi*Data_W +: Data_W] = rdata_mux;
    end
  end
	
endmodule

`default_nettype wire

