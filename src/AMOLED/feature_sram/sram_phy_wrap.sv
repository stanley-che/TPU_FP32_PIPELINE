`ifndef SRAM_PHY_WRAP_SV
`define SRAM_PHY_WRAP_SV

`timescale 1ns/1ps
`default_nettype none

module sram_phy_wrap #(
  parameter int unsigned SRAM_BUS_W = 32,
  parameter int unsigned MEM_AW     = 18,
  parameter int unsigned RD_LAT     = 1,
  // for simulation/reg-mem fallback
  parameter int unsigned MEM_WORDS  = (1 << MEM_AW)
)(
  input  logic clk,

  // ---------- Command ----------
  input  logic                  mem_cmd_valid,
  output logic                  mem_cmd_ready,
  input  logic                  mem_cmd_we,      // 1=write, 0=read
  input  logic [MEM_AW-1:0]      mem_cmd_addr,
  input  logic [SRAM_BUS_W-1:0]  mem_cmd_wdata,
  input  logic [SRAM_BUS_W/8-1:0] mem_cmd_wmask, // 1=write that byte

  // ---------- Read return ----------
  output logic                  mem_rvalid,
  output logic [SRAM_BUS_W-1:0] mem_rdata
);

  // ------------------------------------------------------------
  // Ready policy:
  // - simplest: always ready (you can add stall later)
  // ------------------------------------------------------------
  assign mem_cmd_ready = 1'b1;

    // ------------------------------------------------------------
    // Memory array (fallback model)
    // Replace this block with SRAM macro/BRAM later.
    // ------------------------------------------------------------
    logic [SRAM_BUS_W-1:0] mem [0:MEM_WORDS-1];

  // ------------------------------------------------------------
  // Byte-mask write helper
  // ------------------------------------------------------------
  function automatic [SRAM_BUS_W-1:0] apply_wmask(
    input [SRAM_BUS_W-1:0] oldv,
    input [SRAM_BUS_W-1:0] newv,
    input [SRAM_BUS_W/8-1:0] wmask
  );
    integer b;
    reg [SRAM_BUS_W-1:0] tmp;
    begin
      tmp = oldv;
      for (b = 0; b < SRAM_BUS_W/8; b = b + 1) begin
        if (wmask[b]) tmp[b*8 +: 8] = newv[b*8 +: 8];
      end
      apply_wmask = tmp;
    end
  endfunction

  // ------------------------------------------------------------
  // Read latency pipeline
  // ------------------------------------------------------------
  localparam int unsigned LAT = (RD_LAT < 1) ? 1 : RD_LAT;

  logic [MEM_AW-1:0] rd_addr_q [0:LAT-1];
  logic              rd_vld_q  [0:LAT-1];

  integer i;

  // ------------------------------------------------------------
  // Main sequential
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    // default
    mem_rvalid <= 1'b0;

    // shift read pipeline
    for (i = LAT-1; i > 0; i = i - 1) begin
      rd_vld_q[i]  <= rd_vld_q[i-1];
      rd_addr_q[i] <= rd_addr_q[i-1];
    end
    rd_vld_q[0] <= 1'b0;

    // accept command
    if (mem_cmd_valid && mem_cmd_ready) begin
      if (mem_cmd_we) begin
        // WRITE: update memory (masked)
        mem[mem_cmd_addr] <= apply_wmask(mem[mem_cmd_addr], mem_cmd_wdata, mem_cmd_wmask);
      end else begin
        // READ: enqueue address into latency pipeline
        rd_vld_q[0]  <= 1'b1;
        rd_addr_q[0] <= mem_cmd_addr;
      end
    end

    // produce read return
    if (rd_vld_q[LAT-1]) begin
      mem_rvalid <= 1'b1;
      mem_rdata  <= mem[rd_addr_q[LAT-1]];
    end
  end
  
endmodule

`default_nettype wire
`endif
