`ifndef RV_SYNC_FIFO_SV
`define RV_SYNC_FIFO_SV

`timescale 1ns/1ps
`default_nettype none

module rv_sync_fifo #(
  parameter int unsigned DW    = 32,
  parameter int unsigned DEPTH = 1024  // power-of-2 recommended
)(
  input  logic          clk,
  input  logic          rst,
  input  logic          en,

  input  logic          in_valid,
  output logic          in_ready,
  input  logic [DW-1:0] in_data,

  output logic          out_valid,
  input  logic          out_ready,
  output logic [DW-1:0] out_data,

  output logic          full,
  output logic          empty,
  output logic [$clog2(DEPTH+1)-1:0] level
);

  localparam int unsigned AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);

  logic [DW-1:0] mem [0:DEPTH-1];
  logic [AW-1:0] wptr, rptr;
  logic [AW:0]   count;

  wire push = en && in_valid && in_ready;
  wire pop  = en && out_valid && out_ready;

  always_comb begin
    full      = (count == DEPTH);
    empty     = (count == 0);
    in_ready  = en && !full;
    out_valid = en && !empty;
    out_data  = mem[rptr];     // FWFT
    level     = count[$clog2(DEPTH+1)-1:0];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      wptr  <= '0;
      rptr  <= '0;
      count <= '0;
    end else if (en) begin
      if (push) begin
        mem[wptr] <= in_data;
        wptr <= wptr + 1'b1;
      end
      if (pop) begin
        rptr <= rptr + 1'b1;
      end
      case ({push,pop})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase
    end
  end

endmodule

`default_nettype wire
`endif