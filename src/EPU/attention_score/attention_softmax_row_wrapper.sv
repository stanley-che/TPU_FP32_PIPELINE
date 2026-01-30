// ============================================================================
// attention_softmax_row_wrapper.sv
// - Wraps existing attention_softmax_top
// - Converts p_re / p_rdata into row-stream interface
// - ZERO change to original softmax
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module attention_softmax_row_wrapper #(
  parameter int unsigned T      = 8,
  parameter int unsigned DATA_W = 32,
  localparam int unsigned T_W   = (T<=1)?1:$clog2(T)
)(
  input  logic clk,
  input  logic rst_n,

  input  logic start,
  output logic busy,
  output logic done,

  // original softmax ports
  output logic        p_re,
  output logic [T_W-1:0] p_tq,
  output logic [T_W-1:0] p_tk,
  input  logic [DATA_W-1:0] p_rdata,
  input  logic        p_rvalid,

  // row-stream output
  output logic              p_row_valid,
  output logic [T_W-1:0]    p_row_idx,
  output logic [T*DATA_W-1:0] p_row_flat
);

  typedef enum logic [1:0] {
    S_IDLE,
    S_REQ,
    S_WAIT,
    S_NEXT
  } st_t;

  st_t st;
  logic [T_W-1:0] rq, rk;

  always_comb begin
    p_re = 1'b0;
    p_tq = rq;
    p_tk = rk;
    p_row_valid = 1'b0;

    if (st == S_REQ)
      p_re = 1'b1;
    if (st == S_WAIT && p_rvalid)
      p_row_valid = 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE;
      rq <= '0;
      rk <= '0;
      p_row_flat <= '0;
      p_row_idx <= '0;
    end else begin
      case (st)
        S_IDLE: begin
          if (start) begin
            rq <= 0;
            rk <= 0;
            st <= S_REQ;
          end
        end

        S_REQ: st <= S_WAIT;

        S_WAIT: begin
          if (p_rvalid) begin
            p_row_flat[rk*DATA_W +: DATA_W] <= p_rdata;
            if (rk == T-1) begin
              p_row_idx <= rq;
              rk <= 0;
              st <= S_NEXT;
            end else begin
              rk <= rk + 1'b1;
              st <= S_REQ;
            end
          end
        end

        S_NEXT: begin
          if (rq == T-1) st <= S_IDLE;
          else begin
            rq <= rq + 1'b1;
            st <= S_REQ;
          end
        end
      endcase
    end
  end

  assign busy = (st != S_IDLE);
  assign done = (st == S_IDLE);

endmodule

`default_nettype wire
