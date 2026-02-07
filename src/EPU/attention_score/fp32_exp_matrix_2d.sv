// ============================================================================
// fp32_exp_matrix_2d.sv
// Apply exp(x) to each element of a 2D matrix (flattened 1D bus), sequentially
// Uses single fp32_exp_no_lut core
//
// Flatten order: row-major
//   idx = r*COLS + c
//   in_flat [DATA_W*idx +: DATA_W]  = element(r,c)
// ============================================================================

`include "./src/EPU/attention_score/fp32_exp_no_lut.sv"
`timescale 1ns/1ps
`default_nettype none

module fp32_exp_matrix_flat #(
  parameter int unsigned ROWS   = 4,
  parameter int unsigned COLS   = 4,
  parameter int unsigned DATA_W = 32,

  localparam int unsigned N      = ROWS*COLS,
  localparam int unsigned ROW_W  = (ROWS <= 1) ? 1 : $clog2(ROWS),
  localparam int unsigned COL_W  = (COLS <= 1) ? 1 : $clog2(COLS),
  localparam int unsigned IDX_W  = (N    <= 1) ? 1 : $clog2(N)
)(
  input  logic clk,
  input  logic rst_n,

  input  logic start,
  output logic busy,
  output logic done,

  input  logic [DATA_W*N-1:0] in_flat,
  output logic [DATA_W*N-1:0] out_flat,

  output logic [ROW_W-1:0] cur_r,
  output logic [COL_W-1:0] cur_c,
  output logic [IDX_W-1:0] cur_idx
);

  // ----------------------------
  // exp core
  // ----------------------------
  logic        exp_in_valid;
  logic [31:0] exp_in_fp32;
  logic        exp_out_valid;
  logic [31:0] exp_out_fp32;

  fp32_exp_no_lut u_exp (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (exp_in_valid),
    .in_fp32  (exp_in_fp32),
    .out_valid(exp_out_valid),
    .out_fp32 (exp_out_fp32)
  );

  // ----------------------------
  // FSM
  // ----------------------------
  typedef enum logic [2:0] {
    S_IDLE,
    S_SEND,
    S_WAIT,
    S_STORE,
    S_DONE
  } state_t;

  state_t st;

  // indices
  logic [ROW_W-1:0] r;
  logic [COL_W-1:0] c;
  logic [IDX_W-1:0] idx;

  assign cur_r   = r;
  assign cur_c   = c;
  assign cur_idx = idx;

  // helper: idx = r*COLS + c (combinational)
  // (ROWS/COLS 小時這很OK；要更硬可用 counter idx++ 取代乘法)
  function automatic [IDX_W-1:0] rc_to_idx(input logic [ROW_W-1:0] rr,
                                           input logic [COL_W-1:0] cc);
    rc_to_idx = rr*COLS + cc;
  endfunction

  // ----------------------------
  // sequential
  // ----------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st           <= S_IDLE;
      busy         <= 1'b0;
      done         <= 1'b0;
      exp_in_valid <= 1'b0;
      exp_in_fp32  <= 32'd0;

      r   <= '0;
      c   <= '0;
      idx <= '0;

      out_flat <= '0;
    end else begin
      done         <= 1'b0;
      exp_in_valid <= 1'b0;

      case (st)
        S_IDLE: begin
          busy <= 1'b0;
          if (start) begin
            r   <= '0;
            c   <= '0;
            idx <= '0;
            busy <= 1'b1;
            st   <= S_SEND;
          end
        end

        S_SEND: begin
          idx         <= rc_to_idx(r, c);
          exp_in_fp32 <= in_flat[DATA_W*rc_to_idx(r,c) +: DATA_W];
          exp_in_valid <= 1'b1;          // 1-cycle pulse
          st <= S_WAIT;
        end

        S_WAIT: begin
          if (exp_out_valid) begin
            st <= S_STORE;
          end
        end

        S_STORE: begin
          out_flat[DATA_W*idx +: DATA_W] <= exp_out_fp32;

          if (c == COLS-1) begin
            c <= '0;
            if (r == ROWS-1) begin
              st <= S_DONE;
            end else begin
              r  <= r + 1'b1;
              st <= S_SEND;
            end
          end else begin
            c  <= c + 1'b1;
            st <= S_SEND;
          end
        end

        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;   // 1-cycle
          st   <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end



endmodule

`default_nettype wire
