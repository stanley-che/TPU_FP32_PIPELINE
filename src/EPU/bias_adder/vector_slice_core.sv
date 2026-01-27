`timescale 1ns/1ps
`default_nettype none
module vector_slice_core #(
  parameter int unsigned M=8, N=8,
  parameter int unsigned DATA_W=32,
  parameter int unsigned ROW_W=(M<=1)?1:$clog2(M),
  parameter int unsigned COL_W=(N<=1)?1:$clog2(N),
  parameter int unsigned VADDR_W=$clog2((M>N)?M:N)
)(
  input  logic clk, rst,

  input  logic        cfg_we,
  input  logic        mode_rowcol,   // 0=row, 1=col
  input  logic [ROW_W-1:0] row_idx,
  input  logic [COL_W-1:0] col_idx,
  input  logic [7:0]  len_cfg,        // 0 => default

  input  logic start,
  output logic busy,
  output logic done,
  output logic v_valid,

  // ---- source matrix read port (connect to X SRAM or C SRAM) ----
  output logic             src_rd_en,
  output logic [ROW_W-1:0] src_rd_row,
  output logic [COL_W-1:0] src_rd_col,
  input  logic [DATA_W-1:0] src_rd_rdata,
  input  logic              src_rd_rvalid,

  // ---- V-SRAM write port ----
  output logic             v_we,
  output logic [VADDR_W-1:0] v_waddr,
  output logic [DATA_W-1:0]  v_wdata
);

  logic mode_q;
  logic [ROW_W-1:0] row_q;
  logic [COL_W-1:0] col_q;
  logic [7:0] len_q;

  logic [7:0] k;          // counter
  logic [7:0] len_eff;

  always_ff @(posedge clk) begin
    if (rst) begin
      mode_q <= 1'b0; row_q <= '0; col_q <= '0; len_q <= 8'd0;
    end else if (cfg_we) begin
      mode_q <= mode_rowcol;
      row_q  <= row_idx;
      col_q  <= col_idx;
      len_q  <= len_cfg;
    end
  end

  always_comb begin
    if (len_q != 0) len_eff = len_q;
    else            len_eff = (mode_q==1'b0) ? N : M; // row->N, col->M
  end

  typedef enum logic [1:0] {IDLE, ISSUE, WAIT, FINISH} st_t;
  st_t st;

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= IDLE;
      busy <= 1'b0; done <= 1'b0; v_valid <= 1'b0;
      src_rd_en <= 1'b0;
      v_we <= 1'b0;
      k <= 0;
    end else begin
      done <= 1'b0;
      src_rd_en <= 1'b0;
      v_we <= 1'b0;

      case (st)
        IDLE: begin
          busy <= 1'b0;
          if (start) begin
            busy <= 1'b1;
            v_valid <= 1'b0;
            k <= 0;
            st <= ISSUE;
          end
        end

        ISSUE: begin
          src_rd_en <= 1'b1;
          if (mode_q==1'b0) begin
            src_rd_row <= row_q;
            src_rd_col <= k[COL_W-1:0];
          end else begin
            src_rd_row <= k[ROW_W-1:0];
            src_rd_col <= col_q;
          end
          st <= WAIT;
        end

        WAIT: begin
          if (src_rd_rvalid) begin
            v_we    <= 1'b1;
            v_waddr <= k[VADDR_W-1:0];
            v_wdata <= src_rd_rdata;

            if (k + 1 >= len_eff) st <= FINISH;
            else begin
              k <= k + 1;
              st <= ISSUE;
            end
          end
        end

        FINISH: begin
          busy <= 1'b0;
          done <= 1'b1;
          v_valid <= 1'b1;
          st <= IDLE;
        end
      endcase
    end
  end
endmodule
`default_nettype wire
