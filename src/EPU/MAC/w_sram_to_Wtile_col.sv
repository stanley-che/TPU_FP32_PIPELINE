`timescale 1ns/1ps
`default_nettype none

module w_sram_to_Wtile_col #(
  parameter int unsigned M      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = DATA_W/8,
  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // control
  input  logic            start_k,      // pulse: load column k_idx
  input  logic [K_W-1:0]  k_idx,
  output logic            col_valid,     // level: this k done
  input  logic            col_accept,    // pulse: consumer accepts

  // SRAM port (read-only)
  output logic                 w_en,
  output logic                 w_re,
  output logic                 w_we,
  output logic [ROW_W-1:0]     w_row,
  output logic [K_W-1:0]       w_k,
  output logic [DATA_W-1:0]    w_wdata,
  output logic [BYTE_W-1:0]    w_wmask,
  input  logic [DATA_W-1:0]    w_rdata,
  input  logic                 w_rvalid,

  //  output tile (FLAT packed bus)
  output logic [M*KMAX*DATA_W-1:0] W_tile_flat
);

  // ------------------------
  // helpers
  // ------------------------
  function automatic int unsigned w_idx(input int unsigned row, input int unsigned kk);
    w_idx = (row*KMAX + kk) * DATA_W;
  endfunction

  typedef enum logic [1:0] {IDLE, REQ, WAIT, HOLD} st_t;
  st_t st;

  logic [ROW_W-1:0] row_ptr;
  logic [ROW_W-1:0] row_issued;
  logic [K_W-1:0]   k_latched;

  // ------------------------
  // tie-offs (read-only)
  // ------------------------
  always_comb begin
    w_we    = 1'b0;
    w_wdata = '0;
    w_wmask = '0;
  end

  // ------------------------
  // main FSM
  // ------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      st         <= IDLE;
      row_ptr    <= '0;
      row_issued <= '0;
      k_latched  <= '0;

      // ✅ reset tile to 0 (KMAX 大會花 sim 時間；但你現在本來也有做)
      W_tile_flat <= '0;

    end else begin
      case (st)
        IDLE: begin
          row_ptr <= '0;
          if (start_k) begin
            k_latched <= k_idx;   // latch k
            st <= REQ;
          end
        end

        REQ: begin
          row_issued <= row_ptr;  // latch which row this request is for
          st <= WAIT;
        end

        WAIT: begin
          if (w_rvalid) begin
            int unsigned idx;
            idx = w_idx(row_issued, k_latched);
            W_tile_flat[idx +: DATA_W] <= w_rdata;

            if (row_issued == M-1) st <= HOLD;
            else begin
              row_ptr <= row_ptr + 1;
              st <= REQ;
            end
          end
        end

        HOLD: begin
          if (col_accept) st <= IDLE;
        end
      endcase
    end
  end

  // ------------------------
  // combinational outputs
  // ------------------------
  always_comb begin
    w_en = 1'b0;
    w_re = 1'b0;
    w_row = row_ptr;
    w_k   = k_latched;

    col_valid = (st == HOLD);

    if (st == REQ) begin
      w_en = 1'b1;
      w_re = 1'b1;
    end
  end

endmodule

`default_nettype wire
