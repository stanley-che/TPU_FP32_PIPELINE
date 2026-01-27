`timescale 1ns/1ps
`default_nettype none

module x_sram_to_Xtile_row #(
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = DATA_W/8,
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // control
  input  logic            start_k,      // pulse: load row k_idx
  input  logic [K_W-1:0]  k_idx,
  output logic            row_valid,     // level: this k done
  input  logic            row_accept,    // pulse: consumer accepts

  // SRAM port (read-only)
  output logic                 x_en,
  output logic                 x_re,
  output logic                 x_we,
  output logic [K_W-1:0]       x_k,
  output logic [N_W-1:0]       x_n,
  output logic [DATA_W-1:0]    x_wdata,
  output logic [BYTE_W-1:0]    x_wmask,
  input  logic [DATA_W-1:0]    x_rdata,
  input  logic                 x_rvalid,

  //output tile (FLAT packed bus): idx = (k*N + n)*DATA_W
  output logic [KMAX*N*DATA_W-1:0] X_tile_flat
);

  // ------------------------
  // helpers
  // ------------------------
  function automatic int unsigned x_idx(input int unsigned kk, input int unsigned nn);
    x_idx = (kk*N + nn) * DATA_W;
  endfunction

  // tie-offs (read-only)
  always_comb begin
    x_we    = 1'b0;
    x_wdata = '0;
    x_wmask = '0;
  end

  typedef enum logic [1:0] {IDLE, REQ, WAIT, HOLD} st_t;
  st_t st;

  logic [N_W-1:0] n_ptr;
  logic [N_W-1:0] n_issued;
  logic [K_W-1:0] k_latched;

  always_ff @(posedge clk) begin
    if (rst) begin
      st        <= IDLE;
      n_ptr     <= '0;
      n_issued  <= '0;
      k_latched <= '0;

      // ✅ reset tile flat
      X_tile_flat <= '0;

    end else begin
      case (st)
        IDLE: begin
          n_ptr <= '0;
          if (start_k) begin
            k_latched <= k_idx;
            st <= REQ;
          end
        end

        REQ: begin
          n_issued <= n_ptr;
          st <= WAIT;
        end

        WAIT: begin
          if (x_rvalid) begin
            int unsigned idx;
            idx = x_idx(k_latched, n_issued);
            X_tile_flat[idx +: DATA_W] <= x_rdata;

            if (n_issued == N-1) st <= HOLD;
            else begin
              n_ptr <= n_ptr + 1;
              st <= REQ;
            end
          end
        end

        HOLD: begin
          if (row_accept) st <= IDLE;
        end
      endcase
    end
  end

  always_comb begin
    x_en = 1'b0;
    x_re = 1'b0;

    x_k  = k_latched;
    x_n  = n_ptr;

    row_valid = (st == HOLD);

    if (st == REQ) begin
      x_en = 1'b1;
      x_re = 1'b1;
    end
  end

endmodule

`default_nettype wire
