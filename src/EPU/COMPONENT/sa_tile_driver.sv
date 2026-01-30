`include "./src/EPU/attention_score/systolic_array_os.sv"
`timescale 1ns/1ps
`default_nettype none


module sa_tile_driver_flat #(
  parameter int M    = 8,
  parameter int N    = 8,
  parameter int KMAX = 1024
)(
  input  logic clk,
  input  logic rst,

  input  logic        tile_start,
  input  logic [15:0] K_len,
  output logic        tile_busy,
  output logic        tile_done,

  input  logic [M*KMAX*32-1:0] W_tile_flat,
  input  logic [KMAX*N*32-1:0] X_tile_flat,

  output logic              step_valid,
  output logic [M*32-1:0]   a_row_flat,
  output logic [N*32-1:0]   b_col_flat,
  output logic              k_first,
  output logic              k_last,
  input  logic              step_ready,

  input  logic [M*N*32-1:0] c_out_flat,
  input  logic [M*N-1:0]    c_valid_flat
);

  function automatic int w_off(input int i, input int k);
    return (i*KMAX + k);
  endfunction
  function automatic int x_off(input int k, input int j);
    return (k*N + j);
  endfunction

  typedef enum logic [1:0] {IDLE, ISSUE, WAIT_STEP, FINISH} state_t;
  state_t st;

  int unsigned kk;
  logic [15:0] K_len_lat;

  // NEW: track acceptance (step_ready must go low once)
  logic accepted;

  always_ff @(posedge clk) begin
    if (rst) begin
      st        <= IDLE;
      kk        <= 0;
      K_len_lat <= '0;

      tile_busy <= 1'b0;
      tile_done <= 1'b0;

      step_valid <= 1'b0;
      k_first    <= 1'b0;
      k_last     <= 1'b0;
      a_row_flat <= '0;
      b_col_flat <= '0;

      accepted   <= 1'b0;

    end else begin
      tile_done  <= 1'b0;
      step_valid <= 1'b0;
      k_first    <= 1'b0;
      k_last     <= 1'b0;

      case (st)
        IDLE: begin
          tile_busy <= 1'b0;
          kk        <= 0;
          accepted  <= 1'b0;

          if (tile_start) begin
            tile_busy <= 1'b1;
            K_len_lat <= K_len;

            // guard: K_len==0 -> done immediately
            if (K_len == 0) begin
              st <= FINISH;
            end else begin
              st <= ISSUE;
            end
          end
        end

        ISSUE: begin
          if (step_ready === 1'b1) begin
            int i, j;

            for (i = 0; i < M; i++)
              a_row_flat[i*32 +: 32] <= W_tile_flat[w_off(i, kk)*32 +: 32];

            for (j = 0; j < N; j++)
              b_col_flat[j*32 +: 32] <= X_tile_flat[x_off(kk, j)*32 +: 32];

            k_first    <= (kk == 0);
            k_last     <= (kk == (K_len_lat-1));
            step_valid <= 1'b1;

            accepted  <= 1'b0;
            st        <= WAIT_STEP;
          end
        end

        WAIT_STEP: begin
          // phase-1: wait SA to leave IDLE (ready goes low) => accepted
          if (!accepted) begin
            if (step_ready === 1'b0) begin
              accepted <= 1'b1;
            end
          end else begin
            // phase-2: wait SA to return IDLE (ready high) => done
            if (step_ready === 1'b1) begin
              if (kk + 1 >= K_len_lat) begin
                st <= FINISH;
              end else begin
                kk <= kk + 1;
                st <= ISSUE;
              end
            end
          end
        end

        FINISH: begin
          tile_busy <= 1'b0;
          tile_done <= 1'b1;
          st <= IDLE;
        end

        default: st <= IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
