`timescale 1ns/1ps
`default_nettype none

module tile_load_controller #(
  parameter int unsigned KMAX = 1024,
  parameter int unsigned K_W  = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // ---- command ----
  input  logic        start,     // pulse
  input  logic [15:0] K_len,     // number of k to load (<= KMAX)
  output logic        busy,
  output logic        done,      // 1-cycle pulse when all k loaded

  // ---- W loader ----
  output logic           w_start_k,     // pulse
  output logic [K_W-1:0] w_k_idx,
  input  logic           w_col_valid,   // level
  output logic           w_col_accept,  // pulse

  // ---- X loader ----
  output logic           x_start_k,     // pulse
  output logic [K_W-1:0] x_k_idx,
  input  logic           x_row_valid,   // level
  output logic           x_row_accept   // pulse
);

  typedef enum logic [1:0] {IDLE, ISSUE, WAIT_VALID, ACCEPT} st_t;
  st_t st;

  logic [15:0] k_cnt;  // 0..K_len-1

  // continuous assigns
  assign w_k_idx = k_cnt[K_W-1:0];
  assign x_k_idx = k_cnt[K_W-1:0];

  // busy flag
  always_comb busy = (st != IDLE);

  // pulse outputs default
  always_comb begin
    w_start_k    = 1'b0;
    w_col_accept = 1'b0;
    x_start_k    = 1'b0;
    x_row_accept = 1'b0;

    if (st == ISSUE) begin
      w_start_k = 1'b1;
      x_start_k = 1'b1;
    end
    if (st == ACCEPT) begin
      w_col_accept = 1'b1;
      x_row_accept = 1'b1;
    end
  end

  // FSM
  always_ff @(posedge clk) begin
    if (rst) begin
      st    <= IDLE;
      k_cnt <= '0;
      done  <= 1'b0;
    end else begin
      done <= 1'b0; // default 0, pulse for 1 cycle only

      case (st)
        IDLE: begin
          k_cnt <= '0;
          if (start) st <= ISSUE;
        end

        ISSUE: begin
          // 1-cycle start_k pulse
          st <= WAIT_VALID;
        end

        WAIT_VALID: begin
          // wait until both loaders finished this k
          if (w_col_valid && x_row_valid) st <= ACCEPT;
        end

        ACCEPT: begin
          // 1-cycle accept pulse
          if ((k_cnt + 1) >= K_len) begin
            done <= 1'b1;
            st   <= IDLE;
          end else begin
            k_cnt <= k_cnt + 1;
            st    <= ISSUE;
          end
        end

        default: st <= IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
